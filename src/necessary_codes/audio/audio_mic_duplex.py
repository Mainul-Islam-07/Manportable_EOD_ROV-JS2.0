"""
Pi5 side: full-duplex audio (mic + soundbox) over UDP, with Speex echo
cancellation so the remote voice played on the soundbox is not picked up
by the mic and sent back.

Mic capture and speaker playback share one duplex callback, so the near-end
(mic) and far-end (played) signals are time-aligned -- which is what the
Speex AEC needs to work.
"""

import os
import socket
import sys
import queue
import threading

import numpy as np
import sounddevice as sd
from speexdsp import EchoCanceller

# --- Config ---
PEER_IP   = "192.168.144.20"  # IP of the OTHER device
PORT      = 5555             # both sides bind this port and send to it
RATE      = 16000            # Hz -- voice rate; AEC works best here
CHANNELS  = 1                # mono
FRAME_SIZE    = 256          # samples per block (also the AEC frame size)
FILTER_LENGTH = 2048         # echo tail length in samples (~128 ms @ 16 kHz)
DTYPE     = "int16"          # 2 bytes/sample
GAIN      = 4.0              # playback volume multiplier for the soundbox
RECONNECT_WAIT = 2.0         # seconds to wait before retrying a lost device

# Card name substrings, kept in step with audio_startup.sh. Only used for the
# log line below -- the stream itself goes through the sound server (see
# resolve_audio_device), which is what audio_startup.sh points at these cards.
MIC_MATCH = os.environ.get("MIC_MATCH", "UM02")
SPK_MATCH = os.environ.get("SPK_MATCH", "UACDemo")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", PORT))

# Jitter buffer of far-end (remote) audio waiting to be played.
playback_q = queue.Queue(maxsize=20)

# Speex acoustic echo canceller (runs on the Pi only).
echo_canceller = EchoCanceller.create(FRAME_SIZE, FILTER_LENGTH, RATE)


def receiver():
    """Receive remote audio and queue it for playback."""
    while True:
        data, _ = sock.recvfrom(4096)
        try:
            playback_q.put_nowait(data)
        except queue.Full:
            # Drop the oldest frame to keep latency low.
            try:
                playback_q.get_nowait()
            except queue.Empty:
                pass
            try:
                playback_q.put_nowait(data)
            except queue.Full:
                pass


def callback(indata, outdata, frames, time, status):
    if status:
        print(status, flush=True)

    # Far-end frame to play on the soundbox (silence if buffer is empty).
    try:
        far = playback_q.get_nowait()
    except queue.Empty:
        far = bytes(frames * 2)
    if len(far) != frames * 2:
        far = (far + bytes(frames * 2))[: frames * 2]

    # Amplify the far-end audio, clipping to int16 range so it doesn't wrap.
    far_amp = np.frombuffer(far, dtype=DTYPE).astype(np.float32) * GAIN
    np.clip(far_amp, -32768, 32767, out=far_amp)
    far_amp = far_amp.astype(DTYPE)

    outdata[:] = far_amp.reshape(-1, CHANNELS)

    # Cancel the echo of what is ACTUALLY played (amplified) from the mic,
    # then send the clean mic audio.
    clean = echo_canceller.process(indata.tobytes(), far_amp.tobytes())
    sock.sendto(clean, (PEER_IP, PORT))


# The receiver thread keeps draining the UDP socket no matter what the audio
# device is doing, so a mic/soundbox disconnect never kills the connection.
threading.Thread(target=receiver, daemon=True).start()

# Set when PortAudio finishes/aborts the stream -- e.g. the USB mic or soundbox
# was unplugged. Unplugging mid-stream does NOT raise into the main thread; the
# callback just stops firing, so we must watch for this explicitly.
device_lost = threading.Event()


def stream_finished():
    device_lost.set()


def resolve_audio_device():
    """Return the PortAudio index of the sound server device.

    The server is named "pipewire" on PipeWire systems and "pulse" on
    PulseAudio ones -- this Pi is PipeWire, so both are tried in that order.

    We deliberately bind the SERVER, not the USB card's raw ALSA hw: node.
    The mic does only 44.1/48 kHz and the speaker only 48 kHz, while this
    script wants 16 kHz mono -- only the sound server resamples that. Binding
    the card directly reproduces "Invalid sample rate [PaErrorCode -9997]"
    (see pi5_setup_commands.txt, section 11e).

    Which physical cards the server routes to is pinned by audio_startup.sh
    (pactl set-default-source/sink). Passing no device at all -- what this
    script used to do -- meant PortAudio picked ALSA's "default", whose
    routing depended on whatever the server had promoted at that instant.

    Falls back to "default" if neither is exposed (PortAudio builds vary).
    That is still better than passing no device at all, because
    audio_startup.sh has pinned the server's defaults by then. Returns None
    only if none of them exist, so the caller can fail loudly.

    Matching by NAME, not index, is deliberate: the indices are not stable.
    The same machine listed "default" at index 9 during boot and at index 1
    a few minutes later, because ALSA hides card devices that are already
    open.
    """
    def duplex(dev):
        return dev["max_input_channels"] > 0 and dev["max_output_channels"] > 0

    devices = list(sd.query_devices())
    for server in ("pipewire", "pulse"):
        for idx, dev in enumerate(devices):
            if server in dev["name"].lower() and duplex(dev):
                return idx
    for idx, dev in enumerate(devices):
        if dev["name"].lower().startswith("default") and duplex(dev):
            print("[audio] WARNING: no 'pipewire'/'pulse' device; falling back "
                  "to 'default'. Routing depends on the server defaults that "
                  "audio_startup.sh pinned.", file=sys.stderr, flush=True)
            return idx
    return None


print(f"Two-way audio with Speex AEC -> {PEER_IP}:{PORT}. Ctrl+C to stop.")
print(f"[audio] expecting mic '{MIC_MATCH}', speaker '{SPK_MATCH}' "
      f"(routing pinned by audio_startup.sh)", flush=True)

device = resolve_audio_device()
if device is None:
    # Exit rather than open whatever PortAudio would have picked. A
    # wrong-but-openable device opens cleanly and then runs silent forever --
    # no exception, no stream stop -- so neither the retry loop below nor the
    # unit's Restart=always ever fires. Failing here makes systemd retry.
    print("[audio] FATAL: no duplex sound-server device found. Is PulseAudio/"
          "PipeWire running in this user session? (check: pactl info)",
          file=sys.stderr, flush=True)
    sys.exit(1)

print(f"[audio] bound to device {device}: {sd.query_devices(device)['name']}",
      flush=True)

while True:
    try:
        device_lost.clear()
        stream = sd.Stream(samplerate=RATE, blocksize=FRAME_SIZE, dtype=DTYPE,
                           channels=CHANNELS, device=(device, device),
                           callback=callback,
                           finished_callback=stream_finished)
        with stream:
            print("Audio device open.", flush=True)
            # Poll instead of sleeping forever: stream.active goes False (and
            # device_lost fires) the moment the device disappears.
            while stream.active and not device_lost.is_set():
                sd.sleep(200)
        raise RuntimeError("audio device disconnected")
    except KeyboardInterrupt:
        print("\nStopped.")
        break
    except Exception as exc:
        # Device unplugged / not ready: wait and retry instead of crashing.
        print(f"[audio] device unavailable: {exc} -- waiting to reconnect...",
              flush=True)
        # Re-scan devices so a re-plugged USB mic/soundbox is detected. PortAudio
        # caches the device list at init, so without this a re-plugged device
        # (often at a new index) is invisible.
        try:
            sd._terminate()
            sd._initialize()
        except Exception:
            pass
        # The re-init invalidates device indices, so re-resolve rather than
        # reusing a stale one. Keep the old index if the rescan comes up empty
        # (server still restarting) -- the next loop will just retry.
        rescanned = resolve_audio_device()
        if rescanned is not None and rescanned != device:
            device = rescanned
            print(f"[audio] rebound to device {device}: "
                  f"{sd.query_devices(device)['name']}", flush=True)
        sd.sleep(int(RECONNECT_WAIT * 1000))
