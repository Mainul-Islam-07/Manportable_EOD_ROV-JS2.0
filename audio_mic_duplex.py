"""
Pi5 side: full-duplex audio (mic + soundbox) over UDP, with Speex echo
cancellation so the remote voice played on the soundbox is not picked up
by the mic and sent back.

Mic capture and speaker playback share one duplex callback, so the near-end
(mic) and far-end (played) signals are time-aligned -- which is what the
Speex AEC needs to work.
"""

import socket
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


print(f"Two-way audio with Speex AEC -> {PEER_IP}:{PORT}. Ctrl+C to stop.")
while True:
    try:
        device_lost.clear()
        stream = sd.Stream(samplerate=RATE, blocksize=FRAME_SIZE, dtype=DTYPE,
                           channels=CHANNELS, callback=callback,
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
        sd.sleep(int(RECONNECT_WAIT * 1000))
