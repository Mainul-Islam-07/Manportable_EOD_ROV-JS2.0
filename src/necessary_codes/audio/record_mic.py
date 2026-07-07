"""
Standalone recorder for the SIYI MK32 mic (or any input device), for later
noise analysis (e.g. quiet baseline vs. joystick handling noise).

Usage:
    python3 record_mic.py                # records DURATION_SEC seconds
    python3 record_mic.py 20              # records 20 seconds instead
    python3 record_mic.py 20 out.wav      # custom filename too

Suggested recording plan when you run it:
    0-5s   : silence / normal hold, no joystick movement   (baseline noise floor)
    5-10s  : talk normally, no joystick movement           (clean voice)
    10-15s : stay silent, move joysticks aggressively      (handling noise only)
    15-20s : talk WHILE moving joysticks                   (worst case, mixed)

That structure makes it easy to slice the WAV afterwards and compare
spectra segment by segment.
"""

import sys
import wave
import numpy as np
import sounddevice as sd

RATE = 16000          # match audio_mic_duplex.py
CHANNELS = 1
DTYPE = "int16"
DURATION_SEC = 20.0    # default length
OUT_FILE = "mic_recording.wav"


def list_devices():
    print("\nAvailable input devices:")
    for i, dev in enumerate(sd.query_devices()):
        if dev["max_input_channels"] > 0:
            print(f"  [{i}] {dev['name']}  "
                  f"(in ch: {dev['max_input_channels']}, "
                  f"default sr: {dev['default_samplerate']:.0f})")
    print()


def record(duration_sec, out_file, device=None):
    n_frames = int(duration_sec * RATE)
    print(f"Recording {duration_sec:.1f}s at {RATE} Hz -> {out_file}")
    print("Recording starts in 1 second...")
    sd.sleep(1000)

    audio = sd.rec(n_frames, samplerate=RATE, channels=CHANNELS,
                    dtype=DTYPE, device=device)

    # Live progress so you know where you are in the recording timeline.
    step = 1.0
    elapsed = 0.0
    while elapsed < duration_sec:
        sd.sleep(int(step * 1000))
        elapsed += step
        print(f"  {elapsed:4.1f}s / {duration_sec:.1f}s", flush=True)

    sd.wait()
    print("Recording finished.")

    with wave.open(out_file, "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(np.dtype(DTYPE).itemsize)  # 2 bytes for int16
        wf.setframerate(RATE)
        wf.writeframes(audio.tobytes())

    print(f"Saved to {out_file}")


if __name__ == "__main__":
    args = sys.argv[1:]

    if args and args[0] in ("-l", "--list"):
        list_devices()
        sys.exit(0)

    duration = float(args[0]) if len(args) >= 1 else DURATION_SEC
    out_file = args[1] if len(args) >= 2 else OUT_FILE

    list_devices()
    record(duration, out_file)
