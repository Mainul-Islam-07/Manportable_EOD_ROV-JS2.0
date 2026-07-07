#!/usr/bin/env python3
import os
import wave
import numpy as np
import subprocess


def play_louder(input_file, gain=4.0):
    with wave.open(input_file, 'rb') as wav:
        params = wav.getparams()
        frames = wav.readframes(params.nframes)

    audio = np.frombuffer(frames, dtype=np.int16).astype(np.float32)
    audio *= gain
    audio = np.clip(audio, -32768, 32767).astype(np.int16)

    out_file = "/tmp/louder.wav"
    with wave.open(out_file, 'wb') as out:
        out.setparams(params)
        out.writeframes(audio.tobytes())

    subprocess.run(["aplay", out_file])


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_path = os.path.join(script_dir, "mic_recording.wav")
    play_louder(input_path, gain=4.0)