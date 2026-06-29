# Live audio over UDP (Pi5)

Two-way (full-duplex) live audio between a Raspberry Pi 5 (mic + soundbox) and
another device, over UDP. Speex acoustic echo cancellation runs on the Pi only.

## Files

- `pi_duplex_aec.py` — run on the **Pi5**. Full duplex + Speex AEC.
- `peer_duplex.py` — run on the **other device**. Full duplex, no AEC.
- `udp_audio_sender.py` — simple one-way mic → UDP sender (no duplex/AEC).
- `fix_speexdsp.py` — re-applies the Python 3.12 patch for the `speexdsp` package.

## Setup (Pi side)

System packages (need sudo):

```bash
sudo apt update
sudo apt install -y libportaudio2 libspeexdsp-dev swig python3-venv
```

Python venv + packages:

```bash
python3 -m venv venv
source venv/bin/activate
pip install sounddevice numpy Cython
pip install speexdsp
python fix_speexdsp.py        # required on Python 3.12+ (see below)
```

The other device only needs `sounddevice` and `numpy`.

## Configure

- `pi_duplex_aec.py` → set `PEER_IP` to the other device's IP.
- `peer_duplex.py` → set `PEER_IP` to the **Pi's** IP.
- Both sides must use the same `RATE`, `FRAME_SIZE`, and `PORT` (default 16 kHz
  mono, 256-sample blocks, port 5005).

## Run

```bash
# on the other device
python peer_duplex.py

# on the Pi (activate venv first)
source venv/bin/activate
python pi_duplex_aec.py
```

## The `speexdsp` / Python 3.12 fix

The PyPI `speexdsp` wheel ships a SWIG wrapper that does `import imp`, a module
removed in Python 3.12. Importing it fails with:

```
ModuleNotFoundError: No module named 'imp'
```

`fix_speexdsp.py` rewrites that block to a modern import. It is idempotent.
**Re-run it any time you reinstall or recreate the venv**, e.g. after
`pip install --force-reinstall speexdsp`:

```bash
source venv/bin/activate
python fix_speexdsp.py
```

## Echo cancellation notes

- AEC works best when the mic and soundbox share one clock/device. A separate
  USB soundbox drifts against the mic clock and echo can creep back over time.
- If echo persists, raise `FILTER_LENGTH` in `pi_duplex_aec.py` (e.g. `4096`)
  to cover a longer acoustic tail.
- Adding a Speex `Preprocessor` (noise + residual-echo suppression) on the mic
  path can further clean up the result.
```
