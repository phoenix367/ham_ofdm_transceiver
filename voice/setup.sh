#!/usr/bin/env bash
# One-time setup: venv, repos, checkpoints. Safe to re-run.
set -euo pipefail
# Build the codec HOME off-tree, not in the repo: this clones ~6 GB of
# third-party trees plus a venv and checkpoints, none of which belong in
# version control. LSCODEC_HOME is the same knob the scripts and the web
# app read; the default matches this stand.
TARGET="${LSCODEC_HOME:-/mnt/data/lscodec/adapter}"
mkdir -p "$TARGET"
cd "$TARGET"
echo "building LSCodec home in $TARGET"

# --- venv -----------------------------------------------------------------
# CUDA (cu121) by default -- the GTX 1050 is Pascal. Set TORCH_INDEX to
# .../whl/cpu to force a CPU build.

if [ ! -d venv ]; then
    python3.10 -m venv venv
    ./venv/bin/pip -q install --upgrade pip
fi
# Idempotent, and OUTSIDE the venv-exists guard: an interrupted first run
# leaves a venv with torch and nothing else, and re-running must finish it.
./venv/bin/python -c "import torch" 2>/dev/null || \
    ./venv/bin/pip install --index-url "${TORCH_INDEX:-https://download.pytorch.org/whl/cu121}" torch torchaudio
    # cu121, not cu126/cu128/cu13: the GTX 1050 is Pascal (sm_61) and CUDA 13
    # dropped Pascal entirely. A cu13 wheel installs happily and then fails at
    # the first kernel launch with "no kernel image is available".
./venv/bin/python -c "import numpy, scipy, soundfile, yaml, einops, kaldiio, h5py, pyworld, librosa" 2>/dev/null || \
    ./venv/bin/pip install numpy scipy soundfile pyyaml einops kaldiio h5py pyworld librosa tqdm

# --- code: LSCodec (for its WavLM extractor) and FACodec ------------------
[ -d LSCodec-Inference ] || git clone --depth 1 https://github.com/X-LANCE/LSCodec-Inference
[ -d naturalspeech3_facodec ] || git clone --depth 1 https://github.com/lifeiteng/naturalspeech3_facodec

# --- checkpoints ----------------------------------------------------------
mkdir -p ckpt
for f in ns3_facodec_encoder.bin ns3_facodec_decoder.bin; do
    [ -s "ckpt/$f" ] || curl -sSL -o "ckpt/$f" \
        "https://huggingface.co/amphion/naturalspeech3_facodec/resolve/main/$f"
done
# LSCodec vocoder + codebook, only needed by eval_adapter.py
mkdir -p ckpt/lscodec_25hz
for f in encoder_config.yml vocoder_config.yml codebook.npy lscodec_encoder.pt lscodec_vocoder.pt; do
    [ -s "ckpt/lscodec_25hz/$f" ] || curl -sSL -o "ckpt/lscodec_25hz/$f" \
        "https://huggingface.co/cantabile-kwok/lscodec_25hz/resolve/main/$f"
done

# WavLM-Large.pt is NOT scripted: the official Azure link returns
# AuthenticationFailed and the Google Drive mirror throttles hard.
if [ ! -s "ckpt/WavLM-Large.pt" ] && [ ! -s "$HOME/Downloads/WavLM-Large.pt" ]; then
    echo
    echo "MISSING: ~/Downloads/WavLM-Large.pt (1.2 GB, fairseq format, 'cfg'+'model' keys)"
    echo "  from https://github.com/microsoft/unilm/tree/master/wavlm (Google Drive link)"
    echo "  pass another location with --wavlm, or drop it in ckpt/"
fi
echo
echo "setup complete in $TARGET"
echo "Next:  LSCODEC_HOME=$TARGET $TARGET/venv/bin/python <repo>/voice/extract.py --n 5000"
