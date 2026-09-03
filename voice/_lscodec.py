"""Locate the LSCodec code, its checkpoints, and the training work area.

The CODE is vendored as a git submodule
(`voice/third_party/LSCodec-Inference`, the phoenix367 fork pinned by
commit) -- no configuration, and it always matches the checkpoints it was
tested against. This is what replaced LSCODEC_HOME for imports.

The WEIGHTS cannot live in git (the lscodec_25hz + FACodec checkpoints are
~580 MB, WavLM another 1.2 GB). `LSCODEC_CKPT` points at the checkpoint
directory and defaults to this stand, so nothing needs setting here; a
clone points it wherever `setup.sh` downloaded them. The training WORK
area (dataset shards, trained adapters) is simply the checkpoint dir's
parent, so it too needs no separate knob. `WAVLM_CKPT` locates WavLM.
"""
import os
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(_REPO, "voice", "third_party", "LSCodec-Inference")
CKPT = os.environ.get("LSCODEC_CKPT", "/mnt/data/lscodec/adapter/ckpt")
WORK = os.path.dirname(CKPT)
WAVLM = os.environ.get("WAVLM_CKPT",
                       os.path.expanduser("~/Downloads/WavLM-Large.pt"))


def add_src():
    """Put the vendored LSCodec tree on sys.path (idempotent)."""
    if not os.path.isdir(os.path.join(SRC, "lscodec")):
        raise SystemExit(
            "LSCodec submodule missing at %s\n"
            "  run: git submodule update --init --depth 1 %s"
            % (SRC, os.path.relpath(SRC, _REPO)))
    if SRC not in sys.path:
        sys.path.insert(0, SRC)
