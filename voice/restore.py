#!/usr/bin/env python3
"""Restore a recording from a .lsc token file and a binary speaker prompt.

    ./venv/bin/python restore.py speech.lsc speech.prompt -o restored.wav

This is the receiving half: everything it needs is in those two files plus
the codec weights. The .lsc carries WHAT was said at 250 bit/s; the prompt
carries WHO said it. They are separable by design -- decode the same
tokens against a different prompt and you get a different person speaking
the same words, which is a property of the codec, not a bug here.

The .lsc holds one chunk per line, each packed independently, so a line
that is missing or corrupt costs only its own audio. Damaged lines are
reported and skipped rather than aborting the file: on a broadcast link
chunks really do go missing and the rest is still worth hearing.
"""
import argparse, os, sys, time
import numpy as np

AP = argparse.ArgumentParser(
    description=__doc__.splitlines()[0],
    epilog="\n".join(__doc__.splitlines()[1:]),
    formatter_class=argparse.RawDescriptionHelpFormatter)
AP.add_argument("lsc", help="token file written by roundtrip.py --bitstream")
AP.add_argument("prompt", help="binary prompt written by roundtrip.py --prompt-out")
AP.add_argument("-o", "--out", default=None, help="restored wav (default: <lsc>.restored.wav)")
AP.add_argument("--prompt-format", choices=("auto", "f16", "int8"), default="auto")
AP.add_argument("--gap-fill", choices=("silence", "none"), default="silence",
                help="what to put where a chunk is missing (default: silence, "
                     "so the timeline stays true)")
AP.add_argument("--chunk-seconds", type=float, default=1.0,
                help="chunk length, only used to size a gap")
from _lscodec import SRC, CKPT, add_src
AP.add_argument("--ckpt", default=os.path.join(CKPT, "lscodec_25hz"))
AP.add_argument("--lscodec-dir", default=SRC)
A = AP.parse_args()
A.out = A.out or os.path.splitext(A.lsc)[0] + ".restored.wav"

sys.path.insert(0, A.lscodec_dir); add_src()
import scipy.signal as _ss
if not hasattr(_ss, "kaiser"):
    from scipy.signal.windows import kaiser as _k
    _ss.kaiser = _k
import soundfile as sf, torch, torch.nn as nn, yaml
_o = torch.load
torch.load = lambda *a, **k: _o(*a, **{**k, "weights_only": False})
from lscodec.utils import load_vocoder

DIM, TOK_HZ = 1024, 25


def read_tokens(path):
    """One chunk per line, hex. Returns [(index, [tokens]) ...] and a gap list."""
    chunks, bad = [], []
    for ln, line in enumerate(open(path), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        try:
            raw = bytes.fromhex(line)
        except ValueError:
            bad.append(ln); continue
        n = (len(raw) * 8) // 10
        if n == 0:
            bad.append(ln); continue
        b = "".join(format(v, "08b") for v in raw)
        t = [int(b[i * 10:(i + 1) * 10], 2) for i in range(n)]
        if max(t) > 1023:                       # 10-bit codebook, 1024 entries
            bad.append(ln); continue
        chunks.append(t)
    return chunks, bad


def read_prompt(path, fmt):
    """float16 LE (T x 1024), or int8 with per-channel lo/scale in front."""
    raw = open(path, "rb").read()
    sz = len(raw)
    cands = []
    if fmt in ("auto", "f16") and sz % (DIM * 2) == 0 and sz > 0:
        cands.append(("f16", np.frombuffer(raw, dtype="<f2").astype(np.float32).reshape(-1, DIM)))
    if fmt in ("auto", "int8") and sz > DIM * 8 and (sz - DIM * 8) % DIM == 0:
        lo = np.frombuffer(raw, dtype="<f4", count=DIM).reshape(1, DIM)
        sc = np.frombuffer(raw, dtype="<f4", count=DIM, offset=DIM * 4).reshape(1, DIM)
        q = np.frombuffer(raw, dtype=np.uint8, offset=DIM * 8).reshape(-1, DIM)
        cands.append(("int8", q.astype(np.float32) * sc + lo))
    if not cands:
        sys.exit("prompt %s: %d bytes is not a whole number of %d-dim frames "
                 "in either format" % (path, sz, DIM))
    if fmt != "auto":
        return cands[0]
    # WavLM layer-6 features sit around mean 0.16, std 3.6. A misread format
    # lands nowhere near that, so plausibility picks the right one.
    def score(v):
        return abs(float(v.std()) - 3.6) + abs(float(v.mean()) - 0.16)
    cands.sort(key=lambda c: score(c[1]))
    return cands[0]


chunks, bad = read_tokens(A.lsc)
if not chunks:
    sys.exit("%s: no usable token lines" % A.lsc)
name, P = read_prompt(A.prompt, A.prompt_format)
ntok = sum(len(c) for c in chunks)
print("tokens  %s: %d chunks, %d tokens (%.2f s of speech)"
      % (os.path.basename(A.lsc), len(chunks), ntok, ntok / TOK_HZ))
if bad:
    print("        %d unusable line(s): %s -- skipped, their audio is lost"
          % (len(bad), ", ".join(str(b) for b in bad[:8])))
print("prompt  %s: %s, %d x %d frames (%.2f s of enrolment)"
      % (os.path.basename(A.prompt), name, P.shape[0], P.shape[1], P.shape[0] / 50.0))

print("loading vocoder ...", flush=True)
vc = yaml.load(open(f"{A.ckpt}/vocoder_config.yml"), Loader=yaml.Loader)
vc["vq_codebook"] = f"{A.ckpt}/codebook.npy"
VOC = load_vocoder(vc, f"{A.ckpt}/lscodec_vocoder.pt").eval()
cb = torch.tensor(np.load(vc["vq_codebook"], allow_pickle=True))
if cb.ndim == 2:
    cb = cb.unsqueeze(0)
NG = cb.shape[0]
CBM = nn.ModuleList([nn.Embedding.from_pretrained(cb[i], freeze=True) for i in range(NG)])
OUT_SR = vc["sampling_rate"]

flat = [t for c in chunks for t in c]
idx = torch.tensor(flat, dtype=torch.long).unsqueeze(1)
t0 = time.time()
ys = []
with torch.no_grad():
    pr = torch.from_numpy(P).float().unsqueeze(0)
    for i in range(0, len(flat), 150):              # bounded memory
        j = idx[i:i + 150]
        e = j.repeat_interleave(2, dim=0) if vc.get("repeat_input_tokens") else j
        v = torch.cat([CBM[g](e[:, g]) for g in range(NG)], dim=-1).unsqueeze(0)
        ys.append(VOC.inference(v, pr)[-1].view(-1).numpy())
y = np.concatenate(ys)
if bad and A.gap_fill == "silence":
    y = np.concatenate([y, np.zeros(int(len(bad) * A.chunk_seconds * OUT_SR), np.float32)])
dt = time.time() - t0
sf.write(A.out, y, OUT_SR, "PCM_16")
print("decode  %.2f s -> %s (%.2f s, %d Hz)   %.1fx real time"
      % (dt, os.path.basename(A.out), len(y) / OUT_SR, OUT_SR,
         (len(y) / OUT_SR) / max(dt, 1e-9)))
