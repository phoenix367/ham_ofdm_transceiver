#!/usr/bin/env python3
"""Put a recording through the LSCodec streaming pipeline and write it back.

    ./venv/bin/python roundtrip.py speech.wav -o restored.wav
    ./venv/bin/python roundtrip.py speech.wav --prompt enrol.wav -o out.wav
    ./venv/bin/python roundtrip.py speech.wav --bitstream out.lsc --measure

Encode and decode both run CHUNKED, exactly as the live path does, so the
result is what the radio would actually deliver -- not a one-shot encode.
That distinction matters: one-shot encoding degrades past ~15 s because
every normalisation in the encoder is GroupNorm over channels AND time,
making its effective receptive field the whole input, and it was trained
on utterances of a few seconds. Chunked is not a compromise here; beyond
half a minute it is measurably better.

Defaults are the configuration settled by measurement:
  chunk 1.00 s, left context 1.28 s, right context 0.32 s
Left context is audio already spoken and costs nothing. Only the right
context is lookahead, so algorithmic latency is chunk + right = 1.32 s.
Dropping the lookahead entirely costs ~9 dB -- it is a cliff, not a knob.

The PROMPT carries the speaker. The token stream is speaker-decoupled by
design, so with the wrong prompt the same bytes decode as a different
person. Without --prompt the first 2 s of the input is used, which is the
optimistic case; a real link caches a prompt per correspondent.
"""
import argparse, os, sys, time
import numpy as np

AP = argparse.ArgumentParser(
    description=__doc__.splitlines()[0],
    epilog="\n".join(__doc__.splitlines()[1:]),
    formatter_class=argparse.RawDescriptionHelpFormatter)
AP.add_argument("input", help="speech to send (any rate, mono or stereo)")
AP.add_argument("-o", "--out", default=None, help="restored wav (default: <input>.restored.wav)")
AP.add_argument("--prompt", default=None,
                help="enrolment audio for the speaker (default: first 2 s of the input)")
AP.add_argument("--prompt-seconds", type=float, default=2.0,
                help="how much of the prompt file to use (2 s buys ~98%% of the benefit)")
AP.add_argument("--chunk", type=float, default=1.0, help="encoder chunk, seconds")
AP.add_argument("--left", type=float, default=1.28, help="left context (free)")
AP.add_argument("--right", type=float, default=0.32, help="lookahead (this is the latency)")
AP.add_argument("--one-shot", action="store_true",
                help="encode the whole file at once instead (worse past ~15 s)")
AP.add_argument("--bitstream", default=None,
                help="write the packed 10-bit tokens, ONE CHUNK PER LINE (hex). "
                     "Each line is independently decodable, which is what the "
                     "radio actually carries: a lost chunk costs its own audio "
                     "and nothing else.")
AP.add_argument("--prompt-out", default=None,
                help="write the speaker prompt as raw binary (float16 LE, "
                     "T x 1024). This is what the decoder needs and what a "
                     "receiver caches per correspondent.")
AP.add_argument("--prompt-int8", action="store_true",
                help="quantise the prompt binary to int8 per channel: half the "
                     "size for a measured 0.03 dB, but never per-tensor -- the "
                     "features are outlier-dominated and a global scale costs 2.4 dB")
AP.add_argument("--measure", action="store_true",
                help="report log-mel distance to the input and air time")
AP.add_argument("--rung", type=int, default=12, help="rung for the air-time estimate")
# Code from the vendored submodule; weights from _lscodec.CKPT.
from _lscodec import SRC, CKPT, WAVLM, add_src
AP.add_argument("--ckpt", default=os.path.join(CKPT, "lscodec_25hz"))
AP.add_argument("--wavlm", default=None)
AP.add_argument("--lscodec-dir", default=SRC)
A = AP.parse_args()

A.wavlm = A.wavlm or WAVLM
A.out = A.out or os.path.splitext(A.input)[0] + ".restored.wav"

sys.path.insert(0, A.lscodec_dir); add_src()
import scipy.signal as _ss
if not hasattr(_ss, "kaiser"):                     # removed in scipy 1.13
    from scipy.signal.windows import kaiser as _k
    _ss.kaiser = _k
import soundfile as sf, torch, torch.nn as nn, yaml
_o = torch.load                                    # torch>=2.6 weights_only
torch.load = lambda *a, **k: _o(*a, **{**k, "weights_only": False})
from lscodec.utils import load_model, load_vocoder
from lscodec.ssl_models.wavlm_extractor import Extractor

SR, HOP = 16000, 640                               # 25 tokens/s

def load_mono16k(path, limit=None):
    x, sr = sf.read(path, dtype="float32")
    if x.ndim > 1:
        x = x.mean(1)
    if sr != SR:
        x = _ss.resample_poly(x, SR, sr).astype(np.float32)
    return x[:int(limit * SR)] if limit else x

def pack10(idx):
    b = "".join(format(int(v), "010b") for v in idx.reshape(-1).tolist())
    b += "0" * ((8 - len(b) % 8) % 8)
    return bytes(int(b[i:i + 8], 2) for i in range(0, len(b), 8))

print("loading codec ...", flush=True)
ec = yaml.load(open(f"{A.ckpt}/encoder_config.yml"), Loader=yaml.Loader)
ec["pretrain_codebook"] = f"{A.ckpt}/codebook.npy"
vc = yaml.load(open(f"{A.ckpt}/vocoder_config.yml"), Loader=yaml.Loader)
vc["vq_codebook"] = f"{A.ckpt}/codebook.npy"
ENC = load_model(ec, f"{A.ckpt}/lscodec_encoder.pt").eval()
VOC = load_vocoder(vc, f"{A.ckpt}/lscodec_vocoder.pt").eval()
WLM = Extractor(checkpoint=A.wavlm, device="cpu")
cb = torch.tensor(np.load(vc["vq_codebook"], allow_pickle=True))
if cb.ndim == 2:
    cb = cb.unsqueeze(0)
NG = cb.shape[0]
CBM = nn.ModuleList([nn.Embedding.from_pretrained(cb[i], freeze=True) for i in range(NG)])
OUT_SR = vc["sampling_rate"]

x = load_mono16k(A.input)
prompt_src = load_mono16k(A.prompt, A.prompt_seconds) if A.prompt \
    else x[:int(A.prompt_seconds * SR)]
print("input  %s: %.2f s" % (os.path.basename(A.input), len(x) / SR))
print("prompt %s: %.2f s"
      % (os.path.basename(A.prompt) if A.prompt else "(first %.1f s of the input)"
         % A.prompt_seconds, len(prompt_src) / SR), flush=True)
P = WLM.extract(prompt_src).numpy()

# ---- encode ---------------------------------------------------------------
t0 = time.time()
if A.one_shot:
    with torch.no_grad():
        _, _, idx = ENC.encode(torch.from_numpy(x).view(1, 1, -1))
    scheme = "one-shot"
else:
    CH, L, R = int(A.chunk * SR), int(A.left * SR), int(A.right * SR)
    parts, k = [], 0
    while k * CH < len(x):
        lo = max(0, k * CH - L)
        hi = min(len(x), (k + 1) * CH + R)
        seg = x[lo:hi]
        if len(seg) < 2000:                        # too short for the encoder
            break
        with torch.no_grad():
            _, _, t = ENC.encode(torch.from_numpy(seg).view(1, 1, -1))
        skip = (k * CH - lo) // HOP
        parts.append(t[skip:skip + min(CH // HOP, t.shape[0] - skip)])
        k += 1
    idx = torch.cat(parts, 0)
    scheme = "chunked %.2fs (+%.2f left, +%.2f right)" % (A.chunk, A.left, A.right)
if A.one_shot:
    parts = [idx]
t_enc = time.time() - t0
raw = pack10(idx)
n = idx.reshape(-1).shape[0]

# ---- decode ---------------------------------------------------------------
t0 = time.time()
ys = []
with torch.no_grad():
    pr = torch.from_numpy(P).float().unsqueeze(0)
    for i in range(0, n, 150):                     # blocks: bounded memory
        j = idx[i:i + 150]
        e = j.repeat_interleave(2, dim=0) if vc.get("repeat_input_tokens") else j
        v = torch.cat([CBM[g](e[:, g]) for g in range(NG)], dim=-1).unsqueeze(0)
        ys.append(VOC.inference(v, pr)[-1].view(-1).numpy())
y = np.concatenate(ys)
t_dec = time.time() - t0
sf.write(A.out, y, OUT_SR, "PCM_16")
if A.bitstream:
    # One line per chunk, packed independently. Concatenating the lines is
    # NOT the same as packing the whole stream: each chunk pads to a byte
    # boundary so it can be lost or decoded on its own.
    with open(A.bitstream, "w") as f:
        f.write("# LSCodec-25Hz tokens, 10 bit/token, %d tokens/s\n" % (SR // HOP))
        f.write("# one line per %s chunk, hex, MSB first, zero-padded to a byte\n"
                % ("one-shot" if A.one_shot else "%.2f s" % A.chunk))
        f.write("# %d chunks, %d tokens, %d bytes packed contiguously\n"
                % (len(parts), n, len(raw)))
        for c in parts:
            f.write(pack10(c).hex() + "\n")
    per_line = sum(len(pack10(c)) for c in parts)

if A.prompt_out:
    if A.prompt_int8:
        lo = P.min(0, keepdims=True); hi = P.max(0, keepdims=True)
        sc = (hi - lo) / 255.0
        q = np.clip(np.round((P - lo) / sc), 0, 255).astype(np.uint8)
        with open(A.prompt_out, "wb") as f:
            f.write(lo.astype("<f4").tobytes())     # per-channel scale first
            f.write(sc.astype("<f4").tobytes())
            f.write(q.tobytes())
    else:
        open(A.prompt_out, "wb").write(P.astype("<f2").tobytes())

secs = len(x) / SR
print("\nencode  %-38s %5.1fx real time" % (scheme, secs / max(t_enc, 1e-9)))
print("stream  %d tokens -> %d bytes, %.1f bit/s" % (n, len(raw), 8 * len(raw) / secs))
print("decode  %.2f s -> %s (%.2f s, %d Hz)   %5.1fx real time"
      % (t_dec, os.path.basename(A.out), len(y) / OUT_SR, OUT_SR, secs / max(t_dec, 1e-9)))
if A.bitstream:
    print("tokens  %s: %d lines, %d B packed per-chunk (%+d B vs contiguous)"
          % (os.path.basename(A.bitstream), len(parts), per_line, per_line - len(raw)))
if A.prompt_out:
    sz = os.path.getsize(A.prompt_out)
    print("prompt  %s: %s, %d x %d, %d B"
          % (os.path.basename(A.prompt_out),
             "int8 per-channel + f32 scales" if A.prompt_int8 else "float16 LE",
             P.shape[0], P.shape[1], sz))
if A.measure:
    import librosa
    def mel(a, sr):
        if sr != SR:
            a = _ss.resample_poly(a, SR, sr).astype(np.float32)
        return librosa.power_to_db(librosa.feature.melspectrogram(
            y=a, sr=SR, n_fft=1024, hop_length=256, n_mels=80), ref=1.0)
    mi, mo = mel(x, SR), mel(y, OUT_SR)
    m = min(mi.shape[1], mo.shape[1])
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    try:
        from ofdm_phy.station import estimate_air_time
        air = estimate_air_time(A.rung, min(len(raw), 255))
        note = ("%.2f s of air at rung %d for the first 255 B" % (air, A.rung))
    except Exception:
        note = "(air-time model unavailable)"
    print("\nlog-mel distance to the input: %.2f dB" % float(np.abs(mi[:, :m] - mo[:, :m]).mean()))
    print("  this includes the codec's own reconstruction loss, so it is NOT")
    print("  comparable with decode-against-decode figures; %s" % note)
