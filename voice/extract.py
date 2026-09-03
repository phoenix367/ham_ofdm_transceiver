#!/usr/bin/env python3
import os
"""Build a (FACodec timbre -> LSCodec prompt) training set from LibriTTS.

Each utterance yields one pair:

    timbre  256 float16      FACodec's global speaker vector
    prompt  K x 1024 float16 WavLM-Large layer-6 features, k-means'd to K
                             frames -- the SHAPE the adapter must predict

K > 1 matters.  LSCodec's vocoder cross-attends over the prompt, so a
single broadcast vector caps the achievable quality at ~4.1 dB log-mel
distance while K >= 2 reaches ~2.0-2.5 dB, for a few bytes on the wire.
Predicting one vector throws that away before training starts.

Audio is windowed to --seconds (default 2.0), which is the enrolment
length the deployment would actually use: quality saturates there
(SECS 0.855 at 2 s vs 0.869 at 16 s).

    ./extract.py --n 5000 --out data/
    ./extract.py --n 20000 --k 16 --device cuda    # resumes automatically

Shards are written every --shard utterances and skipped if present, so
the job survives interruption.
"""
import argparse, glob, io as _io, json, os, shutil, sys, time
import urllib.request, urllib.error
from concurrent.futures import ThreadPoolExecutor
import collections, threading
from _lscodec import SRC, CKPT, WORK, WAVLM, add_src
ROOT_DIR = WORK  # data shards + trained adapters live here

def _wavlm_default():
    """Prefer a copy on /mnt/data; fall back to ~/Downloads."""
    env = os.environ.get("WAVLM_CKPT")
    cands = ([env] if env else []) + [
        "%s/ckpt/WavLM-Large.pt" % ROOT_DIR,
        os.path.expanduser("~/Downloads/WavLM-Large.pt")]
    for p in cands:
        if os.path.exists(p):
            return p
    return os.path.expanduser("~/Downloads/WavLM-Large.pt")

import numpy as np

AP = argparse.ArgumentParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
AP.add_argument("--speakers", type=int, default=300,
                help="distinct speakers. ONE metadata request per speaker: a\n"
                     "request returns consecutive rows and a speaker spans\n"
                     "~100 of them, so this is what the scan phase costs.")
AP.add_argument("--per-speaker", type=int, default=8,
                help="utterances per speaker; free -- they come from the same request")
AP.add_argument("--k", type=int, default=8, help="prompt frames to keep per utterance")
AP.add_argument("--seconds", type=float, default=2.0, help="enrolment window")
AP.add_argument("--out", default=ROOT_DIR + "/data", help="output directory for shards")
AP.add_argument("--shard", type=int, default=250, help="utterances per shard")
AP.add_argument("--split", default="train.clean.360",
                help="LibriTTS split (train.clean.360 = 1151 speakers)")
AP.add_argument("--wavlm", default=None)
AP.add_argument("--facodec-dir", default=CKPT)
AP.add_argument("--lscodec-dir", default=SRC)
AP.add_argument("--facodec-repo", default=ROOT_DIR + "/naturalspeech3_facodec")
AP.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"))
AP.add_argument("--full-prompt", action="store_true",
                help="store the WHOLE T'x1024 prompt instead of K centroids.\nK then becomes a TRAIN-time choice, not an extraction one -- and K was only\never justified by wire cost, which does not apply once the receiver\ngenerates the prompt locally from the timbre vector. ~203 kB/utterance.")
AP.add_argument("--with-tokens", action="store_true",
                help="also store LSCodec tokens, needed for a decoder-in-the-loop loss")
AP.add_argument("--meta-rate", type=int, default=40,
                help="metadata requests per --meta-window. MEASURED limit is\n~46 per ~49 s, unauthenticated, and the server sends NO\nRetry-After: exceed it and every request 429s.")
AP.add_argument("--meta-window", type=float, default=60.0)
AP.add_argument("--meta-workers", type=int, default=3,
                help="parallel metadata requests; the datasets server 500s "
                     "above ~6 and each lost request costs a whole speaker")
AP.add_argument("--fetch-workers", type=int, default=12,
                help="parallel audio downloads; extraction is network-bound, "
                     "not compute-bound -- the GPU barely helps without this")
A = AP.parse_args()
A.wavlm = A.wavlm or _wavlm_default()
A.n = A.speakers * A.per_speaker

# ---- disk guard: this is the failure that wastes an hour ------------------
per_utt = 256 * 2 + (99 if A.full_prompt else A.k) * 1024 * 2
need = per_utt * A.n
free = shutil.disk_usage(os.path.dirname(os.path.abspath(A.out)) or ".").free
print("plan: %d utterances x (256 timbre + %sx1024 prompt) fp16 = %.1f MB"
      % (A.n, "99 (FULL)" if A.full_prompt else A.k, need / 1e6))
if free < need * 1.3:
    sys.exit("refusing to start: %.1f GB free, need ~%.1f GB with headroom.\n"
             "Lower --n or --k, or free space first." % (free / 1e9, need * 1.3 / 1e9))

import torch, soundfile as sf, scipy.signal as ss
from scipy.cluster.vq import kmeans2
dev = ("cuda" if torch.cuda.is_available() else "cpu") if A.device == "auto" else A.device
if dev == "cuda" and not torch.cuda.is_available():
    sys.exit("--device cuda but torch reports no CUDA. This venv may have the\n"
             "CPU-only wheel: reinstall torch from the cu121 index.")
print("device: %s | torch %s" % (dev, torch.__version__), flush=True)

sys.path.insert(0, A.facodec_repo)
sys.path.insert(0, A.lscodec_dir); add_src()
import scipy.signal as _ss                       # lscodec imports a removed scipy name
if not hasattr(_ss, "kaiser"):
    from scipy.signal.windows import kaiser as _k
    _ss.kaiser = _k
_orig = torch.load                               # torch>=2.6 defaults weights_only=True
torch.load = lambda *a, **k: _orig(*a, **{**k, "weights_only": False})

from ns3_codec import FACodecEncoder, FACodecDecoder
from lscodec.ssl_models.wavlm_extractor import Extractor as WavLM

print("loading models ...", flush=True)
fe = FACodecEncoder(ngf=32, up_ratios=[2, 4, 5, 5], out_channels=256)
fd = FACodecDecoder(in_channels=256, upsample_initial_channel=1024, ngf=32,
                    up_ratios=[5, 5, 4, 2], vq_num_q_c=2, vq_num_q_p=1, vq_num_q_r=3,
                    vq_dim=256, codebook_dim=8, codebook_size_prosody=10,
                    codebook_size_content=10, codebook_size_residual=10,
                    use_gr_x_timbre=True, use_gr_residual_f0=True,
                    use_gr_residual_phone=True)
fe.load_state_dict(torch.load(f"{A.facodec_dir}/ns3_facodec_encoder.bin", map_location="cpu"))
fd.load_state_dict(torch.load(f"{A.facodec_dir}/ns3_facodec_decoder.bin", map_location="cpu"))
fe = fe.to(dev).eval(); fd = fd.to(dev).eval()
wl = WavLM(checkpoint=A.wavlm, device=dev)
enc = None
if A.with_tokens:
    import yaml
    from lscodec.utils import load_model
    _pd = os.path.join(A.facodec_dir, "lscodec_25hz")
    _ec = yaml.load(open(f"{_pd}/encoder_config.yml"), Loader=yaml.Loader)
    _ec["pretrain_codebook"] = f"{_pd}/codebook.npy"
    enc = load_model(_ec, f"{_pd}/lscodec_encoder.pt").to(dev).eval()

def tokens_of(x16):
    with torch.no_grad():
        _, _, idx = enc.encode(torch.from_numpy(x16).view(1, 1, -1).to(dev))
    return idx.reshape(-1).cpu().numpy().astype(np.int16)

def timbre(x16):
    with torch.no_grad():
        w = torch.from_numpy(x16).float().unsqueeze(0).unsqueeze(0).to(dev)
        *_, spk = fd(fe(w), eval_vq=False, vq=True)
    return spk.squeeze(0).cpu().numpy()

def prompt(x16, k):
    f = wl.extract(x16).cpu().numpy()
    if A.full_prompt:
        return f
    if len(f) <= k:
        f = np.pad(f, ((0, k - len(f)), (0, 0)), mode="edge")
        return f
    c, _ = kmeans2(f, k, minit="points", seed=0, iter=40)
    return c

# ---- stream utterance metadata from the HF datasets server ---------------
def rows(split, n_spk, per_spk):
    """One request per speaker at spread offsets.

    Rows are speaker-ordered and a speaker spans ~100 of them, so a single
    request lands inside one speaker: the number of requests that SUCCEED is
    the number of speakers you get. Asking for 1250 speakers means 1250 round
    trips through a server that 500s under load -- which is how the first run
    quietly returned 71 speakers instead of 1151.
    """
    base = ("https://datasets-server.huggingface.co/rows?dataset="
            "mythicinfinity/libritts&config=clean&split=%s" % split)
    total = json.load(urllib.request.urlopen(
        "https://datasets-server.huggingface.co/size?dataset=mythicinfinity/libritts"
        "&config=clean", timeout=60))["size"]["splits"]
    n_rows = next(s["num_rows"] for s in total if s["split"] == split)
    step = max(1, n_rows // n_spk)
    offs = list(range(0, n_rows, step))[:n_spk]
    fails, limited, t0 = [0], [0], time.time()
    _lock, _hist = threading.Lock(), collections.deque()
    def _throttle():
        """Token bucket. Concurrency does not buy throughput against a rate
        limit -- it only spends the budget sooner and then fails."""
        while True:
            with _lock:
                now = time.time()
                while _hist and now - _hist[0] > A.meta_window:
                    _hist.popleft()
                if len(_hist) < A.meta_rate:
                    _hist.append(now)
                    return
                wait = A.meta_window - (now - _hist[0]) + 0.25
            time.sleep(wait)
    def grab(off):
        for attempt in range(6):
            _throttle()
            try:
                return json.load(urllib.request.urlopen(
                    "%s&offset=%d&length=100" % (base, off), timeout=90))["rows"]
            except urllib.error.HTTPError as e:
                if e.code == 429:
                    limited[0] += 1
                    time.sleep(A.meta_window)      # no Retry-After is sent
                else:
                    time.sleep(2.0 * (2 ** attempt))
            except Exception:
                time.sleep(2.0 * (2 ** attempt))
        fails[0] += 1
        return []
    print("  scan: %d requests (one per speaker) over %d rows, %d workers"
          % (len(offs), n_rows, A.meta_workers), flush=True)
    chunks, done_n = [], 0
    with ThreadPoolExecutor(max_workers=A.meta_workers) as ex:
        for r in ex.map(grab, offs):
            chunks.append(r); done_n += 1
            if done_n % 25 == 0 or done_n == len(offs):
                el = time.time() - t0
                r = done_n / max(el, 1e-6)
                print("    %d/%d requests, %d failed, %d rate-limited, "
                      "%.2f req/s, eta %.0f s"
                      % (done_n, len(offs), fails[0], limited[0], r,
                         (len(offs) - done_n) / max(r, 1e-6)), flush=True)
    seen, out = {}, []
    for r in chunks:
        for it in r:
            d = it["row"]; sp = str(d["speaker_id"])
            if seen.get(sp, 0) >= per_spk:
                continue
            seen[sp] = seen.get(sp, 0) + 1
            out.append(d)
    if fails[0]:
        print("  WARNING: %d of %d requests failed after retries -- that many "
              "speakers lost" % (fails[0], len(offs)), flush=True)
    print("  scan took %.0f s" % (time.time() - t0), flush=True)
    return out, len(seen)

os.makedirs(A.out, exist_ok=True)
print("querying LibriTTS %s for %d utterances ..." % (A.split, A.n), flush=True)
items, n_spk = rows(A.split, A.speakers, A.per_speaker)
print("  got %d utterances from %d speakers" % (len(items), n_spk), flush=True)

# Resume by utterance id, not by loop position: utterances get skipped
# (too short, fetch failed) so the loop counter and the number actually
# written drift apart. Deriving the shard index from the counter made the
# second flush overwrite the first.
have, shard_idx = set(), 0
for f in sorted(glob.glob(os.path.join(A.out, "shard_*.npz"))):
    try:
        have |= set(np.load(f)["utt"].tolist())
        shard_idx = max(shard_idx, int(os.path.basename(f)[6:10]) + 1)
    except Exception:
        pass
if have:
    print("resuming: %d utterances already extracted in %d shards"
          % (len(have), shard_idx))

def fetch(d):
    """Download and decode one utterance. Runs in a worker thread; the
    models stay on the main thread, so no CUDA context is shared."""
    try:
        a = d["audio"]; a = a[0] if isinstance(a, list) else a
        raw = urllib.request.urlopen(a["src"], timeout=90).read()
        x, sr = sf.read(_io.BytesIO(raw), dtype="float32")
        if x.ndim > 1:
            x = x.mean(1)
        if sr != 16000:
            x = ss.resample_poly(x, 16000, sr).astype(np.float32)
        n = int(A.seconds * 16000)
        return (str(d["id"]), x[:n].copy()) if len(x) >= n else (str(d["id"]), None)
    except Exception:
        return (str(d["id"]), None)

todo = [d for d in items if str(d["id"]) not in have]
skipped = len(items) - len(todo)
T, P, S, U, K_ = [], [], [], [], []
all_spk = set()
t0, done = time.time(), 0
CH = max(A.fetch_workers * 4, A.shard)
pool = ThreadPoolExecutor(max_workers=A.fetch_workers)
fetched = {}
for i, d in enumerate(todo):
    if i % CH == 0:                       # prefetch the next chunk in parallel
        tf = time.time()
        fetched = dict(pool.map(fetch, todo[i:i + CH]))
        got = sum(1 for v in fetched.values() if v is not None)
        el = max(time.time() - t0, 1e-6)
        rate = done / el
        eta = ("eta %.0f min" % ((len(todo) - i) / rate / 60)) if done > 20 else "eta --"
        print("    fetch %d-%d: %d/%d ok in %.0f s | %.2f utt/s, %d done, %s"
              % (i, min(i + CH, len(todo)), got, len(fetched), time.time() - tf,
                 rate, done, eta), flush=True)
    x = fetched.get(str(d["id"]))
    if x is None:
        continue
    try:
        T.append(timbre(x).astype(np.float16))
        P.append(prompt(x, A.k).astype(np.float16))
        if enc is not None:
            K_.append(tokens_of(x))
        S.append(str(d["speaker_id"])); U.append(str(d["id"]))
        all_spk.add(str(d["speaker_id"]))
        done += 1
    except Exception as e:
        print("  skip %s: %s" % (d.get("id"), e), flush=True)
    if len(T) >= A.shard or (i == len(todo) - 1 and T):
        if T:
            path = os.path.join(A.out, "shard_%04d.npz" % shard_idx)
            shard_idx += 1
            extra = {}
            if K_:
                n = min(len(k) for k in K_)      # 2 s windows -> equal length
                extra["tokens"] = np.stack([k[:n] for k in K_])
            np.savez_compressed(path, timbre=np.stack(T), prompt=np.stack(P),
                                spk=np.array(S), utt=np.array(U), **extra)
            rate = done / max(1e-6, time.time() - t0)
            print("  %s: %d utts, %d speakers so far, %.2f utt/s, eta %.0f min"
                  % (os.path.basename(path), done, len(set(S)),
                     rate, (len(todo) - i) / max(rate, 1e-6) / 60))
            T, P, S, U, K_ = [], [], [], [], []
el = time.time() - t0
sz = sum(os.path.getsize(os.path.join(A.out, f)) for f in os.listdir(A.out)
         if f.endswith(".npz"))
print("\ndone in %.0f min: %d extracted, %d skipped, %d speakers, %.1f MB on disk"
      % (el / 60, done, skipped, len(all_spk), sz / 1e6), flush=True)
print("wrote %s" % os.path.abspath(A.out), flush=True)
