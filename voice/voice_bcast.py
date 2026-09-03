#!/usr/bin/env python3
"""End-to-end voice latency over the real radio, by BROADCAST.

Measures the thing that matters: wall time from the first audio sample
entering the encoder on station A to the first decoded sample leaving
the vocoder on station B. Both boards are driven from this one process,
so the two timestamps share a clock.

Broadcast, not ARQ: nothing is acknowledged and nothing is retransmitted,
which is the right contract for speech -- a late repeat is worse than a
gap. Measured on this stand, ARQ costs 2.0-4.8 s per message even in
bulk QoS, and 5-21 s in interactive.

Audio is fed in REAL TIME, as a microphone would deliver it: chunk k is
not encoded until wall time says its lookahead has been spoken.
"""
import argparse, os, sys, threading, time, queue
import numpy as np, soundfile as sf, torch, torch.nn as nn, yaml

AP = argparse.ArgumentParser()
AP.add_argument("--wav", default="/mnt/data/lscodec/audition/ru_5min.wav")
AP.add_argument("--seconds", type=float, default=40.0, help="how much to send")
AP.add_argument("--chunk", type=float, default=1.0, help="encoder chunk")
AP.add_argument("--left", type=float, default=1.28)
AP.add_argument("--right", type=float, default=0.32, help="lookahead = latency")
AP.add_argument("--per-msg", type=float, default=2.0, help="speech per broadcast")
AP.add_argument("--rung", type=int, default=12)
AP.add_argument("--a", default="320047000851333438363436")
AP.add_argument("--b", default="240041000551333438363436")
AP.add_argument("--out", default="/mnt/data/lscodec/audition/bcast_rx.wav")
A = AP.parse_args()

LSHOME = os.environ.get("LSCODEC_HOME", "/mnt/data/lscodec/adapter")
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "host"))
sys.path.insert(0, os.path.join(LSHOME, "LSCodec-Inference"))
import scipy.signal as _ss
if not hasattr(_ss, "kaiser"):
    from scipy.signal.windows import kaiser as _k
    _ss.kaiser = _k
_o = torch.load
torch.load = lambda *a, **k: _o(*a, **{**k, "weights_only": False})
from ofdm_modem import OfdmModem, encode
from lscodec.utils import load_model, load_vocoder
from lscodec.ssl_models.wavlm_extractor import Extractor

PD = os.path.join(LSHOME, "ckpt", "lscodec_25hz")
ec = yaml.load(open(f"{PD}/encoder_config.yml"), Loader=yaml.Loader)
ec["pretrain_codebook"] = f"{PD}/codebook.npy"
vc = yaml.load(open(f"{PD}/vocoder_config.yml"), Loader=yaml.Loader)
vc["vq_codebook"] = f"{PD}/codebook.npy"
print("loading codec ...", flush=True)
enc = load_model(ec, f"{PD}/lscodec_encoder.pt").eval()
voc = load_vocoder(vc, f"{PD}/lscodec_vocoder.pt").eval()
wl = Extractor(checkpoint=os.environ.get("WAVLM_CKPT",
    os.path.expanduser("~/Downloads/WavLM-Large.pt")), device="cpu")
cb = torch.tensor(np.load(vc["vq_codebook"], allow_pickle=True))
if cb.ndim == 2:
    cb = cb.unsqueeze(0)
NG = cb.shape[0]
cbm = nn.ModuleList([nn.Embedding.from_pretrained(cb[i], freeze=True)
                     for i in range(NG)])
H, SR = 640, 16000

x = sf.read(A.wav, dtype="float32")[0][:int(A.seconds * SR)]
prompt = wl.extract(x[:2 * SR]).numpy()          # enrolment, cached at B
print("prompt %s cached at the receiver (enrolment is out of band here)"
      % (tuple(prompt.shape),), flush=True)

def tokens_of(seg):
    with torch.no_grad():
        _, _, i = enc.encode(torch.from_numpy(seg).view(1, 1, -1))
    return i

def pack10(idx):
    b = "".join(format(int(v), "010b") for v in idx.reshape(-1).tolist())
    b += "0" * ((8 - len(b) % 8) % 8)
    return bytes(int(b[i:i + 8], 2) for i in range(0, len(b), 8))

def unpack10(raw, n):
    b = "".join(format(v, "08b") for v in raw)
    return [int(b[i * 10:(i + 1) * 10], 2) for i in range(n)]

ma = OfdmModem(serial=A.a); mb = OfdmModem(serial=A.b)
print("A:", ma.info()["serial"][-6:], " B:", mb.info()["serial"][-6:], flush=True)

rx = queue.Queue(); stop = threading.Event()
def receiver():
    buf = bytearray(); first = None
    while not stop.is_set():
        for name, p in mb.events(timeout=0.05):
            if name != "0x88":
                continue
            if p[0] & 0x80:                       # start of a group
                buf = bytearray()
            elif p[0] & 0x40:                     # end-of-stream stats
                pass
            else:
                if first is None:
                    first = time.time()
                buf += p[1:]
                rx.put((time.time(), bytes(p[1:])))
threading.Thread(target=receiver, daemon=True).start()

CH = int(A.chunk * SR); L = int(A.left * SR); R = int(A.right * SR)
PER = int(round(A.per_msg / A.chunk))            # chunks per broadcast
print("\nchunk %.2f s | left %.2f s | right %.2f s -> codec latency %.2f s"
      % (A.chunk, A.left, A.right, A.chunk + A.right))
print("%.0f s of speech per broadcast, rung %d, ptype 15 (opaque), NON-ARQ\n"
      % (A.per_msg, A.rung), flush=True)

T0 = time.time()                                  # sample 0 enters the encoder
sent = []; pend = []; carry = None; bc_open = False
k = 0
while (k + 1) * CH <= len(x):
    ready_at = T0 + (k + 1) * A.chunk + A.right   # when its lookahead is spoken
    d = ready_at - time.time()
    if d > 0:
        time.sleep(d)
    lo = max(0, k * CH - L); hi = min(len(x), (k + 1) * CH + R)
    t = tokens_of(x[lo:hi]); skip = (k * CH - lo) // H
    pend.append(t[skip:skip + min(CH // H, t.shape[0] - skip)])
    k += 1
    if len(pend) >= PER:
        idx = torch.cat(pend, 0); pend = []
        if carry is not None and carry.shape[0]:
            idx = torch.cat([carry, idx], 0)
        # Whole BYTES only. pack10 zero-pads to a byte boundary, so 50
        # tokens is 500 bits in 63 bytes -- 4 pad bits. The receiver has no
        # framing and unpacks the concatenated bytes as one stream, so those
        # pad bits shift every later group by 4 bits, then 8, then 12: the
        # first group decodes and the rest is noise. 4 tokens = 5 bytes.
        keep = (idx.shape[0] // 4) * 4
        carry = idx[keep:]
        if keep == 0:
            continue
        idx = idx[:keep]
        raw = pack10(idx)
        t_first_sample = T0 + (k - PER) * A.chunk    # first sample IN this group
        mb_seq = len(sent)
        # Continuation chunks, not a new broadcast each time: bc_cmd()
        # treats a non-continuation command as superseding whatever is
        # still unsent, so a fresh command per group overwrites the tail
        # of the one before it (bit 7 = more follows, bit 6 = continue).
        ptype = 15 | 0x80 | (0x40 if bc_open else 0)
        ma.t.write(encode(0x06, bytes([ptype, A.rung]) + raw))
        bc_open = True
        sent.append({"seq": mb_seq, "n": idx.shape[0], "bytes": len(raw),
                     "t_in": t_first_sample, "t_tx": time.time()})
        print("  tx %2d: %3d tokens, %3d B, first sample was %.2f s ago"
              % (mb_seq, idx.shape[0], len(raw), time.time() - t_first_sample),
              flush=True)

# close the stream on the LAST chunk of real data, padded to a whole number
# of bytes so the receiver's byte-derived token count is exact
tail = ([carry] if carry is not None and carry.shape[0] else []) + pend
if tail:
    idx = torch.cat(tail, 0)
    r = idx.shape[0] % 4
    if r:
        idx = torch.cat([idx] + [idx[-1:]] * (4 - r), 0)
    raw = pack10(idx)
    ma.t.write(encode(0x06, bytes([15 | (0x40 if bc_open else 0), A.rung]) + raw))
    sent.append({"seq": len(sent), "n": idx.shape[0], "bytes": len(raw),
                 "t_in": T0 + k * A.chunk, "t_tx": time.time()})
    print("  tx %2d: %3d tokens, %3d B (last)" % (len(sent) - 1, idx.shape[0], len(raw)),
          flush=True)
print("\nall broadcasts queued; draining the receiver ...", flush=True)

deadline = time.time() + 90; got = []
while time.time() < deadline:
    try:
        t, data = rx.get(timeout=1.0)
    except queue.Empty:
        if got and time.time() - got[-1][0] > 25:
            break
        continue
    got.append((t, data))
stop.set()

print("\n=== RESULT ===")
if not got:
    print("  nothing received -- no broadcast decoded")
else:
    t_first_rx = got[0][0]
    nb = sum(len(d) for _, d in got)
    total = b"".join(d for _, d in got)
    ntok = (len(total) * 8) // 10
    t0 = time.time()
    idx = torch.tensor(unpack10(total, ntok), dtype=torch.long).unsqueeze(1)
    with torch.no_grad():
        pr = torch.from_numpy(prompt).float().unsqueeze(0)
        i = idx.repeat_interleave(2, dim=0) if vc.get("repeat_input_tokens") else idx
        v = torch.cat([cbm[g](i[:, g]) for g in range(NG)], dim=-1).unsqueeze(0)
        y = voc.inference(v, pr)[-1].view(-1).numpy()
    sf.write(A.out, y, vc["sampling_rate"], "PCM_16")
    print("  sent      %d broadcasts, %d B of tokens" % (len(sent), sum(s["bytes"] for s in sent)))
    print("  received  %d bytes in %d chunks -> %d tokens, %.1f s of audio"
          % (nb, len(got), ntok, len(y) / vc["sampling_rate"]))
    print("  loss      %.1f %%" % (100.0 * (1 - nb / max(1, sum(s["bytes"] for s in sent)))))
    print()
    # A real receiver decodes the first group as soon as it has it.
    # Time exactly that, rather than scaling a batch decode.
    g0 = got[0][1]
    n0 = (len(g0) * 8) // 10
    td = time.time()
    with torch.no_grad():
        i0 = torch.tensor(unpack10(g0, n0), dtype=torch.long).unsqueeze(1)
        i0 = i0.repeat_interleave(2, dim=0) if vc.get("repeat_input_tokens") else i0
        v0 = torch.cat([cbm[g](i0[:, g]) for g in range(NG)], dim=-1).unsqueeze(0)
        y0 = voc.inference(v0, pr)[-1].view(-1).numpy()
    dec_s = time.time() - td
    t_first_out = t_first_rx + dec_s
    print("  first audio sample INTO encoder (A) : t = 0.00 s")
    print("  first token bytes OUT of radio  (B) : t = %.2f s" % (t_first_rx - T0))
    print("  vocoder on that first group         : %.2f s (%d tokens -> %.1f s audio)"
          % (dec_s, n0, len(y0) / vc["sampling_rate"]))
    print("  first audio sample OUT of vocoder   : t = %.2f s" % (t_first_out - T0))
    print("\n  END-TO-END LATENCY: %.2f s" % (t_first_out - T0))
    print("    codec (chunk %.2f + lookahead %.2f)      %.2f s" % (A.chunk, A.right, A.chunk + A.right))
    print("    fill one broadcast (%.0f s of speech)     %.2f s" % (A.per_msg, sent[0]["t_tx"] - sent[0]["t_in"] - (A.chunk + A.right)))
    print("    radio, 63 B broadcast, non-ARQ          %.2f s" % (t_first_rx - sent[0]["t_tx"]))
    print("    vocoder                                 %.2f s" % dec_s)
    print("  wrote %s" % A.out)
ma.close(); mb.close()
