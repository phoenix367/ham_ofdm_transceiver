#!/usr/bin/env python3
"""Train the adapter through the vocoder's frontend, not on features.

Feature MSE weights all 8192 output dimensions equally; almost none of them
matter perceptually. This optimises what the vocoder actually produces.

The prompt only reaches the output through the frontend -- ConvPromptPrenet
plus cross-attention in both Conformer stages. HiFiGAN takes the frontend's
hidden state and never sees the prompt, so backprop can stop at the
frontend's mel projection and skip the waveform generator entirely. That
turns an intractable loop into one that fits a 4 GB card.

Target is the frontend output under the TRUE K-frame prompt -- i.e. the
K-frame ceiling itself (~2.0-2.5 dB), which is the best this shape allows.

    ./train_dl.py --data data_dl/ --epochs 40
"""
import argparse, glob, os, sys
import numpy as np, torch, torch.nn as nn, yaml

AP = argparse.ArgumentParser()
LSHOME = os.environ.get("LSCODEC_HOME", "/mnt/data/lscodec/adapter")
AP.add_argument("--data", default=os.path.join(LSHOME, "data_dl"))
AP.add_argument("--epochs", type=int, default=40)
AP.add_argument("--hidden", type=int, default=256)
AP.add_argument("--dropout", type=float, default=0.5)
AP.add_argument("--lr", type=float, default=1e-3)
AP.add_argument("--batch", type=int, default=8)
AP.add_argument("--val-speakers", type=float, default=0.15)
AP.add_argument("--ckpt", default=os.path.join(LSHOME, "ckpt"))
AP.add_argument("--lscodec-dir", default=os.path.join(LSHOME, "LSCodec-Inference"))
AP.add_argument("--target", default="enc", choices=("enc", "mel", "both"),
                help="what the L1 is taken on. 'mel' is the frontend's\nAUXILIARY 80-dim projection -- a training aid, NOT what HiFiGAN consumes.\n'enc' is enc_out, the hidden state that fully determines the waveform.")
AP.add_argument("--out", default=os.path.join(LSHOME, "adapter_dl.pt"))
A = AP.parse_args()

sys.path.insert(0, A.lscodec_dir)
import scipy.signal as _ss
if not hasattr(_ss, "kaiser"):
    from scipy.signal.windows import kaiser as _k
    _ss.kaiser = _k
_o = torch.load
torch.load = lambda *a, **k: _o(*a, **{**k, "weights_only": False})
from lscodec.utils import load_vocoder

dev = "cuda" if torch.cuda.is_available() else "cpu"
PD = os.path.join(A.ckpt, "lscodec_25hz")
vc = yaml.load(open(f"{PD}/vocoder_config.yml"), Loader=yaml.Loader)
vc["vq_codebook"] = f"{PD}/codebook.npy"
voc = load_vocoder(vc, f"{PD}/lscodec_vocoder.pt").to(dev).eval()
front = voc.frontend                       # prompt-dependent half only
for p in front.parameters():
    p.requires_grad_(False)
cb = torch.tensor(np.load(vc["vq_codebook"], allow_pickle=True))
if cb.ndim == 2:
    cb = cb.unsqueeze(0)
NG = cb.shape[0]
cbm = nn.ModuleList([nn.Embedding.from_pretrained(cb[i], freeze=True)
                     for i in range(NG)]).to(dev)

fs = sorted(glob.glob(os.path.join(A.data, "shard_*.npz")))
if not fs:
    raise SystemExit("no shards in %s" % A.data)
d0 = np.load(fs[0])
if "tokens" not in d0:
    raise SystemExit("shards have no tokens -- re-extract with --with-tokens")
T = np.concatenate([np.load(f)["timbre"] for f in fs]).astype(np.float32)
P = np.concatenate([np.load(f)["prompt"] for f in fs]).astype(np.float32)
S = np.concatenate([np.load(f)["spk"] for f in fs])
ntok = min(np.load(f)["tokens"].shape[1] for f in fs)
TK = np.concatenate([np.load(f)["tokens"][:, :ntok] for f in fs]).astype(np.int64)
K, D = P.shape[1], P.shape[2]
print("%d utts, %d speakers, prompt %dx%d, %d tokens each"
      % (len(T), len(set(S.tolist())), K, D, ntok), flush=True)

spk = np.array(sorted(set(S.tolist())))
rng = np.random.RandomState(0); rng.shuffle(spk)
val = set(spk[:max(1, int(len(spk) * A.val_speakers))].tolist())
va = np.array([s in val for s in S]); tr = ~va
mu, sd = T[tr].mean(0), T[tr].std(0) + 1e-6
pm = P[tr].reshape(-1, D).mean(0)
print("split by speaker: %d train / %d val" % (tr.sum(), va.sum()), flush=True)

def vqvec(tok):
    i = tok.repeat_interleave(2, dim=1) if vc.get("repeat_input_tokens") else tok
    return torch.cat([cbm[g](i) for g in range(NG)], dim=-1)

def front_out(vq, prompt):
    """enc_out is what the HiFiGAN backend actually receives; mel is an
    auxiliary projection kept from training. Matching enc_out matches the
    waveform; matching mel only correlates with it."""
    enc_out, mel = front(vq, prompt)
    if A.target == "enc":
        return (enc_out,)
    if A.target == "mel":
        return (mel,)
    return (enc_out, mel)

net = nn.Sequential(nn.Linear(256, A.hidden), nn.GELU(), nn.Dropout(A.dropout),
                    nn.Linear(A.hidden, K * D)).to(dev)
opt = torch.optim.AdamW(net.parameters(), lr=A.lr, weight_decay=1e-2)
sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, A.epochs)

Tt = torch.tensor((T - mu) / sd); Pt = torch.tensor(P); Kt = torch.tensor(TK)
pmT = torch.tensor(pm).to(dev)

def batch_loss(idx, train):
    x = Tt[idx].to(dev)
    vq = vqvec(Kt[idx].to(dev))
    with torch.no_grad():
        tgt = front_out(vq, Pt[idx].to(dev))         # the K-frame ceiling
    pred = net(x).view(-1, K, D) + pmT
    out = front_out(vq, pred)
    # normalise each term by the target's own scale so 'both' is balanced
    return sum(nn.functional.l1_loss(o, t) / (t.abs().mean() + 1e-6)
               for o, t in zip(out, tgt)) / len(tgt)

tr_i = np.where(tr)[0]; va_i = np.where(va)[0]
best = 1e9
for ep in range(A.epochs):
    net.train(); perm = np.random.permutation(tr_i)
    tot = n = 0
    for i in range(0, len(perm), A.batch):
        j = perm[i:i + A.batch]
        loss = batch_loss(j, True)
        opt.zero_grad(); loss.backward(); opt.step()
        tot += loss.item() * len(j); n += len(j)
    sched.step()
    net.eval(); vt = vn = 0
    with torch.no_grad():
        for i in range(0, len(va_i), A.batch):
            j = va_i[i:i + A.batch]
            vt += batch_loss(j, False).item() * len(j); vn += len(j)
    v = vt / max(vn, 1)
    if v < best:
        best = v
        torch.save({"state": net.state_dict(), "mu": mu, "sd": sd, "pm": pm,
                    "K": K, "D": D, "hidden": A.hidden, "dropout": A.dropout}, A.out)
    if ep % 2 == 0 or ep == A.epochs - 1:
        print("  epoch %3d  train L1(%s) %.4f  val %.4f%s"
              % (ep, A.target, tot / max(n, 1), v, "  *" if v == best else ""), flush=True)
print("\nbest val L1(%s) %.4f -> %s" % (A.target, best, A.out), flush=True)
