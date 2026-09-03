#!/usr/bin/env python3
import os
"""Train the timbre -> prompt adapter on the extracted shards.

Predicts K prompt FRAMES, not one vector: with this vocoder a single
broadcast vector caps quality at ~4.1 dB while K>=2 reaches ~2.0-2.5 dB.

Split is by SPEAKER, never by utterance -- an utterance split leaks the
speaker into training and the validation number becomes meaningless.

    ./train_adapter.py --data data/ --epochs 60
"""
import argparse, glob, os
ROOT_DIR = os.environ.get("LSCODEC_HOME", "/mnt/data/lscodec/adapter")
import numpy as np, torch, torch.nn as nn

AP = argparse.ArgumentParser()
AP.add_argument("--data", default=ROOT_DIR + "/data")
AP.add_argument("--epochs", type=int, default=60)
AP.add_argument("--hidden", type=int, default=256)
AP.add_argument("--dropout", type=float, default=0.5)
AP.add_argument("--lr", type=float, default=1e-3)
AP.add_argument("--batch", type=int, default=64)
AP.add_argument("--val-speakers", type=float, default=0.15)
AP.add_argument("--out", default=ROOT_DIR + "/adapter.pt")
AP.add_argument("--device", default="auto")
A = AP.parse_args()
dev = ("cuda" if torch.cuda.is_available() else "cpu") if A.device == "auto" else A.device

sh = sorted(glob.glob(os.path.join(A.data, "shard_*.npz")))
if not sh:
    raise SystemExit("no shards in %s -- run extract.py first" % A.data)
T = np.concatenate([np.load(f)["timbre"] for f in sh]).astype(np.float32)
P = np.concatenate([np.load(f)["prompt"] for f in sh]).astype(np.float32)
S = np.concatenate([np.load(f)["spk"] for f in sh])
K, D = P.shape[1], P.shape[2]
print("%d utterances, %d speakers, prompt %d x %d" % (len(T), len(set(S.tolist())), K, D), flush=True)

spk = np.array(sorted(set(S.tolist())))
rng = np.random.RandomState(0); rng.shuffle(spk)
val_spk = set(spk[:max(1, int(len(spk) * A.val_speakers))].tolist())
va = np.array([s in val_spk for s in S]); tr = ~va
print("split by speaker: %d train / %d val utts (%d / %d speakers)"
      % (tr.sum(), va.sum(), len(spk) - len(val_spk), len(val_spk)))

mu, sd = T[tr].mean(0), T[tr].std(0) + 1e-6          # normalise inputs
pm = P[tr].reshape(-1, D).mean(0)                    # predict the RESIDUAL:
                                                     # the common component is
                                                     # huge and learning it wastes
                                                     # the whole capacity
def prep(X, Y):
    return (torch.tensor((X - mu) / sd), torch.tensor(Y - pm))
Xtr, Ytr = prep(T[tr], P[tr]); Xva, Yva = prep(T[va], P[va])

# Swept: capacity is NOT the bottleneck. 0.28M params scores +0.314 and
# 9.7M scores +0.318, while the big model overfits by epoch 2 (val R2 goes
# NEGATIVE). Small + heavy dropout + early stopping is what generalises.
net = nn.Sequential(nn.Linear(256, A.hidden), nn.GELU(), nn.Dropout(A.dropout),
                    nn.Linear(A.hidden, K * D)).to(dev)
opt = torch.optim.AdamW(net.parameters(), lr=A.lr, weight_decay=1e-2)
sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, A.epochs)

def cosine(a, b):
    a = a.reshape(len(a), -1); b = b.reshape(len(b), -1)
    return torch.nn.functional.cosine_similarity(a, b).mean().item()

best = -1
for ep in range(A.epochs):
    net.train(); perm = torch.randperm(len(Xtr))
    for i in range(0, len(perm), A.batch):
        j = perm[i:i + A.batch]
        x, y = Xtr[j].to(dev), Ytr[j].to(dev)
        loss = nn.functional.mse_loss(net(x).view(-1, K, D), y)
        opt.zero_grad(); loss.backward(); opt.step()
    sched.step()
    net.eval()
    with torch.no_grad():
        pv = net(Xva.to(dev)).view(-1, K, D).cpu()
        c = cosine(pv, Yva)
        r2 = 1 - (pv - Yva).pow(2).sum().item() / Yva.pow(2).sum().item()
    if c > best:
        best = c
        torch.save({"state": net.state_dict(), "mu": mu, "sd": sd, "pm": pm,
                    "K": K, "D": D, "hidden": A.hidden,
                    "dropout": A.dropout}, A.out)
    if ep % 5 == 0 or ep == A.epochs - 1:
        print("  epoch %3d  train mse %.4f  val cosine %+.3f  val R2 %+.3f%s"
              % (ep, loss.item(), c, r2, "  *" if c == best else ""))
print("\nbest val cosine %+.3f -> %s" % (best, A.out), flush=True)
print("cosine is a proxy. The number that matters is the DECODE:", flush=True)
print("  ./eval_adapter.py --adapter %s --wav <held-out speaker>.wav" % A.out, flush=True)
