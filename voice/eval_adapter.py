#!/usr/bin/env python3
import os
"""Decode a real utterance with the adapter's predicted prompt.

The training loss is a proxy; this is the number that decides whether the
adapter is good enough. Everything is scored as mean absolute log-mel
distance against the decode under the TRUE prompt, on the same tokens.

Reference points measured on this vocoder:
    0.00 dB  the true prompt (by definition)
    ~2.0-2.5 the ceiling for a K-frame summary, K >= 2
    ~4.1     the ceiling for ONE broadcast vector
    7.5      an untrained linear adapter from FACodec timbre
    8.9      a different real speaker

    ./eval_adapter.py --adapter adapter.pt --wav sample.wav
"""
import argparse, os, sys
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

import numpy as np, torch, torch.nn as nn, soundfile as sf, yaml
import scipy.signal as ss

AP = argparse.ArgumentParser()
AP.add_argument("--adapter", default=ROOT_DIR + "/adapter.pt")
AP.add_argument("--wav", required=True, help="held-out speaker's audio, 16 kHz")
AP.add_argument("--seconds", type=float, default=2.0)
AP.add_argument("--ckpt", default=CKPT)
AP.add_argument("--lscodec-dir", default=SRC)
AP.add_argument("--facodec-repo", default=ROOT_DIR + "/naturalspeech3_facodec")
AP.add_argument("--wavlm", default=None)
AP.add_argument("--outdir", default=ROOT_DIR + "/out")
A = AP.parse_args()
A.wavlm = A.wavlm or _wavlm_default()

sys.path.insert(0, A.facodec_repo); sys.path.insert(0, A.lscodec_dir); add_src()
import scipy.signal as _ss
if not hasattr(_ss, "kaiser"):
    from scipy.signal.windows import kaiser as _k
    _ss.kaiser = _k
_orig = torch.load
torch.load = lambda *a, **k: _orig(*a, **{**k, "weights_only": False})
import librosa
from ns3_codec import FACodecEncoder, FACodecDecoder
from lscodec.utils import load_model, load_vocoder
from lscodec.ssl_models.wavlm_extractor import Extractor as WavLM

PD = os.path.join(A.ckpt, "lscodec_25hz")
ec = yaml.load(open(f"{PD}/encoder_config.yml"), Loader=yaml.Loader)
ec["pretrain_codebook"] = f"{PD}/codebook.npy"
vc = yaml.load(open(f"{PD}/vocoder_config.yml"), Loader=yaml.Loader)
vc["vq_codebook"] = f"{PD}/codebook.npy"
enc = load_model(ec, f"{PD}/lscodec_encoder.pt").eval()
voc = load_vocoder(vc, f"{PD}/lscodec_vocoder.pt").eval()
wl = WavLM(checkpoint=A.wavlm, device="cpu")
cbk = torch.tensor(np.load(vc["vq_codebook"], allow_pickle=True))
if cbk.ndim == 2:
    cbk = cbk.unsqueeze(0)
NG = cbk.shape[0]
cbm = nn.ModuleList([nn.Embedding.from_pretrained(cbk[i], freeze=True) for i in range(NG)])

fe = FACodecEncoder(ngf=32, up_ratios=[2, 4, 5, 5], out_channels=256)
fd = FACodecDecoder(in_channels=256, upsample_initial_channel=1024, ngf=32,
                    up_ratios=[5, 5, 4, 2], vq_num_q_c=2, vq_num_q_p=1, vq_num_q_r=3,
                    vq_dim=256, codebook_dim=8, codebook_size_prosody=10,
                    codebook_size_content=10, codebook_size_residual=10,
                    use_gr_x_timbre=True, use_gr_residual_f0=True, use_gr_residual_phone=True)
fe.load_state_dict(torch.load(f"{A.ckpt}/ns3_facodec_encoder.bin", map_location="cpu"))
fd.load_state_dict(torch.load(f"{A.ckpt}/ns3_facodec_decoder.bin", map_location="cpu"))
fe.eval(); fd.eval()

ck = torch.load(A.adapter, map_location="cpu")
K, D, H = ck["K"], ck["D"], ck["hidden"]
net = nn.Sequential(nn.Linear(256, H), nn.GELU(),
                    nn.Dropout(ck.get("dropout", 0.5)), nn.Linear(H, K * D))
net.load_state_dict(ck["state"]); net.eval()

x, sr = sf.read(A.wav, dtype="float32")
if x.ndim > 1:
    x = x.mean(1)
if sr != 16000:
    x = ss.resample_poly(x, 16000, sr).astype(np.float32)
win = x[:int(A.seconds * 16000)]

with torch.no_grad():
    _, _, idx = enc.encode(torch.from_numpy(x).view(1, 1, -1))
    *_, spk = fd(fe(torch.from_numpy(win).float().unsqueeze(0).unsqueeze(0)),
                 eval_vq=False, vq=True)
    t = ((spk.squeeze(0).numpy() - ck["mu"]) / ck["sd"]).astype(np.float32)
    pred = net(torch.tensor(t).unsqueeze(0)).view(K, D).numpy() + ck["pm"]

def dec(p):
    with torch.no_grad():
        pr = torch.from_numpy(np.ascontiguousarray(p)).float().unsqueeze(0)
        i = idx.repeat_interleave(2, dim=0) if vc.get("repeat_input_tokens") else idx
        v = torch.cat([cbm[g](i[:, g]) for g in range(NG)], dim=-1).unsqueeze(0)
        return voc.inference(v, pr)[-1].view(-1).numpy()

def mel(y):
    return librosa.power_to_db(librosa.feature.melspectrogram(
        y=y, sr=vc["sampling_rate"], n_fft=1024, hop_length=256, n_mels=80), ref=1.0)

true_p = wl.extract(x).numpy()
ref = dec(true_p); mb = mel(ref)
def dist(y):
    m = mel(y); n = min(m.shape[1], mb.shape[1])
    return float(np.abs(m[:, :n] - mb[:, :n]).mean())

rep = max(1, 99 // K)
outs = {"true_prompt": ref,
        "adapter": dec(np.repeat(pred.astype(np.float32), rep, axis=0)),
        "onevector_ceiling": dec(np.broadcast_to(true_p.mean(0).astype(np.float32),
                                                 (99, D)).copy())}
base = os.path.splitext(os.path.basename(A.wav))[0]
print("%-24s %8s" % ("decode", "mel dB"), flush=True)
for n, y in outs.items():
    p = os.path.join(A.outdir, "%s_%s.wav" % (base, n))
    sf.write(p, y, vc["sampling_rate"], "PCM_16")
    print("%-24s %8.2f   %s" % (n, dist(y), os.path.basename(p)), flush=True)
print("\nlisten to them; the metric understates how usable the audio is.", flush=True)
