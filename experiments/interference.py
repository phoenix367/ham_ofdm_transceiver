"""Resistance to concentrated co-channel interference.

A third party keys over the channel while we receive one frame. The
interferers, each at a swept interference-to-signal ratio (ISR):

  preamble    the SAME mode's preamble (Newman tones + ZC), keyed back to
              back -- a station stuck in its sync loop, or a beacon;
  preamble_x  another mode's preamble (NORMAL <-> EXTREME): same tone
              comb, different ZC root;
  frames      the same mode's data frames, back to back with short gaps
              -- a second link sharing the channel;
  voice       SSB speech (synthesised: pitched harmonic source, formant
              resonators, syllable envelope, 300-2700 Hz), or a real
              recording with --voice-wav;
  tone        an unmodulated carrier at a random in-band frequency.

Two receivers are measured on byte-identical recordings: the float
frame-at-once model (`Transceiver.demod_frame`, global argmax over the
recording) and the C STREAMING receiver the boards run (`interf_rx`,
built by `make -C cport interfrx`), which commits causally and keeps
searching after a failed header.

The interferer is on for at least 5.1 s (one EXTREME tone field, the
streaming receiver's carrier-finder history) before the frame starts.
ISR is the ratio of the interferer's mean power over the whole recording
to the wanted frame's mean power at the channel output (after multipath),
so a voice interferer with pauses is rated by its long-term power, as a
power meter would. SNR is against the frame's own power as in
`simulate_channel` (multipath on, BSC/BEC off -- those describe fading,
not interference; the no-interference reference is measured in the same
run, so every comparison is internal).

Run:  python experiments/interference.py [--trials N] [--no-c] [--quick] [--workers N]
      (workers default to the core count minus two, one thread each)
Outputs: results/interference[_nN].json / .png
"""

import argparse
import json
import os
import subprocess
import sys
import time
import zlib
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

# One thread per worker: every worker's NumPy would otherwise open its own
# BLAS/OpenMP pool (measured: 8 workers, 71 threads, load 10 on 8 cores),
# and the work here is FFTs and 7x7 solves that gain nothing from it.
# Must precede the numpy import.
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy as np
from scipy.signal import butter, filtfilt, hilbert, lfilter, resample_poly

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ofdm_phy import (Transceiver, Data, ModType, CCSpeed, LinkMode,  # noqa: E402
                      make_modem)
from ofdm_phy.transceiver import DemodError  # noqa: E402
from ofdm_phy.channel import DEFAULT_CHANNEL_RESPONSE  # noqa: E402

FS = 12000
PAYLOAD_LEN = 27
C_BIN = ROOT / "cport" / "build" / "interf_rx"
NET_KEY = 0x5A   # our link's header CRC seed; the stranger's frames use 0
LEAD_MIN = 3 * 160 * 128   # one EXTREME tone field of interferer before the frame

# (label, mode, mod, spd, snr_db): NORMAL BPSK R12 with ~3 dB margin over
# its -6 dB sensitivity and again strong (+10 dB) where only the
# interferer limits; EXTREME BPSK R13 with ~4 dB margin over -19 dB.
CONFIGS = [
    ("NORMAL BPSK R1/2 @ -3 dB", LinkMode.NORMAL, ModType.BPSK, CCSpeed.R12, -3.0),
    ("NORMAL BPSK R1/2 @ +10 dB", LinkMode.NORMAL, ModType.BPSK, CCSpeed.R12, 10.0),
    ("EXTREME BPSK R1/3 @ -15 dB", LinkMode.EXTREME, ModType.BPSK, CCSpeed.R13, -15.0),
]
ITYPES = ["preamble", "preamble_x", "frames", "voice", "tone"]
ISR_GRID = [-20, -15, -10, -6, -3, 0, 3, 6, 10]
REF_ISR = None  # the no-interference reference point

_TRX = {}


def get_trx(mode):
    if mode not in _TRX:
        _TRX[mode] = Transceiver(make_modem(mode), net_key=NET_KEY)
    return _TRX[mode]


# --- interferers ---------------------------------------------------------

def shift_real(x, cfo_hz):
    """Frequency-shift a real audio signal via its analytic form."""
    a = hilbert(x)
    t = np.arange(len(a)) / FS
    return (a * np.exp(2j * np.pi * cfo_hz * t)).real


def tile_to(x, n, rng):
    reps = -(-n // len(x)) + 1
    y = np.tile(x, reps)
    off = int(rng.integers(0, len(x)))
    return y[off:off + n]


def interferer_preamble(mode, n, rng):
    m = get_trx(mode).modem
    pre = np.concatenate(list(m.gen_preamble())).real.astype(np.float64)
    return tile_to(pre, n, rng)


def interferer_frames(mode, mod, spd, n, rng):
    trx = get_trx(mode)
    chunks, total = [], 0
    while total < 2 * n or len(chunks) < 2:
        pkt = Data(reserved=int(rng.integers(0, 1 << 20)),
                   payload=rng.bytes(PAYLOAD_LEN))
        f = trx.build_frame(pkt, mod=mod, spd=spd, net_key=0)   # another net
        gap = np.zeros(int(rng.integers(0, FS // 2)))
        chunks += [f, gap]
        total += len(f) + len(gap)
    return tile_to(np.concatenate(chunks), n, rng)


VOWELS = [(730, 1090, 2440), (270, 2290, 3010), (300, 870, 2240),
          (530, 1840, 2480), (570, 840, 2410), (660, 1720, 2410),
          (440, 1020, 2240), (490, 1350, 1690)]


def synth_voice(n, rng):
    """Speech-like SSB audio: phrases of syllables, each a pitched
    harmonic source (-6 dB/oct) through three formant resonators under a
    raised-cosine envelope, with noise-burst consonants and pauses.
    Activity factor ~55 %, PAPR ~12 dB -- the shape that matters for the
    detectors is the line spectrum (pitch harmonics) that moves every
    syllable."""
    out = np.zeros(n + FS)
    pos = int(rng.integers(0, FS // 4))
    f0_base = float(rng.uniform(95, 220))
    bhp, ahp = butter(2, 1800 / (FS / 2), btype="high")
    while pos < n:
        for _ in range(int(rng.integers(3, 10))):
            if rng.random() < 0.35:  # unvoiced consonant
                d = int(FS * rng.uniform(0.03, 0.08))
                burst = lfilter(bhp, ahp, rng.standard_normal(d))
                burst *= np.hanning(d) * 0.4
                out[pos:pos + d] += burst[:max(0, min(d, len(out) - pos))]
                pos += d
            d = int(FS * rng.uniform(0.09, 0.22))
            t = np.arange(d) / FS
            f0 = f0_base * (1 + rng.uniform(-0.15, 0.15)) * (
                1 + 0.08 * np.sin(2 * np.pi * rng.uniform(1, 3) * t + rng.uniform(0, 2 * np.pi)))
            phase = 2 * np.pi * np.cumsum(f0) / FS
            kmax = int(3500 / f0.min())
            src = np.zeros(d)
            for k in range(1, kmax + 1):
                src += np.cos(k * phase + rng.uniform(0, 2 * np.pi)) / k
            f1, f2, f3 = VOWELS[int(rng.integers(len(VOWELS)))]
            for fc, bw in ((f1, 60), (f2, 90), (f3, 120)):
                r = np.exp(-np.pi * bw / FS)
                src = lfilter([1 - r], [1, -2 * r * np.cos(2 * np.pi * fc / FS), r * r], src)
            env = np.ones(d)
            a, dcy = int(0.02 * FS), int(0.04 * FS)
            env[:a] = 0.5 - 0.5 * np.cos(np.pi * np.arange(a) / a)
            env[-dcy:] = 0.5 + 0.5 * np.cos(np.pi * np.arange(dcy) / dcy)
            src *= env / (np.sqrt(np.mean(src ** 2)) + 1e-9) * 10 ** (rng.uniform(-4, 4) / 20)
            end = min(pos + d, len(out))
            out[pos:end] += src[:end - pos]
            pos += d + int(FS * rng.uniform(0.0, 0.12))
            if pos >= n:
                break
        pos += int(FS * rng.uniform(0.25, 1.0))  # phrase pause
    return out[:n]


_VOICE_WAV = None


def load_voice_wav(path):
    """A real recording, resampled to 12 kHz and mixed to mono."""
    import wave
    with wave.open(str(path), "rb") as w:
        ch, sw, rate, nfr = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(nfr)
    dt = {1: np.int8, 2: np.int16, 4: np.int32}[sw]
    x = np.frombuffer(raw, dtype=dt).astype(np.float64).reshape(-1, ch).mean(axis=1)
    if rate != FS:
        from math import gcd
        g = gcd(rate, FS)
        x = resample_poly(x, FS // g, rate // g)
    return x / (np.max(np.abs(x)) + 1e-9)


def interferer_voice(n, rng):
    if _VOICE_WAV is not None:
        x = tile_to(_VOICE_WAV, n, rng)
    else:
        x = synth_voice(n, rng)
    b, a = butter(4, [300 / (FS / 2), 2700 / (FS / 2)], btype="band")
    return filtfilt(b, a, x)


def interferer_tone(n, rng):
    f = float(rng.uniform(500, 2200))
    t = np.arange(n) / FS
    return np.cos(2 * np.pi * f * t + rng.uniform(0, 2 * np.pi))


def make_interferer(itype, mode, mod, spd, n, rng):
    other = LinkMode.EXTREME if mode is LinkMode.NORMAL else LinkMode.NORMAL
    if itype == "preamble":
        x = interferer_preamble(mode, n, rng)
    elif itype == "preamble_x":
        x = interferer_preamble(other, n, rng)
    elif itype == "frames":
        x = interferer_frames(mode, mod, spd, n, rng)
    elif itype == "voice":
        x = interferer_voice(n, rng)
    elif itype == "tone":
        x = interferer_tone(n, rng)
    else:
        raise ValueError(itype)
    if itype in ("preamble", "preamble_x", "frames"):
        x = shift_real(x, float(rng.uniform(-60, 60)))  # not netted to us
    return x


# --- channel -------------------------------------------------------------

def air(sig, lead, tail, cfo_hz, snr_db, rng):
    """simulate_channel's steps (Hilbert, CFO + 5 Hz quadratic drift,
    multipath, AWGN) with explicit lead/tail padding and the noise sized
    from the FRAME's own power, not the padded recording's. Returns the
    recording and the frame's channel-output power."""
    x = np.concatenate([np.zeros(lead), sig, np.zeros(tail)])
    a = hilbert(x)
    t = np.arange(len(a)) / FS
    dyn = cfo_hz + 5.0 * (t / t[-1]) ** 2
    a = a * np.exp(1j * 2 * np.pi * np.cumsum(dyn) / FS)
    y = np.convolve(a.real, DEFAULT_CHANNEL_RESPONSE, mode="full")
    ps = float(np.mean(np.convolve(sig, DEFAULT_CHANNEL_RESPONSE) ** 2))
    y += np.sqrt(ps * 10 ** (-snr_db / 10)) * rng.standard_normal(len(y))
    return y, ps


# --- receivers -----------------------------------------------------------

def rx_float(trx, rec, packet, truth_start):
    m = trx.modem
    holder = {}
    orig = type(m).detect_preamble

    def wrapped(sig):
        r = orig(m, sig)
        holder["det"] = r
        return r

    m.detect_preamble = wrapped
    try:
        try:
            dec, st = trx.demod_frame(rec)
            ok, reason = dec == packet, "ok" if dec == packet else "wrong"
        except DemodError as exc:
            ok, reason = False, str(exc).split()[0]
        except Exception as exc:  # noqa: BLE001
            ok, reason = False, type(exc).__name__
    finally:
        del m.detect_preamble
    det = holder.get("det")
    # "locked on our frame" = within an eighth of a symbol: the tiled
    # demodulator absorbs whole-tile (128-sample) slips of the EXTREME
    # timing through its per-symbol phase fit, and those still decode
    lock = det is not None and abs(det[0] - truth_start) <= lock_tolerance(m)
    return ok, reason, lock


def lock_tolerance(m):
    return max(m.cyclic_prefix, m.symbol_len // 8)


def rx_c(rec, mode, packet_bits, truth_start, tag):
    """Run the C streaming receiver on the recording; returns (ok,
    n_commits, n_other_decodes, locked_on_truth). Other decodes are
    CRC-valid frames that are not ours -- a stranger's own frames."""
    tol = lock_tolerance(get_trx(mode).modem) + 31  # fixed chain: Hilbert group delay
    path = Path(os.environ.get("INTERF_TMP", "/tmp")) / f"interf_{os.getpid()}_{tag}.raw"
    x = rec / (np.max(np.abs(rec)) + 1e-12) * 0.9 * 32767  # AGC to full scale
    x.astype(np.int16).tofile(path)
    try:
        out = subprocess.run([str(C_BIN), "-m", str(mode.value), "-k", str(NET_KEY), str(path)],
                             capture_output=True, text=True, check=True).stdout
    finally:
        path.unlink(missing_ok=True)
    want = "".join("1" if b else "0" for b in packet_bits)
    ok, commits, false_dec, lock, preempts, notches = False, 0, 0, False, 0, 0
    for line in out.splitlines():
        if line.startswith("END "):
            toks = line.split()
            preempts = sum(int(t.split(":")[1]) for t in toks if ":" in t)
            if "notches" in toks:
                notches = int(toks[toks.index("notches") + 1])
            continue
        if not line.startswith("EV "):
            continue
        _, _m, typ, start, _snr, nbits, *bits = line.split()
        commits += 1
        if int(typ) == 1:
            if bits and bits[0] == want:
                ok = True
                lock = lock or abs(int(start) - truth_start) <= tol
            else:
                false_dec += 1
    return ok, commits, false_dec, lock, preempts, notches


# --- one point -------------------------------------------------------------

def run_chunk(args):
    cfg_i, itype, isr_db, seed, trials, use_c = args
    _, mode, mod, spd, snr_db = CONFIGS[cfg_i]
    rng = np.random.default_rng(seed)
    trx = get_trx(mode)
    m = trx.modem
    pre_len = sum(len(c) for c in m.gen_preamble())
    acc = dict(trials=0, f_ok=0, f_lock=0, f_reasons={}, c_ok=0, c_commits=0,
               c_false=0, c_trials=0, c_time=0.0, c_preempts=0, c_notches=0)
    for _ in range(trials):
        packet = Data(reserved=int(rng.integers(0, 1 << 20)), payload=rng.bytes(PAYLOAD_LEN))
        sig = trx.build_frame(packet, mod=mod, spd=spd)
        # the interferer is on for at least one EXTREME tone field (5.1 s)
        # before the frame: the streaming receiver's carrier finder needs
        # that much history whatever its mode (rx_stream.c EXC_HIST_SAMPLES),
        # so this measures its excision in steady state, not at onset
        lead = int(rng.integers(LEAD_MIN + FS // 4, LEAD_MIN + 2 * FS))
        tail = int(rng.integers(FS // 4, FS))
        rec, ps = air(sig, lead, tail, float(rng.uniform(-50, 50)), snr_db, rng)
        if isr_db is not None:
            itf = make_interferer(itype, mode, mod, spd, len(rec), rng)
            pi = float(np.mean(itf ** 2))
            rec = rec + itf * np.sqrt(ps * 10 ** (isr_db / 10) / pi)
        truth = lead + pre_len
        ok, reason, lock = rx_float(trx, rec, packet, truth)
        acc["trials"] += 1
        acc["f_ok"] += ok
        acc["f_lock"] += lock
        acc["f_reasons"][reason] = acc["f_reasons"].get(reason, 0) + 1
        if use_c:
            t0 = time.time()
            cok, commits, cfalse, _, pre, notch = rx_c(rec, mode, packet.encode(), truth,
                                                       f"{cfg_i}_{itype}_{isr_db}")
            acc["c_time"] += time.time() - t0
            acc["c_trials"] += 1
            acc["c_ok"] += cok
            acc["c_commits"] += commits
            acc["c_false"] += cfalse
            acc["c_preempts"] += pre
            acc["c_notches"] += notch
    return (cfg_i, itype, isr_db), acc


# --- receiver capture: interferer alone ------------------------------------

def run_capture(args):
    """No wanted frame at all: 60 s of one interferer at +10 dB over the
    noise. Counts what the receivers do with it -- every commit on the C
    receiver is a header attempt that costs it that much listening time,
    and a float 'detection' is a frame-at-once receiver that would have
    spent its whole recording on the stranger."""
    cfg_i, itype, seed, secs, use_c = args
    _, mode, mod, spd, _ = CONFIGS[cfg_i]
    rng = np.random.default_rng(seed)
    trx = get_trx(mode)
    n = secs * FS
    noise = rng.standard_normal(n)
    itf = make_interferer(itype, mode, mod, spd, n, rng)
    rec = noise + itf * np.sqrt(10 ** (10 / 10) / np.mean(itf ** 2))
    det = trx.modem.detect_preamble(hilbert(rec).astype(np.complex64))
    res = dict(secs=secs, f_detected=det is not None)
    if use_c:
        _, commits, cfalse, _, pre, notch = rx_c(rec, mode, np.zeros(1, dtype=np.uint8), -1,
                                                 f"cap_{cfg_i}_{itype}")
        res.update(c_commits=commits, c_false=cfalse, c_preempts=pre, c_notches=notch)
    return (cfg_i, itype), res


def isr_at(grid, per, level):
    """First ISR (dB, linear interpolation) where PER crosses `level`;
    None if it never does, or if it is above it already at the lowest
    point (a floor, not an interference threshold)."""
    if per[0] > level:
        return None
    for i in range(1, len(grid)):
        if per[i] > level:
            a, b = per[i - 1], per[i]
            return grid[i - 1] + (grid[i] - grid[i - 1]) * (level - a) / (b - a)
    return None


def fmt_thr(x):
    return "never" if x is None else f"{x:+.1f} dB"


# --- main ------------------------------------------------------------------

def main():
    global _VOICE_WAV
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=60, help="frames per (config, interferer, ISR)")
    ap.add_argument("--chunk", type=int, default=5)
    ap.add_argument("--no-c", action="store_true", help="skip the C streaming receiver")
    ap.add_argument("--quick", action="store_true", help="NORMAL @ -3 dB only, coarse ISR grid")
    ap.add_argument("--configs", type=str, default=None, help="comma list of config indices")
    ap.add_argument("--itypes", type=str, default=None, help="comma list of interferer types")
    ap.add_argument("--voice-wav", type=str, default=None, help="real speech recording for 'voice'")
    ap.add_argument("--capture-secs", type=int, default=60)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2),
                    help="worker processes (default: cores minus two)")
    args = ap.parse_args()

    use_c = not args.no_c
    if use_c and not C_BIN.exists():
        subprocess.run(["make", "-C", str(ROOT / "cport"), "interfrx"], check=True,
                       capture_output=True)
    os.environ["INTERF_TMP"] = os.environ.get("INTERF_TMP",
                                              str(Path(os.environ.get("TMPDIR", "/tmp"))))
    if args.voice_wav:
        _VOICE_WAV = load_voice_wav(args.voice_wav)

    cfg_ids = list(range(len(CONFIGS)))
    isr_grid = list(ISR_GRID)
    if args.quick:
        cfg_ids, isr_grid = [0], [-10, -3, 0, 3, 10]
    if args.configs:
        cfg_ids = [int(c) for c in args.configs.split(",")]
    itypes = args.itypes.split(",") if args.itypes else list(ITYPES)

    jobs = []
    for ci in cfg_ids:
        for it in itypes:
            for isr in ([REF_ISR] if it == itypes[0] else []) + isr_grid:
                n_chunks = -(-args.trials // args.chunk)
                for c in range(n_chunks):
                    trials = min(args.chunk, args.trials - c * args.chunk)
                    seed = zlib.crc32(f"{ci}:{it}:{isr}:{c}".encode())
                    jobs.append((ci, it, isr, seed, trials, use_c))
    cap_jobs = [(ci, it, zlib.crc32(f"cap:{ci}:{it}".encode()), args.capture_secs, use_c)
                for ci in cfg_ids for it in itypes]
    print(f"{len(cfg_ids)} configs x {len(ITYPES)} interferers x {len(isr_grid)} ISR points, "
          f"{args.trials} frames/point, {len(jobs)} chunks + {len(cap_jobs)} capture runs, "
          f"C receiver {'on' if use_c else 'off'}, {args.workers} workers", flush=True)

    acc, caps = {}, {}
    t0 = time.time()
    # EXTREME chunks are ~40x the cost of NORMAL ones: submit them first so
    # the pool's tail is short
    jobs.sort(key=lambda j: -CONFIGS[j[0]][1].value)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futs = [pool.submit(run_capture, j) for j in cap_jobs]
        futs += [pool.submit(run_chunk, j) for j in jobs]
        done = 0
        for fut in futs:
            key, r = fut.result()
            done += 1
            if len(key) == 2:
                caps[key] = r
                continue
            a = acc.setdefault(key, dict(trials=0, f_ok=0, f_lock=0, f_reasons={}, c_ok=0,
                                         c_commits=0, c_false=0, c_trials=0, c_time=0.0,
                                         c_preempts=0, c_notches=0))
            for k, v in r.items():
                if k == "f_reasons":
                    for rk, rv in v.items():
                        a[k][rk] = a[k].get(rk, 0) + rv
                else:
                    a[k] += v
            if done % 50 == 0:
                print(f"  {done}/{len(futs)} chunks, {time.time() - t0:.0f} s", flush=True)

    # ---- report
    out = dict(trials=args.trials, isr_grid=isr_grid, snr_convention="frame power after multipath",
               isr_convention="interferer mean power over the recording / frame power",
               configs=[], capture=[])
    for ci in cfg_ids:
        label, mode, mod, spd, snr = CONFIGS[ci]
        ref = acc[(ci, itypes[0], REF_ISR)]
        ref_per = 1 - ref["f_ok"] / ref["trials"]
        ref_cper = (1 - ref["c_ok"] / ref["c_trials"]) if use_c else None
        print(f"\n=== {label}   reference PER (no interferer): float {ref_per * 100:.1f} %"
              + (f", C stream {ref_cper * 100:.1f} %" if use_c else ""))
        print("  ISR dB:      " + " ".join(f"{i:6d}" for i in isr_grid))
        cfg = dict(label=label, mode=mode.name, mod=mod.name, spd=spd.name, snr_db=snr,
                   ref_per_float=ref_per, ref_per_c=ref_cper, curves={})
        for it in itypes:
            fper, cper, lock, commits, preempts, notches = [], [], [], [], [], []
            for isr in isr_grid:
                a = acc[(ci, it, isr)]
                fper.append(1 - a["f_ok"] / a["trials"])
                lock.append(a["f_lock"] / a["trials"])
                if use_c:
                    cper.append(1 - a["c_ok"] / a["c_trials"])
                    commits.append(a["c_commits"] / a["c_trials"])
                    preempts.append(a["c_preempts"] / a["c_trials"])
                    notches.append(a["c_notches"] / a["c_trials"])
            print(f"  {it:11s} float PER%  " + " ".join(f"{p * 100:6.1f}" for p in fper))
            if use_c:
                print(f"  {'':11s} C-str PER%  " + " ".join(f"{p * 100:6.1f}" for p in cper))
                print(f"  {'':11s} C commits   " + " ".join(f"{c:6.2f}" for c in commits))
                print(f"  {'':11s} C preempts  " + " ".join(f"{c:6.2f}" for c in preempts))
                print(f"  {'':11s} C notched   " + " ".join(f"{c:6.2f}" for c in notches))
            print(f"  {'':11s} float lock% " + " ".join(f"{l * 100:6.1f}" for l in lock))
            thr_f = isr_at(isr_grid, fper, 0.10)
            thr_c = isr_at(isr_grid, cper, 0.10) if use_c else None
            print(f"  {'':11s} ISR @ PER 10 %: float {fmt_thr(thr_f)}"
                  + (f", C stream {fmt_thr(thr_c)}" if use_c else ""))
            cfg["curves"][it] = dict(per_float=fper, per_c=cper if use_c else None,
                                     isr_at_per10_float=thr_f, isr_at_per10_c=thr_c,
                                     lock_float=lock, c_commits_per_frame=commits if use_c else None,
                                     c_preempts_per_frame=preempts if use_c else None,
                                     c_notches_per_frame=notches if use_c else None,
                                     reasons={str(isr): acc[(ci, it, isr)]["f_reasons"]
                                              for isr in isr_grid})
        out["configs"].append(cfg)

    print(f"\n=== receiver capture: {args.capture_secs} s of interferer alone at +10 dB over noise")
    for ci in cfg_ids:
        label = CONFIGS[ci][0]
        for it in itypes:
            r = caps[(ci, it)]
            line = f"  {label:28s} {it:11s} float detects: {'YES' if r['f_detected'] else 'no '}"
            if use_c:
                line += f"   C commits: {r['c_commits']:4d} ({r['c_commits'] / r['secs'] * 60:.1f}/min)" \
                        f"  preempts: {r.get('c_preempts', 0)} notches: {r.get('c_notches', 0)}" \
                        f"  decodes: {r['c_false']}" + ("  (the stranger's own frames)" if it == "frames" else "")
            print(line)
            out["capture"].append(dict(config=label, itype=it, **r))

    suffix = "" if args.trials == 60 and not args.quick and not args.configs and not args.itypes \
        else f"_n{args.trials}"
    if args.quick:
        suffix += "_quick"
    jpath = ROOT / "results" / f"interference{suffix}.json"
    jpath.write_text(json.dumps(out, indent=1))
    plot(out, ROOT / "results" / f"interference{suffix}.png", use_c)
    print(f"\nwrote {jpath} and .png in {time.time() - t0:.0f} s")


def plot(out, path, use_c):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = len(out["configs"])
    fig, axes = plt.subplots(1, n, figsize=(5.2 * n, 4.4), squeeze=False)
    colors = dict(zip(ITYPES, ["C3", "C1", "C0", "C2", "C4"]))
    for ax, cfg in zip(axes[0], out["configs"]):
        x = out["isr_grid"]
        for it, cv in cfg["curves"].items():
            ax.plot(x, [p * 100 for p in cv["per_float"]], "-o", ms=4, color=colors[it],
                    label=f"{it} (float)")
            if use_c and cv["per_c"]:
                ax.plot(x, [p * 100 for p in cv["per_c"]], "--s", ms=3, color=colors[it],
                        alpha=0.7, label=f"{it} (C stream)")
        ax.axhline(cfg["ref_per_float"] * 100, color="k", ls=":", lw=1, label="no interferer")
        ax.set_title(cfg["label"], fontsize=10)
        ax.set_xlabel("interference-to-signal ratio, dB")
        ax.set_ylabel("PER, %")
        ax.set_ylim(-2, 102)
        ax.grid(alpha=0.3)
    axes[0][0].legend(fontsize=7, ncol=2)
    fig.suptitle("One frame under a third party's continuous signal", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=120)


if __name__ == "__main__":
    main()
