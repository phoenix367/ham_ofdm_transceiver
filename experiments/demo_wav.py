"""Record a complete link session to a WAV file: negotiation start (both
stations bootstrap at EXTREME), transmission speed selection (rung requests
climb in the link-control words), and message transfer at the negotiated
rate -- everything a monitor SDR would hear on the shared simplex frequency.

The channel is realistic on both paths: the station-to-station link carries
Rayleigh fading (0.2 Hz Doppler QSB) + multipath + AWGN, so the transcript
may show ARQ retransmissions where fades ate a frame; and the recording
itself is the MONITOR RECEIVER's audio -- the whole timeline passed once
through the fading channel with a continuous noise floor, so frames rise out
of band noise and occasionally sink into fades.

The WAV is verified by blind-decoding it frame-by-frame (mode auto-detected)
and printing the session transcript with the unpacked link-control words.

Run:  python experiments/demo_wav.py
Output: results/system_demo.wav + transcript
"""

import sys
from pathlib import Path

import numpy as np
from scipy.io import wavfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ofdm_phy import Transceiver
from ofdm_phy.channel import DEFAULT_CHANNEL_RESPONSE, rayleigh_fading
from ofdm_phy.link import LADDER, LinkControl
from ofdm_phy.rf import (StationRF, rf_link, ssb_modulate, ssb_demodulate,
                         expected_cfo_hz, RF_FS, UP)
from ofdm_phy.station import LinkStation, FS
from ofdm_phy.transceiver import DemodError, CODECS, MAPPERS
from ofdm_phy.modes import make_modem
from scipy.signal import fftconvolve, hilbert

import argparse
_ap = argparse.ArgumentParser()
_ap.add_argument("--snr", type=float, default=-2.0,
                 help="nominal SNR toward B; toward A is +1 dB")
_ap.add_argument("--phy", choices=("float", "fixed"), default="float",
                 help="fixed = run BOTH stations and the monitor decode on "
                      "the full fixed-point pipeline (FixedPHY)")
_ap.add_argument("--tag", default=None,
                 help="output-name suffix (default: '' / '_fixed' by --phy)")
_ARGS, _ = _ap.parse_known_args()

if _ARGS.phy == "fixed":
    from ofdm_phy.fixed import FixedPHY
    PHY = FixedPHY()          # stations' PHY (fixed TX + per-mode fixed RX)
    MON_PHY = FixedPHY()      # monitor's own receiver state
else:
    PHY = MON_PHY = None
TAG = _ARGS.tag if _ARGS.tag is not None else ("_fixed" if PHY else "")

SNR_TO_B = _ARGS.snr   # A's signal at B (nominal; Rayleigh fading on top)
SNR_TO_A = _ARGS.snr + 1.0  # B's signal at A
FADING_DOPPLER = 0.2   # Hz, HF-typical QSB
# recording noise floor (vs average power incl. gaps); tracks the session
# SNR so a strong-signal (QAM16-era) session is monitored proportionally --
# high-rate frames need headroom the default floor would deny the monitor
MONITOR_SNR = max(1.0, _ARGS.snr - 1.0)
TICK = 0.1

# real rigs on 40 m: each station one reference oscillator (TX and RX share
# it); the audio CFO between stations EMERGES from these, it is not injected
RF_A = StationRF("A", ppm=+6.0, drift_hz_per_s=+0.03, rf_carrier_hz=7.1e6)
RF_B = StationRF("B", ppm=-8.0, drift_hz_per_s=-0.02, rf_carrier_hz=7.1e6)
RF_MON = StationRF("MON", ppm=+2.0, rf_carrier_hz=7.1e6)  # monitor receiver

MESSAGES_A = [
    (b"CQ", "control"),                                   # negotiation opener
    (b"MSG DE R9FEU: QTH LO88CA OP IVAN 73", "interactive"),
    (bytes(np.random.default_rng(9).integers(0, 256, 200, dtype=np.uint8)), "bulk"),
]
# B's reply traffic, submitted only after A's transfer completes (a QSO
# turn); the reverse direction starts at full speed because the rate
# negotiation already happened in the LC words of phase-1 ACKs
MESSAGES_B = [
    (b"R R TNX MSG DE UB1ABC UR 579 QTH KP50 73", "interactive"),
    (bytes(np.random.default_rng(11).integers(0, 256, 120, dtype=np.uint8)), "bulk"),
]


def run_session():
    rng = np.random.default_rng(5)
    # AFC/netting wired: each station trims its shared reference on request
    A = LinkStation("A", np.random.default_rng(1), phy=PHY,
                    freq_trim_cb=lambda hz: setattr(RF_A, "trim_hz", RF_A.trim_hz + hz))
    B = LinkStation("B", np.random.default_rng(2), phy=PHY,
                    freq_trim_cb=lambda hz: setattr(RF_B, "trim_hz", RF_B.trim_hz + hz))
    for p, q in MESSAGES_A:
        A.submit(p, q)

    peers = {A: B, B: A}
    snr_of = {A: SNR_TO_A, B: SNR_TO_B}
    active = []
    recording = []  # (start_sample, samples, owner)
    session_log = []  # per frame: t, owner, rung, ok, measured snr, delivered
    b_submitted = False
    t = 0.0
    while t < 400:
        for tx in [x for x in active if t >= x["end"]]:
            active.remove(tx)
            tx["owner"].on_tx_end(tx["end"])
            peer = peers[tx["owner"]]
            rf_of = {A: RF_A, B: RF_B}
            rx = rf_link(tx["sig"].astype(np.float64),
                         rf_of[tx["owner"]], rf_of[peer], t0=tx["start"],
                         snr_db=snr_of[peer],
                         time_shift_s=float(rng.uniform(0.03, 0.1)),
                         fading_doppler_hz=FADING_DOPPLER,
                         bit_flip_prob=0.001, erasure_prob=0.02, rng=rng)
            n_before = peer.stats.rx_frames
            peer.rx_frame(rx, tx["end"])
            ok = peer.stats.rx_frames > n_before
            session_log.append({
                "t": tx["end"], "owner": tx["owner"].name, "rung": tx["rung"],
                "ok": ok,
                "snr": peer.ctl._snr_hist[-1][1] if ok else None,
                "delivered_ab": sum(len(m) for m in B.delivered) + sum(len(a) for a in B.rx_assembly.values()),
                "delivered_ba": sum(len(m) for m in A.delivered) + sum(len(a) for a in A.rx_assembly.values()),
            })
            # QSO turn: once A's traffic has fully arrived, B starts its own
            if not b_submitted and len(B.delivered) == len(MESSAGES_A):
                for pl, q in MESSAGES_B:
                    B.submit(pl, q)
                b_submitted = True
                print(f"t={tx['end']:6.1f}s  A->B phase complete, "
                      f"B starts its reply traffic")
        for st in ([A, B] if rng.random() < 0.5 else [B, A]):
            if any(x["owner"] is st for x in active):
                continue
            sig = st.poll_tx(t, len(active) > 0)
            if sig is not None:
                active.append({"owner": st, "start": t, "end": t + len(sig) / FS,
                               "sig": sig, "rung": st.stats.rung_trace[-1][1]})
                # snapshot the LO offset AT TRANSMIT TIME: trims evolve during
                # the session (AFC netting) and the monitor recording must
                # reproduce the historical carrier of each frame
                lo_now = (RF_A if st is A else RF_B).lo_offset_hz(t)
                recording.append((int(t * FS), sig, st.name, lo_now))
        if not active and not A.has_traffic() and not B.has_traffic() \
                and len(B.delivered) == len(MESSAGES_A) \
                and len(A.delivered) == len(MESSAGES_B):
            break
        t += TICK

    assert len(B.delivered) == len(MESSAGES_A), "A->B transfer did not complete"
    assert len(A.delivered) == len(MESSAGES_B), "B->A transfer did not complete"

    # the monitor receiver's audio: each frame is SSB-modulated onto RF with
    # ITS OWNER'S LO, summed on one RF timeline, then faded, noised, and
    # product-detected with the MONITOR's own (third) LO -- so A's and B's
    # frames land at different audio offsets in the recording
    mrng = np.random.default_rng(31)
    total_rf = (int(t * FS) + FS) * UP
    rf_timeline = np.zeros(total_rf, dtype=np.float64)
    for start, sig, owner, lo_hz in recording:
        t_start = start / FS
        frame_rf = ssb_modulate(sig.astype(np.float64), lo_hz, t0=t_start)
        rf_timeline[start * UP: start * UP + len(frame_rf)] += frame_rf

    g = rayleigh_fading(len(rf_timeline), RF_FS, 0.08, mrng)
    rf_timeline = np.real(hilbert(rf_timeline) * g)
    taps_rf = np.zeros((len(DEFAULT_CHANNEL_RESPONSE) - 1) * UP + 1)
    taps_rf[::UP] = DEFAULT_CHANNEL_RESPONSE
    rf_timeline = fftconvolve(rf_timeline, taps_rf, mode="full")
    var = 2.0 * np.mean(rf_timeline ** 2) * 10 ** (-MONITOR_SNR / 10)
    rf_timeline += np.sqrt(var) * mrng.standard_normal(len(rf_timeline))

    monitored = ssb_demodulate(rf_timeline, RF_MON.lo_offset_hz(0.0), t0=0.0)
    audio16 = (monitored / np.max(np.abs(monitored)) * 0.9 * 32767).astype(np.int16)
    return audio16, recording, t, session_log


def transcript(samples):
    """Blind monitor decode of the shared channel (earliest frame first)."""
    x = samples.astype(np.float64) / 32768.0
    window = int(30 * FS)
    pos = 0
    n = 0
    print(f"\n{'t, s':>7} {'mode':8} {'rate':10} {'cfo':>7} {'seq':>3} {'ack':>3} "
          f"{'req_rung -> speed selection':28} {'snr rpt':>8}  payload")
    while pos < len(x) - FS // 2:
        chunk = x[pos:pos + window]
        found = None
        limit = len(chunk)
        while True:
            try:
                if MON_PHY is not None:
                    pkt, stats, mode = MON_PHY.demod_frame_auto(chunk[:limit])
                else:
                    pkt, stats, mode = Transceiver().demod_frame_auto(chunk[:limit])
            except DemodError:
                break
            found = (pkt, stats, mode)
            modem = make_modem(mode)
            pre = 3 * modem._newman_preamble_tile * modem.fft_bins + modem.symbol_len
            new_limit = stats.start_sample - pre
            if new_limit <= FS // 2:
                break
            limit = new_limit
        if found is None:
            # small step: a dense session packs many short frames per
            # window -- skipping half a window because one detected frame
            # failed to decode would discard its neighbors too
            pos += 2 * FS
            continue
        pkt, stats, mode = found
        hdr = stats.header
        lc = LinkControl.unpack(pkt.reserved)
        req = LADDER[min(lc.req_rung, len(LADDER) - 1)]
        rate = f"{hdr.mod.name} {['1/3','1/2','2/3','3/4'][hdr.spd.value]}"
        req_s = f"rung {lc.req_rung} ({req.user_rate:.0f} bit/s)"
        pl = pkt.payload if len(pkt.payload) <= 20 else pkt.payload[:17] + b"..."
        n += 1
        print(f"{(pos + stats.start_sample) / FS:7.1f} {mode.name:8} {rate:10} "
              f"{stats.cfo_hz:+6.1f}Hz {lc.seq:>3} {lc.ack:>3} {req_s:28} "
              f"{lc.snr_db:+6.1f}dB  {pl!r}")

        mapper = MAPPERS[hdr.mod]
        coded = CODECS[hdr.spd].calc_cc_elements(hdr.len)
        n_data = -(-coded // (make_modem(mode).data_carriers_len * mapper.MU))
        pos += stats.start_sample + (6 + n_data) * make_modem(mode).symbol_len
    return n


def plot_timeline(session_log, t_end):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    # panel 1: nominal SNR + per-frame RX-measured SNR (fading visible)
    axes[0].axhline(SNR_TO_B, color="tab:blue", ls="--", lw=1,
                    label=f"nominal toward B ({SNR_TO_B:+.0f} dB)")
    axes[0].axhline(SNR_TO_A, color="tab:orange", ls="--", lw=1,
                    label=f"nominal toward A ({SNR_TO_A:+.0f} dB)")
    for d, color in (("A", "tab:blue"), ("B", "tab:orange")):
        pts = [(e["t"], e["snr"]) for e in session_log
               if e["owner"] == d and e["snr"] is not None]
        if pts:
            axes[0].plot(*zip(*pts), "o-", ms=4, lw=0.8, color=color,
                         label=f"measured at {'B' if d == 'A' else 'A'} "
                               f"({d}->{'B' if d == 'A' else 'A'})")
    axes[0].set_ylabel("SNR, dB")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[0].set_title("System demo over the RF chain: measured SNR, rung trace, delivery"
                      + (" (fixed-point PHY)" if PHY is not None else ""))

    # panel 2: rung trace per direction, losses marked
    for d, color in (("A", "tab:blue"), ("B", "tab:orange")):
        ev = [(e["t"], e["rung"]) for e in session_log if e["owner"] == d]
        if ev:
            axes[1].step(*zip(*ev), where="post", color=color, label=f"{d} TX rung")
        lost = [(e["t"], e["rung"]) for e in session_log
                if e["owner"] == d and not e["ok"]]
        if lost:
            axes[1].scatter(*zip(*lost), marker="x", color="red", zorder=3,
                            label="lost" if d == "A" else None)
    axes[1].set_ylabel("ladder rung")
    axes[1].set_yticks(range(len(LADDER)))
    axes[1].legend(fontsize=9)
    axes[1].grid(alpha=0.3)

    # panel 3: cumulative delivery, both directions of the QSO
    xs = [0] + [e["t"] for e in session_log]
    axes[2].step(xs, [0] + [e["delivered_ab"] for e in session_log],
                 where="post", color="tab:blue", label="A->B")
    axes[2].step(xs, [0] + [e["delivered_ba"] for e in session_log],
                 where="post", color="tab:orange", label="B->A")
    axes[2].set_ylabel("delivered bytes")
    axes[2].set_xlabel("time, s")
    axes[2].legend(fontsize=9)
    axes[2].grid(alpha=0.3)

    fig.tight_layout()
    out = ROOT / "results" / f"system_demo{TAG}_timeline.png"
    fig.savefig(out, dpi=130)
    print(f"saved {out}")


def main():
    print(f"expected CFOs at monitor: A frames {expected_cfo_hz(RF_A, RF_MON, 0):+.1f} Hz, "
          f"B frames {expected_cfo_hz(RF_B, RF_MON, 0):+.1f} Hz")
    print(f"expected station-to-station CFO A->B: {expected_cfo_hz(RF_A, RF_B, 0):+.1f} Hz "
          f"(AFC netting active)\n")
    audio16, recording, dur, session_log = run_session()
    print(f"final trims: A {RF_A.trim_hz:+.1f} Hz, B {RF_B.trim_hz:+.1f} Hz; "
          f"residual A->B offset {expected_cfo_hz(RF_A, RF_B, dur):+.1f} Hz")
    plot_timeline(session_log, dur)
    out = ROOT / "results" / f"system_demo{TAG}.wav"
    wavfile.write(out, FS, audio16)
    print(f"saved {out}  ({len(audio16) / FS:.1f} s, "
          f"{out.stat().st_size / 1024:.0f} KiB, {len(recording)} frames)")
    for start, sig, owner, lo_hz in recording:
        print(f"  t={start / FS:6.1f}s  {owner} transmits {len(sig) / FS:5.1f} s "
              f"(carrier {lo_hz:+6.1f} Hz)")

    fs_read, back = wavfile.read(out)
    n = transcript(back)
    print(f"\ndecoded {n}/{len(recording)} frames from the noisy/fading WAV "
          f"(fade dropouts are expected -- the stations' ARQ recovered them)")
    sys.exit(0 if n >= int(0.6 * len(recording)) else 1)


if __name__ == "__main__":
    main()
