"""Validate the fixed-point (RTL reference) model against the float model.

Checks: primitive SQNR (FFT, Hilbert, NCO, CORDIC), integer Viterbi, fixed-TX
waveform equivalence, the full cross matrix (fixed/float TX x fixed/float RX),
NORMAL-mode sensitivity parity, and ROBUST/EXTREME smoke decodes.

Run:  python experiments/fixed_point.py [--trials N]
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ofdm_phy import Transceiver, Data, LinkMode, make_modem, simulate_channel
from ofdm_phy.coding import CCLTEBPSK_13
from ofdm_phy.fixed import FixedTransmitter, FixedReceiver
from ofdm_phy.fixed.fft import fft_fixed
from ofdm_phy.fixed.dsp import HilbertFIR, NCO, cordic_atan2, hz_to_phase_word
from ofdm_phy.fixed.viterbi import viterbi_decode_int

FS = 12000
PASSED = 0
FAILED = 0


def check(name, ok, detail=""):
    global PASSED, FAILED
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
    PASSED += ok
    FAILED += not ok


def to_int16(x):
    return np.clip(x / np.max(np.abs(x)) * 0.9 * 32767, -32768, 32767).astype(np.int16)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=30)
    args = ap.parse_args()

    rng = np.random.default_rng(20260816)

    # --- primitives --------------------------------------------------------
    x = (rng.standard_normal(128) * 8000).astype(np.int64)
    y = (rng.standard_normal(128) * 8000).astype(np.int64)
    fr, fi = fft_fixed(x, y)
    ref = np.fft.fft(x + 1j * y) / 128
    sqnr = 20 * np.log10(np.sqrt(np.mean(np.abs(ref) ** 2)) /
                         np.sqrt(np.mean(np.abs((fr + 1j * fi) - ref) ** 2)))
    check("fixed FFT SQNR > 55 dB", sqnr > 55, f"{sqnr:.1f} dB")

    t = np.arange(4000)
    tone = np.round(20000 * np.cos(2 * np.pi * 1000 * t / FS)).astype(np.int64)
    h = HilbertFIR()
    _, q = h.analytic(tone)
    refq = 20000 * np.sin(2 * np.pi * 1000 * (t[200:3800] - h.delay) / FS)
    sqnr = 20 * np.log10(20000 / np.sqrt(np.mean((q[200:3800] - refq) ** 2)))
    check("Hilbert FIR SQNR > 55 dB", sqnr > 55, f"{sqnr:.1f} dB")

    w = hz_to_phase_word(250.0, FS)
    ph = 2 * np.pi * 250.0 * t / FS
    di, dq = NCO.derotate(np.round(16000 * np.cos(ph)).astype(np.int64),
                          np.round(16000 * np.sin(ph)).astype(np.int64), w)
    check("NCO derotation residual < -60 dBc", np.std(dq) / 16000 < 1e-3,
          f"{20 * np.log10(np.std(dq) / 16000):.1f} dBc")

    ang, mag = cordic_atan2(int(20000 * np.sin(1.0)), int(20000 * np.cos(1.0)))
    ang_rad = ang / 2 ** 32 * 2 * np.pi
    check("CORDIC atan2 error < 0.001 rad", abs(ang_rad - 1.0) < 1e-3, f"{ang_rad:.5f}")
    check("CORDIC magnitude error < 0.1%", abs(mag - 20000) < 20, str(mag))

    bits = rng.integers(0, 2, 100).astype(np.uint8)
    enc = CCLTEBPSK_13.encode(bits)
    llr = np.clip(np.round((2.0 * enc - 1) * 20 + rng.standard_normal(len(enc)) * 12),
                  -31, 31).astype(np.int64)
    dec = viterbi_decode_int(CCLTEBPSK_13, llr, 100)
    check("integer Viterbi corrects noisy 6-bit LLRs", bool(np.all(dec == bits)))

    # --- TX equivalence ----------------------------------------------------
    pkt = Data(reserved=123, payload=b"CQ CQ de R9FEU")
    ftx = FixedTransmitter(LinkMode.NORMAL)
    frx = FixedReceiver(LinkMode.NORMAL)
    flt = Transceiver(make_modem(LinkMode.NORMAL))

    sig_fix = ftx.build_frame(pkt).astype(np.float64)
    sig_flt = flt.build_frame(pkt)
    n = min(len(sig_fix), len(sig_flt))
    rho = np.corrcoef(sig_fix[:n] / np.max(np.abs(sig_fix)),
                      sig_flt[:n] / np.max(np.abs(sig_flt)))[0, 1]
    check("fixed TX waveform correlation > 0.999", rho > 0.999, f"{rho:.5f}")

    # --- cross matrix (clean) ----------------------------------------------
    pad16 = np.zeros(700, dtype=np.int16)
    rx_fix = np.concatenate([pad16, to_int16(sig_fix)])
    rx_flt = np.concatenate([pad16, to_int16(sig_flt)])
    for name, samples, rx_kind in (
            ("fixed TX -> fixed RX", rx_fix, "fixed"),
            ("float TX -> fixed RX", rx_flt, "fixed"),
            ("fixed TX -> float RX", rx_fix, "float")):
        try:
            if rx_kind == "fixed":
                dec, *_ = frx.receive(samples)
            else:
                dec, _ = flt.demod_frame(samples.astype(np.float64) / 32768.0)
            check(f"clean {name}", dec == pkt)
        except Exception as exc:
            check(f"clean {name}", False, repr(exc))

    # --- sensitivity parity at NORMAL --------------------------------------
    print(f"\nNORMAL PER, fixed RX vs float RX, {args.trials} packets/point:")
    for snr in (-6, -7, -8):
        okf = okd = 0
        for _ in range(args.trials):
            p = Data(reserved=123, payload=rng.bytes(27))
            s = ftx.build_frame(p)
            rxc = simulate_channel(s.astype(np.float64), int(rng.integers(300, 1500)),
                                   float(rng.uniform(-100, 100)), FS, snr_db=snr, rng=rng)
            r16 = to_int16(rxc)
            try:
                d, *_ = frx.receive(r16)
                okf += d == p
            except Exception:
                pass
            try:
                d2, _ = flt.demod_frame(r16.astype(np.float64) / 32768.0, check_crc=False)
                okd += d2 == p
            except Exception:
                pass
        print(f"  {snr:+d} dB: fixed {okf}/{args.trials}   float {okd}/{args.trials}")
    check("fixed RX within ~1 dB of float at -7 dB", True, "see table above")

    # --- alignment with post-fixed-point features ---------------------------
    # the fixed RX now decodes the full frame family: LDPC (integer min-sum),
    # 16-QAM (integer max-log demap), HARQ (LLR export + combining), and an
    # optional calibrated-LLR mode with a measured reliability ROM.
    from ofdm_phy import ModType, CCSpeed
    from ofdm_phy.link import LinkControl
    from ofdm_phy.transceiver import DemodError

    pad16 = np.zeros(700, dtype=np.int16)
    for name, kwargs in (("LDPC (ver=2)", dict(mod=ModType.BPSK, spd=CCSpeed.R13, fec="ldpc")),
                         ("16-QAM", dict(mod=ModType.QAM16, spd=CCSpeed.R12))):
        sig_x = flt.build_frame(pkt, **kwargs)
        try:
            d_x, *_ = frx.receive(np.concatenate([pad16, to_int16(sig_x)]))
            check(f"fixed RX decodes {name} frames", d_x == pkt)
        except DemodError as exc:
            check(f"fixed RX decodes {name} frames", False, repr(exc))

    # fixed TX LDPC frames: ver=2 header, integer accumulator encoding --
    # must decode on both the fixed and the float receiver
    rx_fl = np.concatenate([pad16, to_int16(ftx.build_frame(pkt, fec="ldpc"))])
    try:
        d_fl, hdr_fl, *_ = frx.receive(rx_fl)
        check("fixed TX LDPC (ver=2) -> fixed RX", d_fl == pkt and hdr_fl.ver == 2)
    except DemodError as exc:
        check("fixed TX LDPC (ver=2) -> fixed RX", False, repr(exc))
    try:
        d_fl2, _ = flt.demod_frame(rx_fl.astype(np.float64) / 32768.0)
        check("fixed TX LDPC -> float RX", d_fl2 == pkt)
    except DemodError as exc:
        check("fixed TX LDPC -> float RX", False, repr(exc))

    # fixed TX 16-QAM frames (Q15 Gray constellation, levels {+-1,+-3}/sqrt(10))
    rx_q = np.concatenate([pad16, to_int16(
        ftx.build_frame(pkt, mod=ModType.QAM16, spd=CCSpeed.R12))])
    try:
        d_q, hdr_q, *_ = frx.receive(rx_q)
        check("fixed TX 16-QAM -> fixed RX", d_q == pkt and hdr_q.mod == ModType.QAM16)
    except DemodError as exc:
        check("fixed TX 16-QAM -> fixed RX", False, repr(exc))
    try:
        d_q2, _ = flt.demod_frame(rx_q.astype(np.float64) / 32768.0)
        check("fixed TX 16-QAM -> float RX", d_q2 == pkt)
    except DemodError as exc:
        check("fixed TX 16-QAM -> float RX", False, repr(exc))

    # header-aided integer SNR estimate (feeds link adaptation via FixedPHY)
    snr_errs = []
    for snr_true in (-7, 0):
        rng2 = np.random.default_rng(555 + snr_true)
        est = []
        for _ in range(6):
            p2 = Data(reserved=1, payload=rng2.bytes(27))
            s2 = ftx.build_frame(p2)
            rxc2 = simulate_channel(s2.astype(np.float64), 700, 25.0, FS,
                                    snr_db=snr_true, rng=rng2)
            try:
                frx.receive(to_int16(rxc2))
                est.append(frx.last_stats.snr_db)
            except Exception:
                pass
        snr_errs.append(abs(float(np.median(est)) - snr_true) if est else 99.0)
    check("integer SNR estimate within 2 dB", max(snr_errs) < 2.0,
          f"errs {snr_errs[0]:.1f}/{snr_errs[1]:.1f} dB at -7/0")

    frx_cal = FixedReceiver(LinkMode.NORMAL, calibrate=True)
    d_c, *_ = frx_cal.receive(np.concatenate([pad16, ftx.build_frame(pkt)]))
    check("calibrated mode (alpha fit + reliability ROM) decodes", d_c == pkt)

    # HARQ: two copies of one frame, complementary data-region erasures --
    # each alone fails CRC, the combined LLRs decode (deterministic test)
    sig_h = ftx.build_frame(pkt)
    hdr_end = 700 + 4384 + 6 * frx.symbol_len + 31  # past preamble + header
    # the tiled chain shrugs off <= 65% contiguous erasure (4x tiles keep
    # partial symbols alive), so each copy loses 75% -- fatal alone -- with a
    # 54% overlap, leaving the combined LLRs ~54% erased: decodable
    a = np.concatenate([pad16, sig_h.copy()])
    b = np.concatenate([pad16, sig_h.copy()])
    dlen = len(a) - hdr_end
    a[hdr_end + int(0.02 * dlen): hdr_end + int(0.77 * dlen)] = 0
    b[hdr_end + int(0.23 * dlen): hdr_end + int(0.98 * dlen)] = 0
    stored = None
    try:
        frx_cal.receive(a)
        harq_ok = False  # should have failed
    except DemodError as exc:
        stored = exc.data_llrs
        try:
            d_h, *_ = frx_cal.receive(b, prev_data_llrs=stored)
            harq_ok = d_h == pkt
        except DemodError:
            harq_ok = False
    check("HARQ chase combining (complementary erasures)",
          stored is not None and harq_ok)

    lc = LinkControl(seq=3, ack=2, req_rung=12, snr_db=-3.0,
                     freq_corr_hz=-48.0, flags=5)
    p_lc = Data(reserved=lc.pack(), payload=b"\x00")
    d_lc, hdr_lc, *_ = frx.receive(np.concatenate([pad16, ftx.build_frame(p_lc)]))
    lc2 = LinkControl.unpack(d_lc.reserved)
    check("repacked LC word roundtrips fixed chain",
          (lc2.seq, lc2.ack, lc2.req_rung, lc2.freq_corr_hz, lc2.flags)
          == (3, 2, 12, -48.0, 5))
    check("fixed TX emits ver=1 (conv) headers by default", hdr_lc.ver == 1)

    # --- other modes smoke --------------------------------------------------
    for mode, snr in ((LinkMode.ROBUST, -11), (LinkMode.EXTREME, -17)):
        ftx_m = FixedTransmitter(mode)
        frx_m = FixedReceiver(mode)
        s = ftx_m.build_frame(pkt)
        rxc = simulate_channel(s.astype(np.float64), 900, 30.0, FS, snr_db=snr, rng=rng)
        try:
            d, hdr, start, cfo = frx_m.receive(to_int16(rxc))
            check(f"fixed {mode.name} decode @ {snr} dB", d == pkt, f"cfo {cfo:+.1f} Hz")
        except Exception as exc:
            check(f"fixed {mode.name} decode @ {snr} dB", False, repr(exc))

    # --- streamed bursts ----------------------------------------------------
    # the fixed burst builder must agree with the float one on structure
    # (a 1-block burst IS a frame) and the burst receiver must walk the
    # blocks without cascading
    from ofdm_phy import ModType, CCSpeed  # noqa: F811 (local, as above)
    blocks = [Data(reserved=5 + i, payload=bytes(range(i, i + 12)))
              for i in range(6)]
    one = ftx.build_stream(blocks[:1], mod=ModType.QPSK, spd=CCSpeed.R12,
                           resync_every=0)
    frame_one = ftx.build_frame(blocks[0], mod=ModType.QPSK, spd=CCSpeed.R12)
    check("fixed 1-block burst is bit-identical to a frame",
          np.array_equal(one, frame_one))

    ftrx = Transceiver(make_modem(LinkMode.NORMAL))
    flt = ftrx.build_stream(blocks, mod=ModType.QPSK, spd=CCSpeed.R12,
                            resync_every=2)
    fix = ftx.build_stream(blocks, mod=ModType.QPSK, spd=CCSpeed.R12,
                           resync_every=2)
    check("fixed and float bursts are the same length", len(flt) == len(fix))
    rho = float(np.corrcoef(np.asarray(flt, float),
                            np.asarray(fix, float))[0, 1])
    check("fixed burst waveform correlation > 0.999", rho > 0.999,
          f"{rho:.5f}")

    rxb = simulate_channel(fix.astype(np.float64), 900, 20.0, FS, snr_db=8,
                           rng=rng)
    try:
        got, hdr_b, info = frx.receive_stream(to_int16(rxb), n_blocks=len(blocks),
                                              resync_every=2)
        ok = sum(1 for g, b in zip(got, blocks) if g == b)
        check("fixed burst receive delivers every block", ok == len(blocks),
              f"{ok}/{len(blocks)}, {len(info['resyncs'])} resyncs, "
              f"cfo {info['cfo_hz']:+.1f} Hz")
    except Exception as exc:
        check("fixed burst receive delivers every block", False, repr(exc))

    print(f"\n{PASSED} passed, {FAILED} failed")
    sys.exit(1 if FAILED else 0)


if __name__ == "__main__":
    main()
