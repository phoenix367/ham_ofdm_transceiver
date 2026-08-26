"""End-to-end smoke test: TX frame -> simulated channel -> RX decode."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ofdm_phy import (
    Transceiver, Beacon, Data, ModType, CCSpeed, simulate_channel,
)

trx = Transceiver()
rng = np.random.default_rng(42)


def run_case(name, packet, mod, spd, channel_kwargs=None, time_shift=700, freq_shift_hz=0.0):
    sig = trx.build_frame(packet, mod=mod, spd=spd)
    if channel_kwargs is None:
        rx_sig = np.concatenate([np.zeros(time_shift), sig])
    else:
        rx_sig = simulate_channel(sig, time_shift=time_shift, freq_shift_hz=freq_shift_hz,
                                  sample_rate=trx.modem.sample_rate, rng=rng, **channel_kwargs)
    try:
        decoded, stats = trx.demod_frame(rx_sig)
        ok = decoded == packet
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {decoded}")
        print(f"        start={stats.start_sample} cfo={stats.cfo_hz:+.2f}Hz "
              f"EsN0={stats.es_n0_db:+.2f}dB SNR={stats.snr_db:+.2f}dB BER={stats.ber * 100:.2f}%")
        return ok
    except Exception as exc:
        print(f"[FAIL] {name}: {type(exc).__name__}: {exc}")
        return False


beacon = Beacon(callsign="R9FEU", qth="LO88CA")
data = Data(reserved=123, payload=b"    Though this be madness,")

results = []
results.append(run_case("clean beacon BPSK R13", beacon, ModType.BPSK, CCSpeed.R13))
results.append(run_case("clean data QPSK R12", data, ModType.QPSK, CCSpeed.R12))

clean_ch = dict(snr_db=30, channel_response=np.array([1.0]), bit_flip_prob=0, erasure_prob=0)
results.append(run_case("high-SNR channel, CFO +100 Hz", data, ModType.BPSK, CCSpeed.R13,
                        clean_ch, freq_shift_hz=100.0))
results.append(run_case("high-SNR channel, CFO -250 Hz", data, ModType.BPSK, CCSpeed.R13,
                        clean_ch, freq_shift_hz=-250.0))

article_ch = dict(snr_db=-6)  # multipath + BSC/BEC defaults
results.append(run_case("article channel @ -6 dB, CFO +30 Hz (beacon BPSK R13)",
                        beacon, ModType.BPSK, CCSpeed.R13, article_ch, freq_shift_hz=30.0))
results.append(run_case("article channel @ -3 dB (data QPSK R13)",
                        data, ModType.QPSK, CCSpeed.R13, dict(snr_db=-3), freq_shift_hz=-40.0))

# STF-style preamble variant (tones every 8 bins, delay-and-correlate CFO)
from ofdm_phy import STFOFDMModem

trx = Transceiver(STFOFDMModem())
results.append(run_case("STF preamble @ -6 dB, CFO -280 Hz (data BPSK R13)",
                        data, ModType.BPSK, CCSpeed.R13, article_ch, freq_shift_hz=-280.0))

# adaptive link modes: ROBUST (16x tiles) at -12 dB, EXTREME (64x) at -18 dB,
# plus mode auto-detection from the per-mode ZC preamble root
from ofdm_phy import LinkMode, make_modem

trx = Transceiver(make_modem(LinkMode.ROBUST))
results.append(run_case("ROBUST mode @ -12 dB (data BPSK R13)",
                        data, ModType.BPSK, CCSpeed.R13, dict(snr_db=-12), freq_shift_hz=40.0))
trx = Transceiver(make_modem(LinkMode.EXTREME))
results.append(run_case("EXTREME mode @ -18 dB (data BPSK R13)",
                        data, ModType.BPSK, CCSpeed.R13, dict(snr_db=-18), freq_shift_hz=-60.0))

sig = Transceiver(make_modem(LinkMode.ROBUST)).build_frame(data, mod=ModType.BPSK, spd=CCSpeed.R13)
rx_sig = simulate_channel(sig, time_shift=900, freq_shift_hz=25.0,
                          sample_rate=12000, snr_db=-10, rng=rng)
try:
    dec, st, mode = Transceiver().demod_frame_auto(rx_sig)
    ok = dec == data and mode is LinkMode.ROBUST
    print(f"[{'PASS' if ok else 'FAIL'}] auto-mode RX @ -10 dB: mode={mode.name}, {dec}")
    results.append(ok)
except Exception as exc:
    print(f"[FAIL] auto-mode RX: {exc}")
    results.append(False)

# streamed burst: one preamble + one header for 8 packets, ZC resync every 4
blocks = [Data(reserved=123, payload=bytes([65 + k]) * 27) for k in range(8)]
trx = Transceiver(make_modem(LinkMode.NORMAL))
sig = trx.build_stream(blocks, mod=ModType.QPSK, spd=CCSpeed.R12)
rx_sig = simulate_channel(sig, time_shift=700, freq_shift_hz=-35.0,
                          sample_rate=12000, snr_db=-2, rng=rng)
try:
    got, sst = trx.demod_stream(rx_sig, n_blocks=len(blocks))
    ok = got == blocks
    per_frame = sum(len(trx.build_frame(b, mod=ModType.QPSK, spd=CCSpeed.R12))
                    for b in blocks) / 12000
    print(f"[{'PASS' if ok else 'FAIL'}] streamed burst @ -2 dB: "
          f"{sst.ok_count}/{len(blocks)} blocks, {len(sig)/12000:.2f} s vs "
          f"{per_frame:.2f} s per-frame ({per_frame*12000/len(sig):.2f}x)")
    print(f"        start={sst.start_sample} cfo={sst.cfo_hz:+.2f}Hz "
          f"SNR={sst.snr_db:+.2f}dB resyncs={len(sst.resyncs)}")
    results.append(ok)
except Exception as exc:
    print(f"[FAIL] streamed burst: {type(exc).__name__}: {exc}")
    results.append(False)

# broadcast (non-ARQ): nothing is retransmitted, so the measures are
# what fraction arrives and whether a late listener can join at all
from ofdm_phy.broadcast import BroadcastTx, BroadcastRx, PT_TELEMETRY

bc_payload = bytes((i * 37 + 11) & 0xFF for i in range(300))
btx = BroadcastTx(LinkMode.NORMAL, ModType.QPSK, CCSpeed.R12, group=4)
bgroups = btx.build(bc_payload, ptype=PT_TELEMETRY)
bair = np.concatenate(bgroups).astype(float)
try:
    rx_bc = simulate_channel(bair, time_shift=400, freq_shift_hz=20.0,
                             sample_rate=12000, snr_db=6, rng=rng)
    got, bst = BroadcastRx(LinkMode.NORMAL, group=4).receive(rx_bc)
    ok = got == bc_payload and bst.saw_eos and bst.ptype == PT_TELEMETRY
    print(f"[{'PASS' if ok else 'FAIL'}] broadcast @ +6 dB: {bst.report()}")
    results.append(ok)
except Exception as exc:
    print(f"[FAIL] broadcast: {type(exc).__name__}: {exc}")
    results.append(False)

try:  # a receiver that tunes in half-way through group 1
    cut = len(bgroups[0]) // 2
    late = simulate_channel(bair[cut:], time_shift=400, freq_shift_hz=20.0,
                            sample_rate=12000, snr_db=6, rng=rng)
    lgot, lst = BroadcastRx(LinkMode.NORMAL, group=4).receive(late)
    # it cannot recover group 1 -- nothing is retransmitted -- but it
    # must acquire on a later group rather than hearing nothing at all
    ok = lst.groups >= len(bgroups) - 2 and bc_payload.endswith(lgot[-40:])
    print(f"[{'PASS' if ok else 'FAIL'}] broadcast late join: "
          f"{lst.report()}, {len(lgot)}/{len(bc_payload)} B")
    results.append(ok)
except Exception as exc:
    print(f"[FAIL] broadcast late join: {type(exc).__name__}: {exc}")
    results.append(False)

print(f"\n{sum(results)}/{len(results)} cases passed")
sys.exit(0 if all(results) else 1)
