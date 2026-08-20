#!/usr/bin/env python3
"""Hardware bring-up for the SDR driver: enumerate devices, exercise the
receive path, and (only when asked) the transmit path.

The DSP is already covered by `sdr_driver.py --selftest`; what needs real
hardware is the device plumbing -- stream setup, achieved sample rate,
gain settings, TX/RX switching -- plus the analogue realities (DC offset,
clipping, noise floor). This tool reports exactly those.

  --list            devices SoapySDR can see
  --rx              receive bring-up: stream, levels, spectrum, and the
                    driver's own SSB demodulator run on real samples
  --tx              transmit bring-up: send a test frame. Requires
                    --i-have-a-dummy-load, because a HackRF's output is
                    unfiltered and radiating it needs a licence and a
                    low-pass filter.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sdr_driver import AUDIO_FS, SSBDemodulator, SSBModulator, TX_PEAK  # noqa


def _soapy():
    try:
        import SoapySDR
        return SoapySDR
    except ImportError:
        raise SystemExit(
            "SoapySDR python bindings not found:\n"
            "  sudo apt install python3-soapysdr\n"
            "(a venv without --system-site-packages needs the module "
            "symlinked in; see demoapp/README.md)")


def cmd_list():
    S = _soapy()
    devs = S.Device.enumerate()
    if not devs:
        print("no SDR devices found")
        return 1
    for i, d in enumerate(devs):
        info = {k: d[k] for k in d.keys()}
        label = info.get("label", info.get("driver", "?"))
        print(f"[{i}] {label}")
        for k in sorted(info):
            print(f"      {k} = {info[k]}")
        drv = info.get("driver")
        if drv:
            hint = f"driver={drv}"
            if info.get("serial"):
                hint += f",serial={info['serial']}"
            print(f"      --device \"{hint}\"")
    return 0


def _open(args, direction):
    S = _soapy()
    dev = S.Device(dict(kv.split("=", 1)
                        for kv in args.device.split(",") if kv))
    d = S.SOAPY_SDR_RX if direction == "rx" else S.SOAPY_SDR_TX
    dev.setSampleRate(d, 0, float(args.rate))
    dev.setFrequency(d, 0, float(args.freq))
    gain = args.rx_gain if direction == "rx" else args.tx_gain
    names = list(dev.listGains(d, 0))
    named = {k: v for k, v in (("LNA", args.lna), ("VGA", args.vga),
                               ("AMP", args.amp)) if k in names
             and v is not None}
    if named:
        for k, v in named.items():
            dev.setGain(d, 0, k, float(v))
        gain = ", ".join(f"{k} {v:g}" for k, v in named.items())
    else:
        dev.setGain(d, 0, float(gain))
    if args.antenna:
        dev.setAntenna(d, 0, args.antenna)
    got_rate = dev.getSampleRate(d, 0)
    got_freq = dev.getFrequency(d, 0)
    print(f"  requested {args.rate/1e6:.3f} Msps at "
          f"{args.freq/1e6:.4f} MHz, gain {gain} dB")
    print(f"  device reports {got_rate/1e6:.6f} Msps at "
          f"{got_freq/1e6:.6f} MHz, antenna {dev.getAntenna(d, 0)}")
    if abs(got_rate - args.rate) > 1.0:
        print(f"  !! rate mismatch: the driver resamples by an integer "
              f"ratio, so use a rate the device can hit exactly")
    return S, dev, d


def cmd_rx(args):
    S, dev, d = _open(args, "rx")
    stream = dev.setupStream(d, S.SOAPY_SDR_CF32)
    dev.activateStream(stream)
    mtu = dev.getStreamMTU(stream)
    want = int(args.seconds * args.rate)
    print(f"  stream MTU {mtu} samples; capturing {args.seconds:.1f} s "
          f"({want} samples)")

    buf = np.empty(mtu, dtype=np.complex64)
    caught, drops, t0 = [], 0, time.time()
    got_n = 0
    while got_n < want and time.time() - t0 < args.seconds * 5 + 5:
        sr = dev.readStream(stream, [buf], mtu, timeoutUs=int(1e6))
        if sr.ret > 0:
            caught.append(buf[:sr.ret].copy())
            got_n += sr.ret
        else:
            drops += 1
            if drops > 50:
                break
    dev.deactivateStream(stream)
    dev.closeStream(stream)
    if not caught:
        print("  !! no samples received")
        return 1
    iq = np.concatenate(caught)
    wall = time.time() - t0
    print(f"  received {len(iq)} samples in {wall:.2f} s "
          f"({len(iq)/wall/1e6:.3f} Msps effective), {drops} timeouts")

    # --- analogue health
    i, q = iq.real, iq.imag
    print(f"  I/Q  rms {np.sqrt(np.mean(np.abs(iq)**2)):.4f}  "
          f"peak {np.max(np.abs(iq)):.4f}  "
          f"DC offset {np.mean(i):+.5f}{np.mean(q):+.5f}j")
    clip = np.mean((np.abs(i) > 0.98) | (np.abs(q) > 0.98))
    print(f"  clipping {clip*100:.3f} % of samples "
          f"({'reduce --rx-gain' if clip > 1e-3 else 'ok'})")
    imb = 20 * np.log10((np.std(i) + 1e-12) / (np.std(q) + 1e-12))
    print(f"  I/Q amplitude imbalance {imb:+.2f} dB")

    # --- spectrum
    n = 1 << 16
    seg = iq[:n] * np.hanning(n)
    P = np.abs(np.fft.fftshift(np.fft.fft(seg)))**2
    f = np.fft.fftshift(np.fft.fftfreq(n, 1 / args.rate))
    k = int(np.argmax(P))
    print(f"  strongest tone {f[k]/1e3:+.1f} kHz from centre, "
          f"{10*np.log10(P[k]/np.median(P)):.1f} dB over the median floor")

    # --- the driver's own receive chain on real samples
    dem = SSBDemodulator(int(args.rate), args.if_offset)
    step = int(args.rate) // AUDIO_FS * 120
    audio = np.concatenate([dem(iq[a:a + step])
                            for a in range(0, len(iq) - step, step)])
    audio = audio * (32768.0 / TX_PEAK) * 127.0
    print(f"  SSB audio out: {len(audio)} samples "
          f"({len(audio)/AUDIO_FS:.2f} s), rms {np.sqrt(np.mean(audio**2)):.0f}"
          f", peak {np.max(np.abs(audio)):.0f} (int16 scale)")
    if args.save:
        np.clip(audio, -32768, 32767).astype("<i2").tofile(args.save)
        print(f"  wrote {args.save} (raw int16 @ 12 kHz -- feed it to the "
              f"receiver or play it)")
    print("\nRX bring-up complete.")
    return 0


def cmd_tx(args):
    if not args.i_have_a_dummy_load:
        raise SystemExit(
            "Refusing to transmit without --i-have-a-dummy-load.\n"
            "A HackRF's output is unfiltered: radiating it needs a licence "
            "and at least a low-pass filter for the harmonics. Connect a "
            "dummy load (or an attenuator into a second receiver) first.")
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from ofdm_phy import Transceiver, Data, ModType, CCSpeed

    S, dev, d = _open(args, "tx")
    if args.tx_mode == "extreme":
        # one EXTREME frame is ~38 s of *continuous* OFDM -- no frame
        # boundaries, so a spectrum analyser sees the real occupied
        # bandwidth rather than the splatter of a pulsed signal
        from ofdm_phy import LinkMode
        from ofdm_phy.modes import make_modem
        trx = Transceiver(make_modem(LinkMode.EXTREME))
    else:
        trx = Transceiver()
    frame = trx.build_frame(
        Data(reserved=123, payload=b"HACKRF TX BRINGUP"),
        mod=ModType.QPSK, spd=CCSpeed.R12)
    if args.repeat > 1:
        frame = np.tile(frame, args.repeat)
    audio = np.clip(frame / np.max(np.abs(frame)) * 32767, -32768, 32767)
    print(f"  test frame: {len(audio)} samples ({len(audio)/AUDIO_FS:.2f} s "
          f"of audio)")

    mod = SSBModulator(int(args.rate), args.if_offset)
    ratio = int(args.rate) // AUDIO_FS
    air = len(audio) / AUDIO_FS
    print(f"  streaming {air:.1f} s of air time at "
          f"{args.rate/1e6:.3f} Msps (modulated on the fly)")

    stream = dev.setupStream(d, S.SOAPY_SDR_CF32)
    dev.activateStream(stream)
    step = 1200                      # 0.1 s of audio per block
    sent, t0 = 0, time.time()
    peak = 0.0
    for a in range(0, len(audio), step):
        iq = mod(audio[a:a + step]) * (TX_PEAK / 32768.0) / 127.0
        peak = max(peak, float(np.max(np.abs(iq))))
        iq = iq.astype(np.complex64)
        off = 0
        while off < len(iq):
            sr = dev.writeStream(stream, [iq[off:off + 16384]],
                                 min(16384, len(iq) - off),
                                 timeoutUs=int(2e6))
            if sr.ret <= 0:
                print(f"  !! writeStream returned {sr.ret}")
                off = len(iq)
                break
            off += sr.ret
            sent += sr.ret
    handed = time.time() - t0
    remain = air - (time.time() - t0)
    if remain > 0:
        time.sleep(remain)
    time.sleep(0.05)
    dev.deactivateStream(stream)
    dev.closeStream(stream)
    print(f"  sent {sent} samples ({sent/args.rate:.1f} s), peak "
          f"{peak:.3f} of full scale, host kept up: "
          f"{'yes' if handed <= air + 0.5 else 'NO -- underruns likely'}")
    print(f"  total {time.time()-t0:.1f} s against {air:.1f} s of air time")

    print("\nTX bring-up complete.")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--rx", action="store_true")
    ap.add_argument("--tx", action="store_true")
    ap.add_argument("--device", default="driver=hackrf")
    ap.add_argument("--rate", type=float, default=2.4e6)
    ap.add_argument("--freq", type=float, default=7.05e6)
    ap.add_argument("--if-offset", type=float, default=0.0)
    ap.add_argument("--rx-gain", type=float, default=32.0)
    ap.add_argument("--tx-gain", type=float, default=0.0)
    ap.add_argument("--lna", type=float, default=None)
    ap.add_argument("--vga", type=float, default=None)
    ap.add_argument("--amp", type=float, default=None)
    ap.add_argument("--antenna", default=None)
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--save", default=None,
                    help="write the demodulated audio as raw int16 12 kHz")
    ap.add_argument("--i-have-a-dummy-load", action="store_true",
                    help="required to transmit; a 50 ohm load, dummy load "
                         "or instrument input -- not an antenna")
    ap.add_argument("--tx-mode", choices=("normal", "extreme"),
                    default="normal",
                    help="extreme = one ~38 s continuous frame, the right "
                         "signal for a spectrum measurement")
    ap.add_argument("--repeat", type=int, default=1)
    args = ap.parse_args()

    if args.list:
        return cmd_list()
    if args.rx:
        return cmd_rx(args)
    if args.tx:
        return cmd_tx(args)
    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
