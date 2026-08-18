"""RF layer: SSB modulator/demodulator and per-station local oscillators.

Models the real transceiver chain the article's protocol rides on:

  audio 12 kHz -> upsample -> SSB (USB) modulate with the TX station's LO ->
  real RF/IF signal -> channel (Rayleigh fading, multipath, AWGN, BSC/BEC) ->
  product detector with the RX station's LO -> LPF -> decimate -> audio

The simulation runs at an IF sample rate (48 kHz, carrier 12 kHz); LO errors
are specified in ppm of the NOMINAL RF CARRIER (e.g., 7.1 MHz), so the audio
frequency offset between stations emerges physically:

  CFO = f_rf * (ppm_tx - ppm_rx) * 1e-6 + (drift_tx(t) - drift_rx(t))

A transceiver's TX and RX share one reference oscillator, so a station has a
single LO error used for both directions -- as in real rigs.

SNR convention matches the audio-domain simulator: `snr_db` is signal power
vs noise power in the 6 kHz audio bandwidth, so the measured sensitivities
of the modes carry over unchanged.
"""

from dataclasses import dataclass

import numpy as np
from scipy.signal import fftconvolve, firwin, hilbert, resample_poly

from .channel import DEFAULT_CHANNEL_RESPONSE, rayleigh_fading

AUDIO_FS = 12000
RF_FS = 48000
UP = RF_FS // AUDIO_FS
IF_CARRIER = 12000.0
AUDIO_BW = 6000.0

_LPF = firwin(129, AUDIO_BW / (RF_FS / 2))


@dataclass
class StationRF:
    """One station's RF front end: a single reference oscillator (shared by
    TX and RX, as in a transceiver) with a static ppm error and slow thermal
    drift."""
    name: str
    ppm: float                 # reference error, ppm of the RF carrier
    drift_hz_per_s: float = 0.0
    rf_carrier_hz: float = 7.1e6
    trim_hz: float = 0.0       # AFC/netting correction (applied to TX and RX
                               # alike -- one shared reference per rig)

    def lo_offset_hz(self, t: float) -> float:
        """Deviation of this station's carrier from nominal, in Hz, at t."""
        return self.rf_carrier_hz * self.ppm * 1e-6 + self.drift_hz_per_s * t \
            + self.trim_hz


def ssb_modulate(audio, lo_offset_hz: float, t0: float = 0.0):
    """Audio -> real USB RF/IF signal, carrier at IF_CARRIER + lo_offset."""
    up = resample_poly(np.asarray(audio, dtype=np.float64), UP, 1)
    analytic = hilbert(up)
    n = np.arange(len(up))
    f = IF_CARRIER + lo_offset_hz
    carrier = np.exp(2j * np.pi * (f * (t0 + n / RF_FS)))
    return np.real(analytic * carrier)


def ssb_demodulate(rf, lo_offset_hz: float, t0: float = 0.0):
    """Real RF/IF -> audio via product detector with the RX station's LO."""
    n = np.arange(len(rf))
    f = IF_CARRIER + lo_offset_hz
    mixed = np.asarray(rf, dtype=np.float64) * \
        np.exp(-2j * np.pi * (f * (t0 + n / RF_FS)))
    baseband = fftconvolve(mixed, _LPF, mode="same")
    audio_rf_rate = 2.0 * np.real(baseband)
    return resample_poly(audio_rf_rate, 1, UP)


def rf_link(audio, tx: StationRF, rx: StationRF, t0: float,
            snr_db: float,
            time_shift_s: float = 0.05,
            channel_response=DEFAULT_CHANNEL_RESPONSE,
            fading_doppler_hz: float = 0.0,
            bit_flip_prob: float = 0.0,
            erasure_prob: float = 0.0,
            rng: np.random.Generator = None):
    """Full path: TX audio -> SSB up with tx's LO -> RF channel -> product
    detector with rx's LO -> RX audio. t0 is session time (drives LO drift).
    """
    if rng is None:
        rng = np.random.default_rng()

    rf = ssb_modulate(audio, tx.lo_offset_hz(t0), t0)

    # propagation delay / unknown timing
    pad = int(time_shift_s * RF_FS)
    rf = np.concatenate([np.zeros(pad), rf])

    # Rayleigh fading on the RF analytic signal
    if fading_doppler_hz > 0.0:
        g = rayleigh_fading(len(rf), RF_FS, fading_doppler_hz, rng)
        rf = np.real(hilbert(rf) * g)

    # multipath: the audio-domain tap delays are physical, so they map to
    # UP-spaced taps at the RF rate
    taps_rf = np.zeros((len(channel_response) - 1) * UP + 1)
    taps_rf[::UP] = channel_response
    rf = fftconvolve(rf, taps_rf, mode="full")

    # AWGN sized so the post-demod audio-band SNR equals snr_db. Derivation:
    # signal audio power = P_rf (the demod chain is transparent in-band);
    # white RF noise of variance v has two-sided PSD v/RF_FS, the product
    # detector's +-AUDIO_BW LPF passes 2*AUDIO_BW of it as complex noise
    # (v/4 for RF_FS = 8*AUDIO_BW), and 2*Re() doubles power to v/2 -- the
    # Re() halves UNCORRELATED noise relative to the coherent signal, hence
    # the factor 2 rather than (RF_FS/2)/AUDIO_BW.
    p_sig = np.mean(rf ** 2)
    if p_sig > 0:
        var = 2.0 * p_sig * 10 ** (-snr_db / 10)
        rf = rf + np.sqrt(var) * rng.standard_normal(len(rf))

    # block impairments (BSC phase flips / BEC deep erasures) at the RF rate
    frame = 160 * UP
    num_frames = int(np.ceil(len(rf) / frame))
    if bit_flip_prob > 0.0:
        flips = rng.choice([-1, 1], size=num_frames, p=[bit_flip_prob, 1 - bit_flip_prob])
        rf = rf * np.repeat(flips, frame)[:len(rf)]
    if erasure_prob > 0.0:
        erasures = rng.choice([0, 1], size=num_frames, p=[erasure_prob, 1 - erasure_prob])
        rf = rf * np.repeat(erasures, frame)[:len(rf)]

    return ssb_demodulate(rf, rx.lo_offset_hz(t0), t0)


def expected_cfo_hz(tx: StationRF, rx: StationRF, t: float) -> float:
    """The audio-band CFO the RX modem should observe."""
    return tx.lo_offset_hz(t) - rx.lo_offset_hz(t)
