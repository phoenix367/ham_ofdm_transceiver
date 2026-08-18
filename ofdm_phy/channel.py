"""Air-channel simulator: AWGN + multipath + dynamic CFO drift + BSC/BEC fading
(block bit-flips and deep-fade erasures), as described in the article.

Article defaults: SNRdb=-6, channelResponse=[1.0, 0, 0.4, 0, 0, 0.2],
bit_flip_prob=0.001, erasure_prob=0.02.
"""

import numpy as np
from scipy.signal import butter, hilbert, lfilter

DEFAULT_CHANNEL_RESPONSE = np.array([1.0, 0, 0.4, 0, 0, 0.2])


def rayleigh_fading(n: int, sample_rate: int, doppler_hz: float,
                    rng: np.random.Generator):
    """Flat Rayleigh fading gain: complex Gaussian process band-limited to
    the Doppler bandwidth, normalized to unit mean power. At HF-typical
    Doppler spreads (0.1-0.5 Hz) this produces the slow QSB with occasional
    deep fades that the adaptation layer has to survive."""
    warmup = int(4 * sample_rate / doppler_hz)
    w = (rng.standard_normal(n + warmup) + 1j * rng.standard_normal(n + warmup))
    b, a = butter(2, doppler_hz / (sample_rate / 2))
    g = lfilter(b, a, w)[warmup:]
    g /= np.sqrt(np.mean(np.abs(g) ** 2))
    return g


def simulate_channel(
        signal, time_shift: int, freq_shift_hz: float, sample_rate: int,
        frame_size: int = 160,
        snr_db: float = -6.0,
        channel_response=DEFAULT_CHANNEL_RESPONSE,
        bit_flip_prob: float = 0.001,
        erasure_prob: float = 0.02,
        fading_doppler_hz: float = 0.0,
        rng: np.random.Generator = None,
):
    if rng is None:
        rng = np.random.default_rng()

    rx_time = np.zeros(len(signal) + time_shift, dtype=float)
    rx_time[time_shift:] = signal

    analytic_signal = hilbert(rx_time)

    # dynamic CFO: constant offset + quadratic drift up to +5 Hz
    t = np.arange(len(analytic_signal)) / sample_rate

    dynamic_cfo = freq_shift_hz + 5.0 * (t / t[-1]) ** 2
    phase_drift = 2 * np.pi * np.cumsum(dynamic_cfo) / sample_rate
    cfo_vector = np.exp(1j * phase_drift)
    shifted_complex = analytic_signal * cfo_vector

    # slow Rayleigh fading (QSB): time-varying complex gain on the analytic
    # signal; the AWGN below is sized from the *average* faded power, so the
    # instantaneous SNR swings around the nominal value
    if fading_doppler_hz > 0.0:
        shifted_complex = shifted_complex * rayleigh_fading(
            len(shifted_complex), sample_rate, fading_doppler_hz, rng)

    shifted_real = shifted_complex.real

    # multipath + AWGN
    convolved = np.convolve(shifted_real, channel_response, mode='full')

    signal_power = np.mean(convolved ** 2)
    if signal_power > 0:
        sigma2 = signal_power * 10 ** (-snr_db / 10)
        noise = np.sqrt(sigma2) * rng.standard_normal(convolved.shape)
        result = convolved + noise
    else:
        result = rng.standard_normal(convolved.shape) * 10 ** (-snr_db / 20)

    num_samples = len(result)
    num_frames = int(np.ceil(num_samples / frame_size))

    # BSC: block phase inversion (multiply frame by -1)
    if bit_flip_prob > 0.0 and frame_size > 0:
        flip_decisions = rng.choice([-1, 1], size=num_frames, p=[bit_flip_prob, 1.0 - bit_flip_prob])
        flip_mask = np.repeat(flip_decisions, frame_size)[:num_samples]
        result = result * flip_mask

    # BEC: deep fade (frame silenced)
    if erasure_prob > 0.0 and frame_size > 0:
        erasure_decisions = rng.choice([0, 1], size=num_frames, p=[erasure_prob, 1.0 - erasure_prob])
        erasure_mask = np.repeat(erasure_decisions, frame_size)[:num_samples]
        result = result * erasure_mask

    return result
