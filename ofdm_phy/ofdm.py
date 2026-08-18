"""OFDM modem core.

Parameters (article defaults): 12 kHz sample rate, 128-bin FFT (93.75 Hz
spacing), 300-2400 Hz band -> 23 subcarriers (bins 3..25), 7 Zadoff-Chu pilots
(bins 3, 6, 10, 14, 17, 21, 25), 16 data carriers, 25% cyclic prefix, 4x
time-domain symbol tiling (~6 dB coherent-accumulation gain).

Class hierarchy mirrors the article:
  OFDMModem       - symbol mod/demod, pilots, channel estimation, ZC preamble
  TiledOFDMModem  - 4x symbol tiling with polynomial phase-drift correction
  FullOFDMModem   - adds the Newman tone preamble and two-stage detection
"""

import typing

import numpy as np
import numpy.typing as npt
from scipy.signal import fftconvolve

from .mapping import PSKMapper, BPSKMapper

DEFAULT_SAMPLE_RATE = 12000


def freq_shift(sample_rate: int, sig: npt.NDArray[np.complex64], shift_hz: float) -> npt.NDArray[np.complex64]:
    """Shift the signal spectrum DOWN by shift_hz (compensates a +shift_hz CFO)."""
    t = np.arange(len(sig))
    return sig * np.exp(-2j * np.pi * shift_hz * t / sample_rate)


class OFDMModem:
    def __init__(
            self,
            sample_rate=DEFAULT_SAMPLE_RATE,
            fft_bins=128,
            freq_lo=300,
            freq_hi=2400,
            pilots_count=7,
            zc_pilots_root=3,
            zc_preamble_root=17,
            zc_preamble_len=23,
            zc_preamble_count=4,
            mapper: typing.Type[PSKMapper] = BPSKMapper,
            max_cfo=4,
            detection_threshold=0.2,
            wiener_corr_len=8.0,
            zc_coherent_group=1,
    ):
        self._sample_rate = sample_rate
        self._fft_bins = fft_bins
        self._freq_step = self._sample_rate / self._fft_bins
        self._freq_lo = freq_lo
        self._freq_hi = freq_hi
        self._bin_lo = int(self._freq_lo / self._freq_step)
        self._bin_hi = int(self._freq_hi / self._freq_step)
        self._bin_center = (self._bin_lo + self._bin_hi) // 2

        self._channel_indices = np.arange(self._bin_lo, self._bin_hi + 1)
        self._subcarriers = len(self._channel_indices)

        self._pilots_count = pilots_count

        self._pilot_local_indices = np.linspace(0, len(self._channel_indices) - 1, self._pilots_count, dtype=int)
        self._data_local_indices = np.delete(np.arange(len(self._channel_indices)), self._pilot_local_indices)

        self._pilot_carriers = self._channel_indices[self._pilot_local_indices]
        self._pilot_carriers_len = len(self._pilot_carriers)

        self._data_carriers = self._channel_indices[self._data_local_indices]
        self._data_carriers_len = len(self._data_carriers)

        self._zc_pilots_root = zc_pilots_root
        self._zc_preamble_root = zc_preamble_root
        self._zc_preamble_len = zc_preamble_len
        self._zc_preamble_count = zc_preamble_count

        self._cyclic_prefix = self._fft_bins // 4  # 25% of the base symbol

        self._mapper = mapper
        self._max_cfo = max_cfo  # in subcarrier bins
        self._detection_threshold = detection_threshold
        self._wiener_corr_len = wiener_corr_len
        # ZC symbols correlated coherently per matched-filter kernel; the
        # remaining zc_preamble_count/group repetitions are summed by
        # magnitude. >1 needs the CFO residual already down to a few Hz.
        self._zc_coherent_group = zc_coherent_group

    # --- properties --------------------------------------------------------

    @property
    def sample_rate(self):
        return self._sample_rate

    @property
    def fft_bins(self):
        return self._fft_bins

    @property
    def cyclic_prefix(self):
        return self._cyclic_prefix

    @property
    def symbol_len(self):
        return self._cyclic_prefix + self._fft_bins

    @property
    def data_carriers_len(self):
        return self._data_carriers_len

    @property
    def pilot_carriers(self):
        return self._pilot_carriers

    @property
    def channel_indices(self):
        return self._channel_indices

    def set_mapper(self, mapper: typing.Type[PSKMapper]):
        self._mapper = mapper

    @property
    def bits_per_symbol(self):
        return self._data_carriers_len * self._mapper.MU

    # --- DFT helpers -------------------------------------------------------

    def _idft(self, symbol: npt.NDArray[np.complex64]) -> npt.NDArray[np.complex64]:
        return np.fft.ifft(symbol).astype(np.complex64)

    def _dft(self, signal: npt.NDArray[np.complex64]) -> npt.NDArray[np.complex64]:
        return np.fft.fft(signal)

    # --- Zadoff-Chu --------------------------------------------------------

    @staticmethod
    def _gen_zc_seq(root: int, length: int) -> npt.NDArray[np.complex64]:
        n = np.arange(length)
        seq = np.exp(-1j * np.pi * root * n * (n + length % 2) / length)
        return seq

    def _get_pilot_values(self) -> npt.NDArray[np.complex64]:
        return self._gen_zc_seq(self._zc_pilots_root, self._pilots_count)

    # --- ZC preamble generation --------------------------------------------

    def _gen_preamble_symbol(self):
        symbol_freq = np.zeros(self._fft_bins, dtype=np.complex64)

        # x2 gain equalizes the real-signal power of the ZC block (23 complex
        # bins, no Hermitian mirror) with the data symbols (23 + 23 bins)
        zc_seq = 2.0 * self._gen_zc_seq(self._zc_preamble_root, self._zc_preamble_len)

        start_bin = self._bin_center - (self._zc_preamble_len // 2)
        symbol_freq[start_bin: start_bin + self._zc_preamble_len] = zc_seq

        symbol = self._idft(symbol_freq)
        return symbol

    def gen_zc_preamble(self) -> npt.NDArray[np.complex64]:
        preamble_zc = self._gen_preamble_symbol()
        preamble_base = np.tile(preamble_zc, self._zc_preamble_count)
        preamble_block = self._add_cp(preamble_base)

        return preamble_block

    def gen_preamble(self) -> typing.Iterator[npt.NDArray[np.complex64]]:
        yield self.gen_zc_preamble()

    # --- ZC preamble detection ---------------------------------------------

    def detect_preamble(self, signal: npt.NDArray[np.complex64]) -> typing.Optional[typing.Tuple[int, float]]:
        return self.detect_zc_preamble(signal)

    def detect_zc_preamble(self, signal: npt.NDArray[np.complex64],
                           max_cfo: typing.Optional[int] = None,
                           cfo_hypotheses_hz: typing.Optional[np.ndarray] = None,
                           ) -> typing.Optional[typing.Tuple[int, float]]:
        N = self._fft_bins
        L = self._zc_preamble_count
        CP = self._cyclic_prefix
        preamble_len = L * N

        if len(signal) < preamble_len + CP:
            return None

        subcarrier_spacing = self._sample_rate / N
        if cfo_hypotheses_hz is not None:
            hypotheses = np.asarray(cfo_hypotheses_hz, dtype=float)
        else:
            if max_cfo is None:
                max_cfo = self._max_cfo
            hypotheses = np.arange(-max_cfo, max_cfo + 1) * subcarrier_spacing

        global_metric_max = -1.0
        best_m = 0
        best_start_zc_coarse = 0

        preamble_block = self.gen_zc_preamble()

        # kernel = G consecutive ZC symbols correlated coherently; the
        # remaining L/G repetitions are combined by magnitude
        G = min(self._zc_coherent_group, L)
        num_groups = L // G
        kernel_len = G * N
        zc_base = preamble_block[-kernel_len:]
        ref_energy = float(np.sum(np.abs(zc_base) ** 2))

        sig_energy_single = np.abs(signal) ** 2
        energy_cumsum = np.cumsum(np.insert(sig_energy_single, 0, 0.0))

        t = np.arange(kernel_len)
        for f_hyp in hypotheses:
            cfo_shift_vector = np.exp(2j * np.pi * f_hyp * t / self._sample_rate)
            modulated_zc = zc_base * cfo_shift_vector

            one_zc_kernel = np.conj(modulated_zc[::-1])
            corr_single = np.abs(fftconvolve(signal, one_zc_kernel, mode="valid"))

            coarse_corr = np.zeros(len(corr_single) - (num_groups - 1) * kernel_len)
            for i in range(num_groups):
                shift = i * kernel_len
                coarse_corr += corr_single[shift: shift + len(coarse_corr)]

            epsilon = 1e-12

            # normalized cross-correlation coefficient for every candidate
            # position (1.0 for a perfect noise-free match); picking the best
            # NORMALIZED metric (instead of the raw correlation peak) keeps
            # high-energy data symbols from masking the true preamble
            n_pos = min(len(coarse_corr), len(signal) - preamble_len + 1)
            if n_pos <= 0:
                continue
            window_energy = energy_cumsum[preamble_len:preamble_len + n_pos] - energy_cumsum[:n_pos]
            metrics = coarse_corr[:n_pos] / (
                    num_groups * np.sqrt(window_energy / num_groups * ref_energy) + epsilon)

            local_max_idx = int(np.argmax(metrics))
            local_metric = metrics[local_max_idx]
            local_max_val = coarse_corr[local_max_idx]

            # dynamic threshold expressed in multiples of the base threshold
            # (identical to the fixed rule at the default base of 0.2, but it
            # scales down with the base for the low-SNR modes)
            mean_corr_floor = np.mean(coarse_corr)
            peak_to_floor_ratio = local_max_val / (mean_corr_floor + epsilon)
            base_threshold = self._detection_threshold
            if peak_to_floor_ratio < 3.5:
                dynamic_threshold = base_threshold * (1.0 + 0.5 * (3.5 - peak_to_floor_ratio))
                dynamic_threshold = min(dynamic_threshold, 2.25 * base_threshold)
            else:
                dynamic_threshold = base_threshold

            if local_metric > global_metric_max and local_metric >= dynamic_threshold:
                global_metric_max = local_metric
                best_start_zc_coarse = local_max_idx
                best_m = f_hyp

        if global_metric_max <= 0.0:
            return None

        start_zc_coarse = best_start_zc_coarse
        true_start_zc = start_zc_coarse
        est_time = true_start_zc - CP

        if true_start_zc + preamble_len > len(signal) or true_start_zc < 0:
            return None

        preamble_signal = signal[true_start_zc: true_start_zc + preamble_len]

        t_full = np.arange(preamble_len)
        de_rotated_preamble = preamble_signal * np.exp(-2j * np.pi * best_m * t_full / self._sample_rate)

        sig_current = de_rotated_preamble[: (L - 1) * N]
        sig_delayed = de_rotated_preamble[N: preamble_len]

        r_vector = np.sum(np.conj(sig_current) * sig_delayed)
        phase_diff = np.angle(r_vector)

        fractional_cfo = phase_diff / (2 * np.pi * (N / self._sample_rate))

        final_cfo = best_m + fractional_cfo
        final_time = est_time + len(preamble_block)

        return final_time, final_cfo

    # --- modulation --------------------------------------------------------

    def _serial_to_parallel(self, bits: npt.NDArray):
        return bits.reshape((self.data_carriers_len, self._mapper.MU))

    def _map_subsymbols_bits(self, bits: npt.NDArray) -> npt.NDArray[np.complex64]:
        bits_sp = self._serial_to_parallel(bits)
        subsymbols = self._mapper.map(bits_sp)
        return subsymbols

    def gen_symbol(self, bits: npt.NDArray) -> npt.NDArray[np.complex64]:
        subsymbols = np.zeros(self._subcarriers, dtype=np.complex64)
        subsymbols[self._data_local_indices] = self._map_subsymbols_bits(bits)
        subsymbols[self._pilot_local_indices] = self._get_pilot_values()

        symbol = np.zeros(self._fft_bins, dtype=np.complex64)
        symbol[self._channel_indices] = subsymbols

        mirror_bins = self._fft_bins - np.array(self._channel_indices)
        symbol[mirror_bins] = np.conj(subsymbols)

        return symbol

    def _add_cp(self, signal: npt.NDArray[np.complex64]) -> npt.NDArray[np.complex64]:
        cp = signal[-self.cyclic_prefix:]
        return np.hstack([cp, signal])

    def _remove_cp(self, signal: npt.NDArray[np.complex64]) -> npt.NDArray[np.complex64]:
        return signal[self.cyclic_prefix:]

    def _modulate_signal(self, symbol: npt.NDArray[np.complex64]) -> npt.NDArray[np.complex64]:
        signal = self._idft(symbol)
        return signal

    def modulate_symbol_cp(self, bits: npt.NDArray) -> npt.NDArray[np.complex64]:
        symbol = self.gen_symbol(bits)
        signal = self._modulate_signal(symbol)
        signal_cp = self._add_cp(signal)
        return signal_cp

    # --- demodulation ------------------------------------------------------

    def _demodulate_signal(self, signal: npt.NDArray[np.complex64]) -> npt.NDArray[np.complex64]:
        demod = self._dft(signal)
        return demod

    def demodulate_symbol_cp(self, symbol: npt.NDArray[np.complex64]) -> npt.NDArray[np.complex64]:
        assert len(symbol) == self.symbol_len

        signal_no_cp = self._remove_cp(symbol)
        demod_all = self._demodulate_signal(signal_no_cp)
        return demod_all

    # --- channel estimation ------------------------------------------------

    def _channel_estimate_zf(self, pilots_ref, pilots, pilot_carriers):
        h_pilots = pilots / pilots_ref
        h_re = np.interp(self._channel_indices, pilot_carriers, h_pilots.real)
        h_im = np.interp(self._channel_indices, pilot_carriers, h_pilots.imag)
        return h_re + 1j * h_im

    def _channel_estimate_wiener(self, pilots_ref, pilots, pilot_carriers, noise_var):
        h_pilots = pilots / pilots_ref

        k_all = self._channel_indices.astype(np.float64)
        k_p = pilot_carriers.astype(np.float64)
        corr_len = self._wiener_corr_len

        r_hp = np.exp(-np.abs(k_all[:, None] - k_p[None, :]) / corr_len)
        r_pp = np.exp(-np.abs(k_p[:, None] - k_p[None, :]) / corr_len)

        pilot_power = max(float(np.mean(np.abs(h_pilots) ** 2)), 1e-9)
        reg = noise_var / pilot_power

        w = r_hp @ np.linalg.inv(r_pp + reg * np.eye(len(k_p)))
        return w @ h_pilots

    # --- soft demodulation (channel est + MMSE equalization + LLR) ---------

    def demodulate_symbol_cp_soft(self, symbol: npt.NDArray[np.complex64], return_symbols: bool = False):
        demod_all = self.demodulate_symbol_cp(symbol)
        demod_channel = demod_all[self._channel_indices]

        pilots_ref = self._get_pilot_values()
        pilots = demod_all[self._pilot_carriers]

        # initial (zero-forcing) channel estimate and equalization
        H_est_init = self._channel_estimate_zf(pilots_ref, pilots, self._pilot_carriers)
        eq_init = demod_channel / (H_est_init + 1e-6)
        data_init = eq_init[self._data_local_indices]

        mu = self._mapper.MU

        if mu == 1:
            ideal_init = np.sign(np.real(data_init))
            ideal_init[ideal_init == 0] = 1.0
        elif mu == 2:
            ideal_init_real = np.sign(np.real(data_init))
            ideal_init_real[ideal_init_real == 0] = 1.0
            ideal_init_imag = np.sign(np.imag(data_init))
            ideal_init_imag[ideal_init_imag == 0] = 1.0
            ideal_init = (ideal_init_real + 1j * ideal_init_imag) / np.sqrt(2.0)
        elif mu == 4:
            a0 = 1.0 / np.sqrt(10.0)
            q_re = np.clip(np.round((np.real(data_init) / a0 + 3.0) / 2.0), 0, 3) * 2.0 - 3.0
            q_im = np.clip(np.round((np.imag(data_init) / a0 + 3.0) / 2.0), 0, 3) * 2.0 - 3.0
            ideal_init = (q_re + 1j * q_im) * a0
        else:
            raise ValueError("unsupported modulation order")

        # initial noise estimate (MSE against the ideal constellation)
        noise_var_eq_init = np.mean(np.abs(data_init - ideal_init) ** 2)
        mean_H_power_init = np.mean(np.abs(H_est_init[self._data_local_indices]) ** 2)
        curr_noise_var = max(noise_var_eq_init * mean_H_power_init, 1e-6)
        curr_noise_var = min(curr_noise_var, 60.0)

        # refined Wiener channel estimate + MMSE equalizer
        H_est = self._channel_estimate_wiener(pilots_ref, pilots, self._pilot_carriers, curr_noise_var)

        equalized_all = (demod_channel * np.conj(H_est)) / (np.abs(H_est) ** 2 + curr_noise_var)
        equalized_data_symbols = equalized_all[self._data_local_indices]
        data_symbols = equalized_data_symbols

        real_parts = np.real(data_symbols)
        imag_parts = np.imag(data_symbols)

        if mu == 1:
            bpsk_dec = np.sign(real_parts)
            bpsk_dec[bpsk_dec == 0] = 1.0

            amplitude = np.mean(real_parts * bpsk_dec)

            noise_var_eq_real = np.mean((real_parts - amplitude * bpsk_dec) ** 2)
            noise_var_eq_imag = np.mean(imag_parts ** 2)
            noise_var_eq = noise_var_eq_real + noise_var_eq_imag
        elif mu == 4:
            # per-symbol gain estimate -> constellation unit a, then MSE
            # against the nearest 16-QAM point
            gain = max(np.sqrt(np.mean(real_parts ** 2 + imag_parts ** 2)), 1e-6)
            a = gain / np.sqrt(10.0)
            q_re = (np.clip(np.round((real_parts / a + 3.0) / 2.0), 0, 3) * 2.0 - 3.0) * a
            q_im = (np.clip(np.round((imag_parts / a + 3.0) / 2.0), 0, 3) * 2.0 - 3.0) * a
            noise_var_eq = np.mean((real_parts - q_re) ** 2 + (imag_parts - q_im) ** 2)
        else:
            qpsk_dec_real = np.sign(real_parts)
            qpsk_dec_real[qpsk_dec_real == 0] = 1.0
            qpsk_dec_imag = np.sign(imag_parts)
            qpsk_dec_imag[qpsk_dec_imag == 0] = 1.0

            amplitude_real = np.mean(real_parts * qpsk_dec_real)
            amplitude_imag = np.mean(imag_parts * qpsk_dec_imag)

            noise_var_eq_real = np.mean((real_parts - amplitude_real * qpsk_dec_real) ** 2)
            noise_var_eq_imag = np.mean((imag_parts - amplitude_imag * qpsk_dec_imag) ** 2)
            noise_var_eq = noise_var_eq_real + noise_var_eq_imag

        H_data = H_est[self._data_local_indices]
        H_power_sq = np.abs(H_data) ** 2
        mean_H_power = np.mean(H_power_sq)

        updated_noise_var = noise_var_eq * mean_H_power

        alpha = 0.1
        noise_var = alpha * updated_noise_var + (1 - alpha) * curr_noise_var
        noise_var = max(noise_var, 1e-6)

        es_n0_linear_per_carrier = H_power_sq / noise_var
        es_n0_linear = mean_H_power / noise_var

        if mu == 1:
            llr_outputs = 4.0 * real_parts * es_n0_linear_per_carrier
        elif mu == 2:
            llr_i = 2.0 * real_parts * es_n0_linear_per_carrier
            llr_q = 2.0 * imag_parts * es_n0_linear_per_carrier

            llr_outputs = np.empty(2 * len(data_symbols), dtype=np.float64)
            llr_outputs[0::2] = llr_i
            llr_outputs[1::2] = llr_q
        else:
            # 16-QAM max-log LLRs, Gray (b0,b1)=I pair, (b2,b3)=Q pair:
            # b0/b2 = axis sign, b1/b3 = inner-vs-outer (|x| vs 2a)
            llr_outputs = np.empty(4 * len(data_symbols), dtype=np.float64)
            e = es_n0_linear_per_carrier
            llr_outputs[0::4] = 2.0 * real_parts * e / gain
            llr_outputs[1::4] = 2.0 * (2.0 * a - np.abs(real_parts)) * e / gain
            llr_outputs[2::4] = 2.0 * imag_parts * e / gain
            llr_outputs[3::4] = 2.0 * (2.0 * a - np.abs(imag_parts)) * e / gain

        llr_outputs = np.clip(llr_outputs, -20.0, 20.0)

        if return_symbols:
            return es_n0_linear_per_carrier, es_n0_linear, llr_outputs, data_init, data_symbols
        return es_n0_linear_per_carrier, es_n0_linear, llr_outputs


class TiledOFDMModem(OFDMModem):
    """Repeats each OFDM symbol sym_tile times in the time domain; the receiver
    coherently accumulates the repeats (with polynomial phase-drift smoothing),
    gaining ~10*log10(sym_tile) dB of SNR."""

    def __init__(self, *args, sym_tile=4, demod_freq_search=None, demod_freq_range=8.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.sym_tile = sym_tile
        # Long symbols (high tile factors) cannot rely on per-tile phase
        # estimates at very low SNR -- the pairwise tile correlations are
        # noise there -- so they use a per-symbol residual-CFO hypothesis
        # search instead, which spends the whole symbol's energy on the
        # decision. Default: polyfit tracker up to 4 tiles, search above.
        self._demod_freq_search = (sym_tile > 4) if demod_freq_search is None else demod_freq_search
        self._demod_freq_range = demod_freq_range
        # gated two-stage search (see _coarse_freq_window); True = default
        self.coarse_freq_search = True

    @property
    def symbol_len(self):
        return self._cyclic_prefix + self._fft_bins * self.sym_tile

    def _modulate_signal(self, symbol: npt.NDArray[np.complex64]) -> npt.NDArray[np.complex64]:
        signal = super()._modulate_signal(symbol)
        return np.tile(signal, self.sym_tile)

    # coarse top1/median contrast gate for the two-stage search: measured on
    # the fixed model (edge frames <=2.2x, 5 dB of margin >=2.7x) -- below
    # the gate the exhaustive grid runs, so the sensitivity edge is lossless
    COARSE_GATE = 2.25

    def _coarse_freq_window(self, signal, freqs, tiles):
        """Coarse pass of the gated two-stage search: accumulate only the
        first tiles/4 tiles (4x wider frequency mainlobe -> 4x coarser grid
        on 1/4 of the samples), rank hypotheses, and -- if the top-1 peak is
        decisive -- return the union of the top-3 +-5-step fine windows.
        Returns None (= run the full grid) when the coarse contrast is below
        the gate; the coarse metric is ~6 dB noisier than the full symbol
        and a hard coarse decision measurably costs ~0.5 dB at the edge."""
        ct = max(4, tiles // 4)
        tile_len = len(signal) // tiles
        seg = signal[:ct * tile_len]
        t = np.arange(len(seg))
        scores = []
        for k in range(0, len(freqs), 4):
            derot = seg * np.exp(-2j * np.pi * freqs[k] * t / self._sample_rate)
            acc = derot.reshape(ct, -1).mean(axis=0)
            spec = self._dft(acc)
            scores.append((float(np.sum(np.abs(spec[self._channel_indices]) ** 2)), k))
        ranked = sorted(scores, reverse=True)
        median = ranked[len(ranked) // 2][0] or 1.0
        if ranked[0][0] / median < self.COARSE_GATE:
            return None
        window = set()
        for _, k in ranked[:3]:
            window.update(range(max(0, k - 5), min(len(freqs), k + 6)))
        return sorted(window)

    def _demodulate_freq_search(self, signal: npt.NDArray[np.complex64]) -> npt.NDArray[np.complex64]:
        """Coherent tile accumulation with a residual-CFO hypothesis search.

        Derotate the whole symbol by each candidate frequency, average the
        tiles, and keep the hypothesis with the most energy in the active
        subcarriers. The frequency step is a quarter cycle over the symbol
        duration, so the winning hypothesis leaves at most ~pi/4 of phase
        ramp across the accumulation window. With coarse_freq_search (the
        default), a gated quarter-length coarse pass shortlists the
        hypotheses first (~5x fewer full-length derotations when the frame
        has margin; the exhaustive grid remains the low-margin fallback).
        """
        tiles = self.sym_tile
        sym_duration = len(signal) / self._sample_rate
        step = 0.25 / sym_duration
        freqs = np.arange(-self._demod_freq_range, self._demod_freq_range + step / 2, step)

        indices = None
        if self.coarse_freq_search and len(freqs) > 9:
            indices = self._coarse_freq_window(signal, freqs, tiles)
        if indices is None:
            indices = range(len(freqs))

        t = np.arange(len(signal))
        best_spec = None
        best_score = -1.0
        for k in indices:
            derot = signal * np.exp(-2j * np.pi * freqs[k] * t / self._sample_rate)
            acc = derot.reshape(tiles, -1).mean(axis=0)
            spec = self._dft(acc)
            score = float(np.sum(np.abs(spec[self._channel_indices]) ** 2))
            if score > best_score:
                best_score = score
                best_spec = spec

        return best_spec

    def _demodulate_signal(self, signal: npt.NDArray[np.complex64]) -> npt.NDArray[np.complex64]:
        if self._demod_freq_search:
            return self._demodulate_freq_search(signal)

        repeats = signal.reshape(self.sym_tile, -1)
        tiles = self.sym_tile

        x_data = [0]
        y_data = [0.0]

        max_step = min(3, tiles - 1)
        phase_diffs = {}

        for step in range(1, max_step + 1):
            for i in range(tiles - step):
                j = i + step
                R_ij = np.sum(repeats[i] * np.conj(repeats[j]))
                phase_diffs[(i, j)] = np.angle(R_ij)

        for target in range(1, tiles):
            if target <= max_step:
                x_data.append(target)
                y_data.append(-phase_diffs[(0, target)])

            chain_phase = 0.0
            for i in range(target):
                chain_phase += phase_diffs[(i, i + 1)]

            x_data.append(target)
            y_data.append(-chain_phase)

            if target >= 2:
                step2_phase = 0.0
                curr = target

                while curr >= 2:
                    step2_phase += phase_diffs[(curr - 2, curr)]
                    curr -= 2

                if curr == 1:
                    step2_phase += phase_diffs[(0, 1)]

                x_data.append(target)
                y_data.append(-step2_phase)

        poly_coeffs = np.polyfit(x_data, y_data, deg=2)

        idx = np.arange(tiles)
        smooth_phases = np.polyval(poly_coeffs, idx)

        smooth_phases -= smooth_phases[0]

        # de-rotate each repeat back onto the first one and accumulate
        phase_corrs = np.exp(-1j * smooth_phases)[:, np.newaxis]
        corrected = repeats * phase_corrs

        acc = np.mean(corrected, axis=0)
        demod = super()._demodulate_signal(acc)
        return demod


class FullOFDMModem(TiledOFDMModem):
    """Adds the Newman tone preamble (two 4-tone combs, AGC settling +
    coarse CFO estimation) in front of the ZC preamble."""

    def __init__(self, *args, newman_preamble_tile=10, newman_threshold=1.6,
                 detect_fft_len=None, **kwargs):
        super().__init__(*args, **kwargs)

        self._newman_preamble_bins = np.arange(self._bin_center - 6, self._bin_center + 6 + 1, 4)
        self._newman_preamble_shifts = [0, 2]
        self._newman_preamble_tile = newman_preamble_tile
        self._newman_threshold = newman_threshold
        # Detection FFT length (a multiple of fft_bins). Longer blocks
        # concentrate the tone energy into narrower bins -- essential for the
        # very-low-SNR modes -- and make the coarse CFO grid proportionally
        # finer. None keeps the article's one-symbol blocks.
        self._detect_fft_len = detect_fft_len

    # --- generation --------------------------------------------------------

    @staticmethod
    def _gen_newman_phases(M: int) -> npt.NDArray[np.float64]:
        m = np.arange(M)
        newman_phases = (np.pi * (m ** 2)) / M
        return newman_phases

    def gen_newman_preamble_tiles(self) -> npt.NDArray[np.complex64]:
        phases = self._gen_newman_phases(M=len(self._newman_preamble_bins))
        # gain equalizes tone-block power (4+4 bins) with data symbols (46)
        gain = np.sqrt(2.0 * self._subcarriers / (2.0 * len(self._newman_preamble_bins)))
        tone_values = gain * np.exp(1j * phases).astype(np.complex64)

        sym_mat = np.zeros((len(self._newman_preamble_shifts), self._fft_bins), dtype=np.complex64)

        for i, shift in enumerate(self._newman_preamble_shifts):
            current_bins = self._newman_preamble_bins + shift

            sym_mat[i, current_bins] = tone_values
            sym_mat[i, self._fft_bins - current_bins] = np.conj(tone_values)

        return sym_mat

    def gen_newman_preamble(self) -> typing.Iterator[npt.NDArray[np.complex64]]:
        sym_mat = self.gen_newman_preamble_tiles()
        for i, symbol in enumerate(sym_mat):
            signal = self._idft(symbol)
            signal_tile = np.tile(signal, self._newman_preamble_tile * (2 if i == 0 else 1))

            yield signal_tile

    def gen_preamble(self) -> typing.Iterator[npt.NDArray[np.complex64]]:
        yield from self.gen_newman_preamble()
        yield from super().gen_preamble()

    # --- detection ---------------------------------------------------------

    def detect_newman_preamble(self, signal: npt.NDArray[np.complex64]) -> typing.Optional[typing.Tuple[int, float]]:
        N = self._fft_bins
        T = self._newman_preamble_tile

        B = self._detect_fft_len or N  # detection block length
        s = B // N                     # fine bins per subcarrier spacing
        assert B % N == 0 and (T * N) % B == 0, "detect_fft_len must divide the tone fields"
        fine_bin_hz = self._sample_rate / B

        K = len(self._newman_preamble_shifts)
        # tone masks over the positive-frequency fine bins; with fine blocks
        # (s > 1) each tone gets a +-1-bin skirt for scalloping leakage
        halfwidth = 0 if s == 1 else 1
        masks = np.zeros((K, B), dtype=np.float32)
        for i, shift in enumerate(self._newman_preamble_shifts):
            for b in (self._newman_preamble_bins + shift) * s:
                masks[i, max(b - halfwidth, 1): min(b + halfwidth, B // 2 - 1) + 1] = 1.0

        cfo_shifts = np.arange(-self._max_cfo * s, self._max_cfo * s + 1)
        num_shifts = len(cfo_shifts)

        shifted_masks_mat = np.zeros((K, num_shifts, B), dtype=np.float32)
        for s_idx, shift in enumerate(cfo_shifts):
            if shift >= 0:
                shifted_masks_mat[:, s_idx, shift:] = masks[:, :B - shift] if shift > 0 else masks
            else:
                shifted_masks_mat[:, s_idx, :shift] = masks[:, -shift:]

        num_blocks = len(signal) // B
        tone0_blocks = 2 * T * N // B
        tone1_blocks = T * N // B
        total_preamble_blocks = tone0_blocks + (K - 1) * tone1_blocks
        if num_blocks < total_preamble_blocks:
            return None

        blocks = signal[:num_blocks * B].reshape(num_blocks, B)
        spectrogram = (np.abs(np.fft.fft(blocks, axis=1)) ** 2).astype(np.float32)

        # accumulated per-tone-symbol energy for every candidate offset b
        # (vectorized equivalent of the article's per-block loop)
        cum = np.vstack([np.zeros((1, B), dtype=np.float32), np.cumsum(spectrogram, axis=0)])
        num_offsets = num_blocks - total_preamble_blocks + 1

        offs = np.arange(num_offsets)
        accum_symbols = np.zeros((num_offsets, K, B), dtype=np.float32)
        block_offset = np.zeros(num_offsets, dtype=int) + offs
        for k in range(K):
            blocks_to_sum = tone0_blocks if k == 0 else tone1_blocks
            accum_symbols[:, k, :] = cum[block_offset + blocks_to_sum] - cum[block_offset]
            block_offset = block_offset + blocks_to_sum

        # score = mean power in the tone bins / mean power in the remaining
        # positive-frequency bins; ~1 for noise or wideband data symbols,
        # >> 1 when the tone comb is present at that offset and CFO shift
        n_mask_bins = masks[0].sum()
        band = np.zeros(B, dtype=np.float32)
        band[1: B // 2] = 1.0

        signal_powers = np.einsum('bkn,ksn->bks', accum_symbols, shifted_masks_mat)
        band_powers = np.einsum('bkn,n->bk', accum_symbols, band)[:, :, np.newaxis]

        # Regularize the out-of-mask power with 1% of the signal's median
        # block power: on a noise-free recording the rest-band can be ~zero
        # (silence, or pure on-bin tones with no leakage), which would blow
        # the contrast ratio up to ~1/eps at arbitrary offsets and make the
        # argmax a numerical lottery. Under real noise the extra term is
        # negligible (<=1%).
        block_band_power = spectrogram[:, 1: B // 2].sum(axis=1)
        floor_per_block = float(np.median(block_band_power))
        window_blocks = np.array([tone0_blocks if k == 0 else tone1_blocks for k in range(K)])
        floor_windows = 0.01 * floor_per_block * window_blocks[np.newaxis, :, np.newaxis]

        rest_powers = np.maximum(band_powers - signal_powers, 1e-10) + floor_windows

        contrast = (signal_powers / n_mask_bins) / (rest_powers / (band.sum() - n_mask_bins))
        metrics = np.prod(contrast, axis=1)  # (num_offsets, num_shifts)

        best_flat = int(np.argmax(metrics))
        best_block_idx, best_cfo_idx = np.unravel_index(best_flat, metrics.shape)
        best_metric = metrics[best_block_idx, best_cfo_idx]

        if best_metric < self._newman_threshold:
            return None

        sample_index = int(best_block_idx) * B
        coarse_cfo_hz = cfo_shifts[best_cfo_idx] * fine_bin_hz

        preamble_part = signal[sample_index: sample_index + 2 * T * N]

        t = np.arange(len(preamble_part))
        preamble_corrected = preamble_part * np.exp(-1j * 2 * np.pi * coarse_cfo_hz * t / self._sample_rate)

        if s > 1:
            # the fine coarse grid leaves a residual under half a fine bin,
            # well inside the lag-N estimator's unambiguous +-fs/(2N) range
            r_sum = np.sum(np.conj(preamble_corrected[:-N]) * preamble_corrected[N:])
            residual = float(np.angle(r_sum) * self._sample_rate / (2 * np.pi * N))
        else:
            residual = self._estimate_residual_cfo(preamble_corrected)

        cfo = coarse_cfo_hz + residual

        return sample_index, cfo

    def _estimate_residual_cfo(self, preamble_corrected: npt.NDArray[np.complex64]) -> float:
        """Residual CFO of the (coarse-corrected) first tone block.

        A plain lag-N phase estimate is ambiguous modulo one subcarrier
        spacing (fs/N), so it cannot recover a +-1-bin coarse decision error
        -- which happens whenever the true CFO sits near a half-bin and
        leakage splits the tone energy between adjacent bins. Instead, take a
        full-length FFT of the tone block (resolution fs/(2*T*N) ~= 4.7 Hz,
        ~+13 dB coherent gain over the block-wise masks) and locate the common
        frequency offset of the tones within +-1.5 coarse bins, then refine
        with the lag-N phase estimate unwrapped onto it.
        """
        N = self._fft_bins
        F = len(preamble_corrected)
        spectrum = np.abs(np.fft.fft(preamble_corrected)) ** 2
        stride = F // N  # fine bins per coarse bin

        max_off = (3 * stride) // 2
        offsets = np.arange(-max_off, max_off + 1)
        tone_score = np.zeros(len(offsets))
        for b in self._newman_preamble_bins:
            tone_score += spectrum[(b * stride + offsets) % F]

        d = int(np.argmax(tone_score))
        # parabolic interpolation around the peak
        if 0 < d < len(offsets) - 1:
            y0, y1, y2 = tone_score[d - 1], tone_score[d], tone_score[d + 1]
            denom = y0 - 2 * y1 + y2
            frac = 0.5 * (y0 - y2) / denom if abs(denom) > 1e-12 else 0.0
        else:
            frac = 0.0
        cfo_est = (offsets[d] + frac) * self._sample_rate / F

        # precise lag-N phase estimate, unwrapped onto the FFT estimate
        r_sum = np.sum(np.conj(preamble_corrected[:-N]) * preamble_corrected[N:])
        cfo_fine = np.angle(r_sum) * self._sample_rate / (2 * np.pi * N)

        bin_width_hz = self._sample_rate / N
        k = np.round((cfo_est - cfo_fine) / bin_width_hz)
        return float(cfo_fine + k * bin_width_hz)

    def detect_preamble(self, signal: npt.NDArray[np.complex64]) -> typing.Optional[typing.Tuple[int, float]]:
        if not (coarse := self.detect_newman_preamble(signal)):
            return None

        coarse_sample, coarse_cfo = coarse
        coarse_shifted = freq_shift(self._sample_rate, signal[coarse_sample:], coarse_cfo)

        # the ZC preamble sits right behind the tone preamble of a known
        # length, so restrict the matched-filter search to that window
        # (slack covers the coarse stage's block granularity)
        N = self._fft_bins
        tones_len = 3 * self._newman_preamble_tile * N
        window_len = tones_len + self.symbol_len + 4 * N + (self._detect_fft_len or N)

        # The tone stage leaves a residual well below half a bin, so lock the
        # ZC matched filter to the m=0 hypothesis: scanning m=+-1 here would
        # let the ZC time-frequency ambiguity (a frequency-shifted replica
        # correlates almost as well at a shifted time) win at low SNR and
        # inject a one-bin CFO error plus a ~30-sample timing error.
        # Group-coherent kernels (long-symbol modes) are however sensitive to
        # the few-Hz CFO noise the tone stage leaves at very low SNR, so they
        # scan a small fractional grid around zero instead.
        if self._zc_coherent_group > 1:
            zc_hyp = np.arange(-15.0, 15.1, 5.0)
            fine = self.detect_zc_preamble(coarse_shifted[:window_len], cfo_hypotheses_hz=zc_hyp)
        else:
            fine = self.detect_zc_preamble(coarse_shifted[:window_len], max_cfo=0)
        if not fine:
            return None

        fine_sample, fine_cfo = fine

        cfo = coarse_cfo + fine_cfo
        return (fine_sample + coarse_sample, cfo)


class STFOFDMModem(FullOFDMModem):
    """802.11-STF-style tone preamble variant: tones every 8 bins.

    Comb A = bins [8, 16, 24] (all multiples of 8), comb B = [4, 12, 20].
    With tones on multiples of 8 the first tone block is periodic with
    N/8 = 16 samples, so the residual CFO comes from a plain
    delay-and-correlate estimate -- lag-16 phase (unambiguous over
    +-fs/32 = +-375 Hz, covering the protocol's +-300 Hz target) refined by
    the precise lag-N phase -- with no FFT peak search and no ambiguity
    bookkeeping. Detection still uses the block-spectrogram contrast metric
    inherited from FullOFDMModem.
    """

    STF_PERIOD_BINS = 8

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._newman_preamble_bins = np.arange(8, self._bin_hi + 1, self.STF_PERIOD_BINS)
        self._newman_preamble_shifts = [0, -4]

    def _estimate_residual_cfo(self, preamble_corrected: npt.NDArray[np.complex64]) -> float:
        N = self._fft_bins
        period = N // self.STF_PERIOD_BINS  # 16 samples

        # "STF": short-lag estimate over one comb period, wide unambiguous range
        r_short = np.sum(np.conj(preamble_corrected[:-period]) * preamble_corrected[period:])
        cfo_short = np.angle(r_short) * self._sample_rate / (2 * np.pi * period)

        # "LTF": precise lag-N estimate, unwrapped onto the short-lag one
        r_long = np.sum(np.conj(preamble_corrected[:-N]) * preamble_corrected[N:])
        cfo_long = np.angle(r_long) * self._sample_rate / (2 * np.pi * N)

        bin_width_hz = self._sample_rate / N
        k = np.round((cfo_short - cfo_long) / bin_width_hz)
        return float(cfo_long + k * bin_width_hz)
