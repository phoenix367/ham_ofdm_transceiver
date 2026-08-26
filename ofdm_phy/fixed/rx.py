"""Fixed-point receiver: int16 audio in, decoded packet out.

Datapath (all integer):
  FIR Hilbert (Q15 taps)            -> analytic I/Q
  block-floating FFT spectrogram    -> tone detection (integer contrast, Q10)
  NCO derotation (32-bit CFO word)  -> coarse CFO removal
  Q15 ZC kernel correlation         -> timing (alpha-max/beta-min magnitude)
  lag-N correlation + CORDIC atan2  -> fine CFO as a phase word
  NCO grid + tile sum + BFP FFT     -> per-symbol residual-CFO search
  Q15 pilot rotation + weight ROM   -> channel estimate (division-free:
                                       ZC pilots have unit magnitude)
  Re/Im(Y*conj(H)), exponent-aligned -> 6-bit LLRs
  integer Viterbi + CRC             -> packet

CFO is carried end-to-end as a 32-bit phase-increment word; Hz appear only
in logs. The tone-detection FFT length is 256 for NORMAL (float model: 128)
so the coarse grid is fine enough for an unambiguous lag-N residual -- the
one intentional deviation, noted here.
"""

import dataclasses

import numpy as np

from ..modes import LinkMode, MODE_SPECS, make_modem
from ..packets import Header, PACKET_CLASSES
from ..scrambler import descramble, scramble
from ..interleaver import deinterleave, interleave
from ..ldpc import LDPCCodec, ldpc_decode_int
from ..transceiver import CODECS, MAPPERS, HEADER_CODEC, HEADER_MAPPER, DemodError
from .fxp import Q15, rshift_round, isqrt_i64
from .fft import fft_bfp
from .dsp import HilbertFIR, NCO, cordic_atan2, hz_to_phase_word, phase_word_to_hz, PHASE_ONE
from .viterbi import viterbi_decode_int, quantize_llr

Q15_MAX = (1 << Q15) - 1

FIXED_DETECT_FFT = {LinkMode.NORMAL: 256, LinkMode.ROBUST: 512, LinkMode.EXTREME: 512}


def _div_round(a: int, b: int) -> int:
    """Signed round-to-nearest integer division."""
    if a >= 0:
        return (a + b // 2) // b
    return -((-a + b // 2) // b)


@dataclasses.dataclass
class FixedRxStats:
    """Per-frame stats of the last successful receive() -- the fixed twin of
    the float chain's RxStats subset the link layer consumes."""
    header: Header
    start_sample: int
    cfo_hz: float
    snr_db: float
    harq_combined: bool = False


class FixedReceiver:
    # Measured monotone reliability ROM for the CALIBRATED integer LLR
    # domain: index = |L_cal_q2| >> 2 (0..31), output = log-odds of the
    # EMPIRICAL error rate at that reliability, x4. Trained on the fixed
    # chain itself (NORMAL BPSK 1/3 at -7..-9 dB, article channel, 49k
    # samples). The calibrated LLRs are strongly overconfident at the top
    # (|L| claiming log-odds ~25 where reality is ~3.4), so the ROM is
    # compressive -- same shape finding as the float chain's map.
    RECAL_ROM = np.array([2, 2, 2, 3, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 10,
                          10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 15,
                          15, 15, 15, 15], dtype=np.int64)

    def __init__(self, mode: LinkMode = LinkMode.NORMAL, calibrate: bool = False):
        # calibrate=True: header-based integer temperature fit brings LLRs to
        # a stable calibrated scale (one divider), then the reliability ROM
        # reshapes them -- mirrors the float system chain (llr_recal="auto").
        # Default False = bit-parity with the DEFAULT float chain.
        self.calibrate = calibrate
        self.mode = mode
        self.spec = MODE_SPECS[mode]
        m = self._m = make_modem(mode)  # float modem supplies constants only
        self.fs = m.sample_rate
        self.N = m.fft_bins
        self.T = m._newman_preamble_tile
        self.tiles = m.sym_tile
        self.symbol_len = m.symbol_len
        self.cp = m.cyclic_prefix
        self._tile_db = 10.0 * np.log10(m.sym_tile)  # coherent tiling gain
        self.last_stats = None  # per-frame stats of the last receive()
        self.coarse_search = True  # two-stage first-symbol frequency search

        # --- tone-detection constants
        self.B = FIXED_DETECT_FFT[mode]
        self.s = self.B // self.N
        halfwidth = 0 if self.s == 1 else 1
        K = len(m._newman_preamble_shifts)
        self.masks = np.zeros((K, self.B), dtype=np.int64)
        for i, shift in enumerate(m._newman_preamble_shifts):
            for b in (m._newman_preamble_bins + shift) * self.s:
                lo = max(b - halfwidth, 1)
                hi = min(b + halfwidth, self.B // 2 - 1)
                self.masks[i, lo:hi + 1] = 1
        self.n_mask_bins = int(self.masks[0].sum())
        self.band = np.zeros(self.B, dtype=np.int64)
        self.band[1:self.B // 2] = 1
        self.n_band_bins = int(self.band.sum())
        self.newman_thr_q10 = int(round(self.spec.newman_threshold * 1024))
        self.max_shift = m._max_cfo * self.s
        self.word_per_fine_bin = PHASE_ONE // self.B

        # --- ZC correlation constants (Q15 kernel ROM)
        pre_block = m.gen_zc_preamble()
        self.zc_L = m._zc_preamble_count
        self.zc_G = min(m._zc_coherent_group, self.zc_L)
        self.zc_groups = self.zc_L // self.zc_G
        klen = self.zc_G * self.N
        kern = pre_block[-klen:]
        self.zc_rom = (np.round(kern.real * Q15_MAX).astype(np.int64),
                       np.round(kern.imag * Q15_MAX).astype(np.int64))
        self.zc_ref_energy = int(np.sum(self.zc_rom[0] ** 2 + self.zc_rom[1] ** 2) >> Q15)
        self.preamble_block_len = len(pre_block)
        self.zc_thr_q10 = int(round(m._detection_threshold * 1024))
        if self.zc_G > 1:
            self.zc_hyp_words = [hz_to_phase_word(f, self.fs) for f in np.arange(-15.0, 15.1, 5.0)]
        else:
            self.zc_hyp_words = [0]

        # --- channel-estimate interpolation ROM (np.interp weights, Q15)
        pilots = m._get_pilot_values()
        self.pilot_conj_rom = (np.round(pilots.real * Q15_MAX).astype(np.int64),
                               np.round(-pilots.imag * Q15_MAX).astype(np.int64))
        pk = m._pilot_carriers.astype(int)
        self.interp = []  # per channel index: (lo_slot, hi_slot, w_hi_q15)
        for ci in m.channel_indices:
            hi = int(np.searchsorted(pk, ci))
            if hi == 0:
                self.interp.append((0, 0, 0))
            elif hi >= len(pk):
                self.interp.append((len(pk) - 1, len(pk) - 1, 0))
            else:
                lo = hi - 1
                w = ((ci - pk[lo]) << Q15) // (pk[hi] - pk[lo])
                self.interp.append((lo, hi, int(w)))

        # --- per-symbol residual-CFO search grid
        sym_dur = self.symbol_len / self.fs
        step_hz = 0.125 / sym_dur
        freqs = np.arange(-self.spec.freq_range, self.spec.freq_range + step_hz / 2, step_hz)
        self.search_words = [hz_to_phase_word(f, self.fs) for f in freqs]

        self.hilbert = HilbertFIR()

    # ------------------------------------------------------------------ tone
    def _detect_newman(self, i_arr, q_arr):
        B, T, N, s = self.B, self.T, self.N, self.s
        num_blocks = len(i_arr) // B
        tone0 = 2 * T * N // B
        tone1 = T * N // B
        total = tone0 + tone1
        if num_blocks < total:
            return None

        # per-block BFP power spectra aligned to the global minimum exponent
        pows = np.zeros((num_blocks, B), dtype=np.int64)
        exps = np.zeros(num_blocks, dtype=np.int64)
        for b in range(num_blocks):
            re, im, e = fft_bfp(i_arr[b * B:(b + 1) * B], q_arr[b * B:(b + 1) * B], 13)
            pows[b] = re * re + im * im
            exps[b] = e
        e_min = int(exps.min())
        for b in range(num_blocks):
            sh = 2 * int(exps[b] - e_min)
            pows[b] = pows[b] >> min(sh, 62)

        cum = np.vstack([np.zeros((1, B), dtype=np.int64), np.cumsum(pows, axis=0)])
        n_off = num_blocks - total + 1
        offs = np.arange(n_off)
        win0 = cum[offs + tone0] - cum[offs]          # (n_off, B)
        win1 = cum[offs + tone0 + tone1] - cum[offs + tone0]

        band_pow = np.stack([win0 @ self.band, win1 @ self.band])  # (2, n_off)
        block_band = pows[:, 1:B // 2].sum(axis=1)
        floor = int(np.median(block_band))

        best = (-1, 0, 0)  # metric_q20, offset, shift
        shifts = range(-self.max_shift, self.max_shift + 1)
        for sh in shifts:
            m0 = np.roll(self.masks[0], sh)
            m1 = np.roll(self.masks[1], sh)
            sig0 = win0 @ m0
            sig1 = win1 @ m1
            rest0 = np.maximum(band_pow[0] - sig0, 1) + (floor * tone0) // 100
            rest1 = np.maximum(band_pow[1] - sig1, 1) + (floor * tone1) // 100
            c0 = (sig0 * self.n_band_bins * 1024) // (rest0 * self.n_mask_bins)
            c1 = (sig1 * self.n_band_bins * 1024) // (rest1 * self.n_mask_bins)
            metric = (c0 * c1)  # Q20
            j = int(np.argmax(metric))
            if metric[j] > best[0]:
                best = (int(metric[j]), j, sh)

        metric_q20, off, sh = best
        if metric_q20 < self.newman_thr_q10 ** 2:
            return None

        sample_index = off * B
        coarse_word = sh * self.word_per_fine_bin

        # residual CFO: lag-N phase of the derotated first tone field
        seg_i = i_arr[sample_index: sample_index + 2 * T * N]
        seg_q = q_arr[sample_index: sample_index + 2 * T * N]
        di, dq = NCO.derotate(seg_i, seg_q, coarse_word)
        rr = int(np.sum(di[:-N] * di[N:] + dq[:-N] * dq[N:]))
        ri = int(np.sum(di[:-N] * dq[N:] - dq[:-N] * di[N:]))
        angle_word, _ = cordic_atan2(ri, rr)
        residual_word = _div_round(angle_word, N)

        return sample_index, coarse_word + residual_word

    # -------------------------------------------------------------------- ZC
    def _detect_zc(self, i_arr, q_arr):
        N, L, G = self.N, self.zc_L, self.zc_G
        klen = G * N
        ng = self.zc_groups
        preamble_len = L * N
        if len(i_arr) < preamble_len + self.cp:
            return None

        energy = i_arr * i_arr + q_arr * q_arr
        ecs = np.concatenate([[0], np.cumsum(energy)])

        best = (-1, 0, 0)  # metric2_q20, idx, hyp_word
        for w in self.zc_hyp_words:
            # hypothesis-derotated Q15 kernel (RTL: rotate input once per hyp)
            kr, ki = NCO.derotate(self.zc_rom[0], self.zc_rom[1], -w)
            # complex correlation via four real convolutions (int16 x Q15)
            a = np.convolve(i_arr, kr[::-1], mode="valid") + np.convolve(q_arr, ki[::-1], mode="valid")
            b = np.convolve(q_arr, kr[::-1], mode="valid") - np.convolve(i_arr, ki[::-1], mode="valid")
            a = rshift_round(a, Q15)
            b = rshift_round(b, Q15)
            # alpha-max + beta-min/2 magnitude (RTL-standard approximation)
            aa, bb = np.abs(a), np.abs(b)
            mag = np.maximum(aa, bb) + (np.minimum(aa, bb) >> 1)

            cc = np.zeros(len(mag) - (ng - 1) * klen, dtype=np.int64)
            for g in range(ng):
                cc += mag[g * klen: g * klen + len(cc)]

            n_pos = min(len(cc), len(i_arr) - preamble_len + 1)
            we = ecs[preamble_len:preamble_len + n_pos] - ecs[:n_pos]
            # metric^2 in Q20: (cc*2^15)^2 / (ng * we * ref_e * 2^15) -- the
            # correlator output and ref_e each already carry a /2^15 from the
            # Q15 kernel, leaving a net 2^15 to restore. Staged shifts keep
            # every product inside int64:
            #   num = cc^2/2^10, den ~ ng*we*ref/2^35 -> num<<10//den = m^2*2^20
            num = (cc[:n_pos] >> 5) ** 2
            den = np.maximum(((we >> 8) * ((ng * self.zc_ref_energy) >> 12)) >> 15, 1)
            metric2 = (num << 10) // den  # Q20
            j = int(np.argmax(metric2))

            # dynamic threshold in multiples of the base (as the float model)
            floor = int(np.mean(cc)) + 1
            ptf_q4 = (int(cc[j]) << 4) // floor
            thr = self.zc_thr_q10
            if ptf_q4 < 56:  # 3.5 in Q4
                thr = min(thr + (thr * (56 - ptf_q4)) // 32, (thr * 9) // 4)
            if metric2[j] >= thr * thr and metric2[j] > best[0]:
                best = (int(metric2[j]), j, w)

        if best[0] < 0:
            return None
        _, idx, w = best

        # fine CFO from the lag-N phase of the detected preamble
        seg_i, seg_q = i_arr[idx: idx + preamble_len], q_arr[idx: idx + preamble_len]
        di, dq = NCO.derotate(seg_i, seg_q, w)
        rr = int(np.sum(di[:-N] * di[N:] + dq[:-N] * dq[N:]))
        ri = int(np.sum(di[:-N] * dq[N:] - dq[:-N] * di[N:]))
        angle_word, _ = cordic_atan2(ri, rr)
        fine_word = w + _div_round(angle_word, N)

        final_time = (idx - self.cp) + self.preamble_block_len
        return final_time, fine_word

    def _detect(self, i_arr, q_arr):
        coarse = self._detect_newman(i_arr, q_arr)
        if coarse is None:
            return None
        cs, cw = coarse
        win = 3 * self.T * self.N + self.symbol_len + 4 * self.N + self.B
        wi, wq = NCO.derotate(i_arr[cs:cs + win], q_arr[cs:cs + win], cw)
        fine = self._detect_zc(wi, wq)
        if fine is None:
            return None
        ft, fw = fine
        return cs + ft, cw + fw

    # ----------------------------------------------------------------- demod
    COARSE_GATE_Q4 = 36  # 2.25x top1/median contrast: fast path only above

    def _coarse_hyp_window(self, seg_i, seg_q, cfo_word, base_phase, pos):
        """Two-stage frequency search, coarse pass: accumulate only the
        first tiles/4 tiles -- the frequency mainlobe is 4x wider, so a 4x
        coarser grid localizes the peak on 1/4 of the samples (~11x fewer
        derotated samples overall at EXTREME). Returns the fine +-3-step
        window around the coarse winner for the full-precision pass."""
        n_words = len(self.search_words)
        ct = max(4, self.tiles // 4)
        span = self.cp + ct * self.N
        cands = []
        for k in range(0, n_words, 4):
            w = self.search_words[k]
            di, dq = NCO.derotate(seg_i[:span], seg_q[:span], cfo_word + w,
                                  start_phase=(base_phase + w * pos) & (PHASE_ONE - 1))
            acc_i = di[self.cp:].reshape(ct, self.N).sum(axis=0)
            acc_q = dq[self.cp:].reshape(ct, self.N).sum(axis=0)
            re, im, exp = fft_bfp(acc_i, acc_q, 13)
            e = int(np.sum(re[self._m.channel_indices] ** 2 +
                           im[self._m.channel_indices] ** 2))
            cands.append((e, exp, k))
        # BFP exponent counts headroom left-shifts: true energy = e >> 2*exp
        # relative to the common minimum exponent (same as the fine pass)
        e_min = min(c[1] for c in cands)
        aligned = [(c[0] >> min(2 * (c[1] - e_min), 62), c[2]) for c in cands]
        ranked = sorted(aligned, reverse=True)
        # quality gate: the quarter-length coarse metric is ~6 dB noisier
        # than the full symbol, and at the EXTREME sensitivity edge its
        # argmax is unreliable (measured: top1/median contrast <= 2.2x at
        # -17/-18 dB vs >= 2.7x at -12). Below the gate, fall back to the
        # exhaustive grid -- lossless by construction; above it, the top-3
        # shortlist costs ~10x less than the full grid.
        median = max(ranked[len(ranked) // 2][0], 1)
        if (ranked[0][0] << 4) // median < self.COARSE_GATE_Q4:
            return None
        window = set()
        for _, k in ranked[:3]:
            window.update(range(max(0, k - 5), min(n_words, k + 6)))
        return sorted(window)

    def _demod_symbol(self, i_arr, q_arr, pos, cfo_word, mu, hyp_window=None):
        N, tiles = self.N, self.tiles
        sl = self.symbol_len
        if pos + sl > len(i_arr):
            raise DemodError("signal too short")

        seg_i = i_arr[pos:pos + sl]
        seg_q = q_arr[pos:pos + sl]
        base_phase = (cfo_word * pos) & (PHASE_ONE - 1)

        if hyp_window is None and self.coarse_search and len(self.search_words) > 9:
            hyp_window = self._coarse_hyp_window(seg_i, seg_q, cfo_word,
                                                 base_phase, pos)
        indices = range(len(self.search_words)) if hyp_window is None else hyp_window
        candidates = []
        for k in indices:
            w = self.search_words[k]
            di, dq = NCO.derotate(seg_i, seg_q, cfo_word + w,
                                  start_phase=(base_phase + w * pos) & (PHASE_ONE - 1))
            acc_i = di[self.cp:].reshape(tiles, N).sum(axis=0)
            acc_q = dq[self.cp:].reshape(tiles, N).sum(axis=0)
            re, im, exp = fft_bfp(acc_i, acc_q, 13)
            e = int(np.sum(re[self._m.channel_indices] ** 2 + im[self._m.channel_indices] ** 2))
            candidates.append((e, re, im, exp, k))

        # spectra scale as 2^(2*exp); compare energies on a common exponent
        e_min = min(c[3] for c in candidates)
        best = max(candidates, key=lambda c: c[0] >> min(2 * (c[3] - e_min), 62))
        _, re, im, exp, self._last_hyp = best
        m = self._m

        # channel estimate: rotate pilots by conj(ZC ref) -- |ref| = 1, so no
        # division -- then linear interpolation with the Q15 weight ROM
        yp_re = re[m._pilot_carriers]
        yp_im = im[m._pilot_carriers]
        hp_re = rshift_round(yp_re * self.pilot_conj_rom[0] - yp_im * self.pilot_conj_rom[1], Q15)
        hp_im = rshift_round(yp_re * self.pilot_conj_rom[1] + yp_im * self.pilot_conj_rom[0], Q15)

        h_re = np.zeros(len(m.channel_indices), dtype=np.int64)
        h_im = np.zeros(len(m.channel_indices), dtype=np.int64)
        for k, (lo, hi, w) in enumerate(self.interp):
            h_re[k] = (hp_re[lo] * ((1 << Q15) - w) + hp_re[hi] * w) >> Q15
            h_im[k] = (hp_im[lo] * ((1 << Q15) - w) + hp_im[hi] * w) >> Q15

        y_re = re[m.channel_indices][m._data_local_indices]
        y_im = im[m.channel_indices][m._data_local_indices]
        hd_re = h_re[m._data_local_indices]
        hd_im = h_im[m._data_local_indices]

        # matched-filter LLRs: Re/Im(Y * conj(H)); scale ~ 2^(2*exp)
        llr_i = y_re * hd_re + y_im * hd_im
        if mu == 1:
            llr = llr_i
        elif mu == 2:
            llr_q = y_im * hd_re - y_re * hd_im
            llr = np.empty(2 * len(llr_i), dtype=np.int64)
            llr[0::2] = llr_i
            llr[1::2] = llr_q
        else:
            # 16-QAM (Gray, bits [I-sign, I-inner, Q-sign, Q-inner]): the
            # sign bits are the matched-filter outputs; the inner/outer bits
            # compare |L0| against t_c = |H_c|^2 * (2*a*gain), with the
            # per-symbol amplitude reference estimated from the mean
            # matched-filter magnitude (E|x_I| = 2a for 16-QAM). One integer
            # division per symbol; Q8 ratio keeps products inside int64.
            llr_q0 = y_im * hd_re - y_re * hd_im
            h2 = hd_re * hd_re + hd_im * hd_im
            ssum = int(np.sum(np.abs(llr_i)) + np.sum(np.abs(llr_q0)))
            h2sum = max(int(np.sum(h2)), 1)
            ratio_q8 = (ssum << 8) // (2 * h2sum)
            t = (h2 * ratio_q8) >> 8
            llr = np.empty(4 * len(llr_i), dtype=np.int64)
            llr[0::4] = llr_i
            llr[1::4] = t - np.abs(llr_i)
            llr[2::4] = llr_q0
            llr[3::4] = t - np.abs(llr_q0)
        return llr, exp

    def _demod_block(self, i_arr, q_arr, start, cfo_word, n_syms, mapper):
        mu = mapper.MU
        raw = []
        exps = []
        for k in range(n_syms):
            # slew-limited residual-frequency tracker: full grid on the first
            # symbol of the frame, then +-2 grid steps around the previous
            # symbol's winner -- drift is smooth, per-symbol argmax is noisy
            if getattr(self, "_last_hyp", None) is None:
                window = None
            else:
                lo = max(0, self._last_hyp - 2)
                hi = min(len(self.search_words) - 1, self._last_hyp + 2)
                window = range(lo, hi + 1)
            llr, exp = self._demod_symbol(i_arr, q_arr, start + k * self.symbol_len,
                                          cfo_word, mu, hyp_window=window)
            raw.append(llr)
            exps.append(exp)
        # align block exponents (scale ~ 2^(2*exp)); the caller quantizes
        e_min = min(exps)
        arr = np.concatenate([v >> (2 * (e - e_min)) for v, e in zip(raw, exps)])
        return arr, 2 * e_min  # arr ~ L_raw << scale_log2

    @staticmethod
    def _quantize6(arr64):
        """Legacy peak-normalized 6-bit quantization (parity path)."""
        peak = int(np.max(np.abs(arr64))) if len(arr64) else 0
        shift = max(0, peak.bit_length() - 5)
        return np.clip(arr64 >> shift, -31, 31).astype(np.int64)

    @staticmethod
    def _quantize8(arr64):
        """Peak-normalized 8-bit quantization, used for 16-QAM data blocks:
        max-log LLRs span a much wider dynamic range (inner/outer bits x
        per-carrier gain), and the 6-bit compression costs ~1 dB on the
        puncture-weak 2/3 and 3/4 rates (measured, fixed_qam16_sweep.py)."""
        peak = int(np.max(np.abs(arr64))) if len(arr64) else 0
        shift = max(0, peak.bit_length() - 7)
        return np.clip(arr64 >> shift, -127, 127).astype(np.int64)

    def _quantize_data(self, arr64, mu):
        return self._quantize8(arr64) if mu == 4 else self._quantize6(arr64)

    def _decode_block(self, llrs, codec, bits_count):
        descrambled = descramble(llrs.astype(np.float64)).astype(np.int64)
        deinterleaved = deinterleave(descrambled, self._m.data_carriers_len)
        coded_len = codec.calc_cc_elements(bits_count)
        if codec is LDPCCodec:
            return ldpc_decode_int(deinterleaved[:coded_len], bits_count)
        return viterbi_decode_int(codec, deinterleaved[:coded_len], bits_count)

    def _known_ref(self, coded, cap):
        """Map re-encoded coded bits to +-1 in the received LLR stream's
        order (padded to whole symbols, interleaved, scrambled)."""
        n_syms = (len(coded) + cap - 1) // cap
        padded = np.pad(coded, (0, n_syms * cap - len(coded)))
        return 2 * scramble(interleave(padded, self._m.data_carriers_len)).astype(np.int64) - 1

    def _header_ref(self, hdr_bits):
        """Known-bit reference for the header block -- shared by the
        temperature fit and the SNR estimator."""
        coded = HEADER_CODEC.encode(hdr_bits.astype(np.uint8))
        return self._known_ref(coded, self._m.data_carriers_len * HEADER_MAPPER.MU)

    # log2(1 + i/16) in Q4 -- 16-entry mantissa ROM for the integer dB
    # conversion (RTL: priority encoder + this LUT)
    LOG2_FRAC_Q4 = np.array([0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
                             14, 15, 15], dtype=np.int64)

    @classmethod
    def _log2_q4(cls, v):
        """floor-ish log2 of a positive int, Q4 (1/16 bit resolution)."""
        bl = v.bit_length()
        mant = (v >> (bl - 5)) & 0xF if bl >= 5 else (v << (5 - bl)) & 0xF
        return ((bl - 1) << 4) + int(cls.LOG2_FRAC_Q4[mant])

    # calibration offset for the header-aided SNR estimate, measured against
    # the channel nominal: raw estimate biased +7.2 dB, constant across all
    # modes/modulations (in-band vs 6 kHz-band reference + estimator gain)
    SNR_CAL_DB = -7.2

    @staticmethod
    def _snr_block_moments(arr64, ref, cap):
        """Per-column moments of one LLR block, with per-symbol GAIN
        WEIGHTING: each symbol row is weighted by its mean |LLR| (its gain
        estimate), so a tiled frame spanning several fade cycles measures
        the signal-energy-weighted (arithmetic-average) SNR -- the float
        estimator's flavor, which the rate ladder was tuned against --
        instead of counting fading swings as noise. Weighted rows are
        scale-free relative to the block's BFP exponent, so blocks pool
        directly. With constant gain this reduces exactly to the plain
        per-column moment estimator. Returns (num, den) or None."""
        peak = int(np.max(np.abs(arr64))) if len(arr64) else 0
        shift = max(0, peak.bit_length() - 10)  # 10-bit rows: sums fit int64
        hf = (arr64[:len(ref)] >> shift).reshape(-1, cap)
        rf = np.asarray(ref).reshape(-1, cap)
        g = np.sum(np.abs(hf), axis=1) // cap  # per-symbol gain estimate
        keep = g > 0  # an all-zero row is a full-symbol erasure -- skip
        if int(np.sum(keep)) < 2:
            return None
        hf, rf, g = hf[keep], rf[keep], g[keep]
        s_c = np.sum(g[:, None] * hf * rf, axis=0)
        p_c = np.sum(hf * hf, axis=0)
        w = int(np.sum(g * g))
        num = int(np.sum(s_c * s_c))
        den = w * int(np.sum(p_c)) - num
        return (num, den) if num > 0 and den > 0 else None

    def _estimate_snr_db(self, blocks):
        """Data-aided SNR estimate: per-column moments of the aligned LLRs
        (columns remove the multipath |H|^2 spread, rows are gain-weighted
        against fading), pooled Es/N0 over all blocks, integer log2 -> dB,
        minus the tiling gain -- the integer twin of the float chain's
        stats.snr_db. blocks: [(llr_stream, known +-1 ref, columns), ...].
        Pure accumulators + a bit-length/LUT log; no dividers."""
        num = den = 0
        for arr64, ref, cap in blocks:
            m = self._snr_block_moments(arr64, ref, cap)
            if m is not None:
                num += m[0]
                den += m[1]
        if num <= 0 or den <= 0:
            return None
        l2 = self._log2_q4(num) - self._log2_q4(den)
        return l2 / 16.0 * (10.0 * np.log10(2.0)) - self._tile_db + self.SNR_CAL_DB

    def _fit_alpha_q12(self, h64, hdr_bits):
        """Integer temperature fit on the header block: 96 known bits after
        re-encoding vs the full-precision aligned LLRs. Gaussian consistency
        (Var = 2*Mean) gives alpha = 2*m*n / (n*ssq - m^2) -- one divider.
        Returns (alpha_q12, fit_shift) with alpha in the (h64 >> fit_shift)
        domain."""
        ref = self._header_ref(hdr_bits)

        peak = int(np.max(np.abs(h64))) if len(h64) else 0
        fit_shift = max(0, peak.bit_length() - 20)
        hf = h64[:len(ref)] >> fit_shift
        lx = hf * ref
        n = len(lx)
        m = int(np.sum(lx))
        ssq = int(np.sum(lx * lx))
        den = n * ssq - m * m
        if den <= 0 or m <= 0:
            return None, fit_shift
        alpha_q12 = (2 * m * n << 12) // den
        return max(alpha_q12, 1), fit_shift

    def _calibrated_llrs(self, d64, scale_d, alpha_q12, hdr_scale_fit):
        """Bring data LLRs to the calibrated domain (x4) and apply the
        reliability ROM. hdr_scale_fit = header scale_log2 + fit_shift."""
        shift = scale_d - hdr_scale_fit
        d_hf = (d64 >> shift) if shift >= 0 else (d64 << -shift)
        l_cal_q2 = (alpha_q12 * d_hf) >> 10  # calibrated value x4
        idx = np.minimum(np.abs(l_cal_q2) >> 2, 31)
        return np.sign(l_cal_q2) * self.RECAL_ROM[idx]

    # --------------------------------------------------------------- receive
    # ZC resync plausibility window: a lock further than this from the
    # nominal block boundary is a spurious correlation, not drift
    def _resync_win(self):
        return max(4 * self.cp, self.symbol_len // 8)

    def _stream_resync(self, i_arr, q_arr, pos, cfo_word, index, resyncs):
        """Re-lock timing and residual CFO on an interleaved ZC symbol.

        Unlike the float chain the signal is never bulk-derotated here --
        the fixed receiver carries `cfo_word` and derotates per symbol -- so
        the resync derotates only the search window, exactly as `_detect`
        does for the opening preamble, and folds the residual back into
        cfo_word.
        """
        win = self._resync_win()
        nominal = pos + self.symbol_len
        a = max(0, pos - win)
        b = pos + self.symbol_len + win
        if b <= len(i_arr):
            wi, wq = NCO.derotate(i_arr[a:b], q_arr[a:b], cfo_word)
            det = self._detect_zc(wi, wq)
            if det is not None and abs(a + det[0] - nominal) <= win:
                resyncs.append((index, a + det[0] - nominal, det[1]))
                # the CFO changed, so the slew-limited per-symbol tracker's
                # previous hypothesis is stale
                self._last_hyp = None
                return a + det[0], cfo_word + det[1]
        return nominal, cfo_word

    def receive_stream(self, samples, n_blocks=None, resync_every=4,
                       prev_llrs=None):
        """Fixed-point twin of Transceiver.demod_stream.

        Returns (packets, header, info) where packets[k] is None for a block
        that failed CRC and info carries start, cfo, the resync log and the
        per-block LLRs (kept on failure, for chase combining).
        """
        i_arr, q_arr = self.hilbert.analytic(np.asarray(samples, dtype=np.int64))

        det = self._detect(i_arr, q_arr)
        if det is None:
            raise DemodError("no preamble")
        start, cfo_word = det
        self._last_hyp = None

        pad = np.zeros(self.symbol_len, dtype=np.int64)
        i_arr = np.concatenate([i_arr, pad])
        q_arr = np.concatenate([q_arr, pad])

        n_hdr = -(-HEADER_CODEC.calc_cc_elements(Header.PACKET_SIZE) //
                  (self._m.data_carriers_len * HEADER_MAPPER.MU))
        try:
            h64, scale_h = self._demod_block(i_arr, q_arr, start, cfo_word,
                                             n_hdr, HEADER_MAPPER)
            hdr_bits = self._decode_block(self._quantize6(h64), HEADER_CODEC,
                                          Header.PACKET_SIZE)
            header = Header.decode(hdr_bits, check_crc=True)
        except DemodError:
            raise
        except Exception as exc:
            raise DemodError("head") from exc

        codec = LDPCCodec if header.ver == 2 else CODECS[header.spd]
        mapper = MAPPERS[header.mod]
        n_data = -(-codec.calc_cc_elements(header.len) //
                   (self._m.data_carriers_len * mapper.MU))
        block_len = n_data * self.symbol_len
        alpha_q12 = fit_shift = None
        if self.calibrate and mapper.MU <= 2:
            alpha_q12, fit_shift = self._fit_alpha_q12(h64, hdr_bits)

        packets, block_llrs, resyncs = [], [], []
        pos = start + n_hdr * self.symbol_len
        k = 0
        while n_blocks is None or k < n_blocks:
            if resync_every and k and k % resync_every == 0:
                pos, cfo_word = self._stream_resync(i_arr, q_arr, pos,
                                                    cfo_word, k, resyncs)
            if pos + block_len > len(i_arr):
                break
            d64, scale_d = self._demod_block(i_arr, q_arr, pos, cfo_word,
                                             n_data, mapper)
            if alpha_q12 is not None:
                llrs = self._calibrated_llrs(d64, scale_d, alpha_q12,
                                             scale_h + fit_shift)
            else:
                llrs = self._quantize_data(d64, mapper.MU)

            def attempt(stream):
                b = self._decode_block(stream, codec, header.len)
                return PACKET_CLASSES[header.typ].decode(b, check_crc=True)

            packet = None
            try:
                packet = attempt(llrs)
            except Exception:
                pass
            prev = prev_llrs.get(k) if prev_llrs else None
            if packet is None and prev is not None and len(prev) == len(llrs):
                try:
                    packet = attempt(llrs + prev)
                except Exception:
                    pass
            packets.append(packet)
            block_llrs.append(None if packet is not None else llrs)
            pos += block_len
            k += 1

        while n_blocks is not None and len(packets) < n_blocks:
            packets.append(None)
            block_llrs.append(None)

        info = dict(start=start, cfo_hz=phase_word_to_hz(cfo_word, self.fs),
                    resyncs=resyncs, llrs=block_llrs,
                    ok=sum(1 for p in packets if p is not None))
        return packets, header, info

    def receive(self, samples, prev_data_llrs=None):
        """samples: int16 audio. Returns (packet, header, start, cfo_hz).

        prev_data_llrs: stored integer LLRs of a previously failed frame
        (chase-combining HARQ); on data-stage failure the raised DemodError
        carries this frame's LLRs in .data_llrs."""
        i_arr, q_arr = self.hilbert.analytic(np.asarray(samples, dtype=np.int64))

        det = self._detect(i_arr, q_arr)
        if det is None:
            raise DemodError("no preamble")
        start, cfo_word = det
        self._last_hyp = None  # reset the residual-frequency tracker

        # tail padding tolerates a small positive timing slip
        pad = np.zeros(self.symbol_len, dtype=np.int64)
        i_arr = np.concatenate([i_arr, pad])
        q_arr = np.concatenate([q_arr, pad])

        n_hdr = -(-HEADER_CODEC.calc_cc_elements(Header.PACKET_SIZE) //
                  (self._m.data_carriers_len * HEADER_MAPPER.MU))
        try:
            h64, scale_h = self._demod_block(i_arr, q_arr, start, cfo_word,
                                             n_hdr, HEADER_MAPPER)
            hdr_bits = self._decode_block(self._quantize6(h64), HEADER_CODEC,
                                          Header.PACKET_SIZE)
            header = Header.decode(hdr_bits, check_crc=True)
        except DemodError:
            raise
        except Exception as exc:
            raise DemodError("head") from exc

        codec = LDPCCodec if header.ver == 2 else CODECS[header.spd]
        mapper = MAPPERS[header.mod]
        coded_len = codec.calc_cc_elements(header.len)
        n_data = -(-coded_len // (self._m.data_carriers_len * mapper.MU))
        pos = start + n_hdr * self.symbol_len

        d64, scale_d = self._demod_block(i_arr, q_arr, pos, cfo_word, n_data, mapper)

        if self.calibrate and mapper.MU <= 2:
            alpha_q12, fit_shift = self._fit_alpha_q12(h64, hdr_bits)
            if alpha_q12 is not None:
                llrs = self._calibrated_llrs(d64, scale_d, alpha_q12,
                                             scale_h + fit_shift)
            else:
                llrs = self._quantize_data(d64, mapper.MU)
        else:
            llrs = self._quantize_data(d64, mapper.MU)

        def attempt(llr_stream):
            b = self._decode_block(llr_stream, codec, header.len)
            return PACKET_CLASSES[header.typ].decode(b, check_crc=True), b

        packet = data_bits = None
        combined = False
        try:
            packet, data_bits = attempt(llrs)
        except Exception:
            pass
        if packet is None and prev_data_llrs is not None \
                and len(prev_data_llrs) == len(llrs):
            try:  # chase combining (CRC-gated, as in the float chain)
                packet, data_bits = attempt(llrs + prev_data_llrs)
                combined = True
            except Exception:
                pass
        if packet is None:
            raise DemodError("data", header=header, data_llrs=llrs)

        cfo_hz = phase_word_to_hz(cfo_word, self.fs)
        # SNR estimate over every symbol with known bits: header always, plus
        # the decoded data block for the linear-LLR modulations (MU<=2) --
        # a long tiled frame's data symbols average across fading where the
        # ~4 s header alone would sample a single fade valley
        blocks = [(h64, self._header_ref(hdr_bits),
                   self._m.data_carriers_len * HEADER_MAPPER.MU)]
        if mapper.MU <= 2:
            cap_d = self._m.data_carriers_len * mapper.MU
            ref_d = self._known_ref(codec.encode(data_bits.astype(np.uint8)), cap_d)
            blocks.append((d64, ref_d, cap_d))
        snr_db = self._estimate_snr_db(blocks)
        self.last_stats = FixedRxStats(
            header=header, start_sample=start, cfo_hz=cfo_hz,
            snr_db=snr_db if snr_db is not None else -30.0,
            harq_combined=combined)
        return packet, header, start, cfo_hz
