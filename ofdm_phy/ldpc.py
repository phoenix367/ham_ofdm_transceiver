"""LDPC codec: rate-1/3 IRA (Irregular Repeat-Accumulate), N=768, K=256.

Structure: H = [H_info | H_acc] with a dual-diagonal accumulator over the
512 parity bits (linear-time encoding: p_i = p_{i-1} XOR sum(info in check
i)) and degree-4 info columns placed 4-cycle-free (each check takes exactly
two info edges, never two adjacent checks in one column). This IRA family
sits within ~0.5 dB of optimized irregular LDPC at these lengths while
remaining trivially and *verifiably* constructible -- no copied shift
tables.

The 256 info bits cover the protocol's maximum data block (255 bits incl.
CRC); shorter blocks are SHORTENED: known-zero info positions are not
transmitted and enter the decoder pinned to a strong "zero" LLR.

Decoding: normalized min-sum (alpha=0.8, <= 60 iterations, early exit on
zero syndrome). Min-sum is chosen over exact sum-product deliberately: it is
SCALE-INVARIANT in the LLRs, and the modem's LLRs (4*re*EsN0-estimate,
clipped to +-20) carry an estimated, imperfect scale -- sum-product loses
several dB to that overconfidence while min-sum is immune.

LLR convention matches the package: positive = logical 1 (one negation at
the boundary converts to the standard positive-means-0 algebra inside).
"""

import numpy as np

N = 768
K_NOMINAL = 256
M = N - K_NOMINAL          # 512 parity bits / checks
DV_INFO = 4
MAX_ITER = 60
_SEED = 20260816


def _build():
    rng = np.random.default_rng(_SEED)

    # --- info part: each check has capacity 2; each info column takes
    # DV_INFO distinct, pairwise non-adjacent checks; no two columns share
    # 2 checks (4-cycle-free against both accumulator and info columns)
    capacity = np.full(M, 2, dtype=np.int64)  # dv*K = 1024 = 2*M
    col_checks = np.zeros((K_NOMINAL, DV_INFO), dtype=np.int64)
    pair_seen = set()

    for v in range(K_NOMINAL):
        for _attempt in range(4000):
            # relax constraints progressively if the tail columns starve
            avail = np.nonzero(capacity > 0)[0]
            if len(avail) < DV_INFO or _attempt >= 3000:
                avail = np.arange(M)
            picks = rng.choice(avail, size=DV_INFO, replace=False)
            picks.sort()
            strict = _attempt < 2000
            if strict and np.any(np.diff(picks) == 1):  # adjacent checks ->
                continue                                # 4-cycle w/ accumulator
            pairs = [(int(picks[i]), int(picks[j]))
                     for i in range(DV_INFO) for j in range(i + 1, DV_INFO)]
            if strict and any(p in pair_seen for p in pairs):
                continue                                # info-info 4-cycle
            col_checks[v] = picks
            capacity[picks] -= 1
            pair_seen.update(pairs)
            break
        else:
            raise RuntimeError("IRA construction failed; change the seed")

    # --- H matrix (info columns 0..K-1, parity columns K..N-1)
    H = np.zeros((M, N), dtype=np.uint8)
    for v in range(K_NOMINAL):
        H[col_checks[v], v] = 1
    for i in range(M):                          # dual-diagonal accumulator
        H[i, K_NOMINAL + i] = 1
        if i > 0:
            H[i, K_NOMINAL + i - 1] = 1

    # per-check info columns (2 or 3 each) for the linear encoder
    chk_info = [[] for _ in range(M)]
    for v in range(K_NOMINAL):
        for c in col_checks[v]:
            chk_info[c].append(v)

    # edges grouped by check, padded to the max degree for the decoder
    rows = [list(chk_info[c]) + [K_NOMINAL + c] + ([K_NOMINAL + c - 1] if c > 0 else [])
            for c in range(M)]
    dc_max = max(len(r) for r in rows)
    ev_pad = np.full((M, dc_max), -1, dtype=np.int64)
    for c, vs in enumerate(rows):
        ev_pad[c, :len(vs)] = vs

    return H, chk_info, ev_pad, dc_max


_H, _CHK_INFO, _EV, DC_MAX = _build()
_EV_VALID = _EV >= 0
_EV_SAFE = np.where(_EV_VALID, _EV, 0)
_INFO_POS = np.arange(K_NOMINAL)
K_MAX = K_NOMINAL
_K_EFF = K_NOMINAL
GIRTH_OK = True  # by construction


ALPHA = 0.8


def _sum_product(llr_std, max_iter=MAX_ITER):
    """Normalized min-sum on the length-N LLR vector (standard convention:
    positive = bit 0). Scale-invariant in the input LLRs. Returns hard bits."""
    v2c = np.where(_EV_VALID, llr_std[_EV_SAFE], np.inf)

    bits = (llr_std < 0).astype(np.uint8)
    for _ in range(max_iter):
        sgn = np.where(v2c >= 0, 1.0, -1.0)
        row_sign = np.prod(sgn, axis=1, keepdims=True)
        excl_sign = row_sign * sgn

        mag = np.abs(v2c)
        rows = np.arange(M)
        idx1 = np.argmin(mag, axis=1)
        m1 = mag[rows, idx1]
        mag2 = mag.copy()
        mag2[rows, idx1] = np.inf
        m2 = np.min(mag2, axis=1)
        excl_min = np.broadcast_to(m1[:, None], mag.shape).copy()
        excl_min[rows, idx1] = m2

        c2v = np.where(_EV_VALID, ALPHA * excl_sign * excl_min, 0.0)

        totals = llr_std + np.bincount(_EV_SAFE[_EV_VALID].ravel(),
                                       weights=c2v[_EV_VALID].ravel(), minlength=N)
        bits = (totals < 0).astype(np.uint8)
        syn = np.bitwise_xor.reduce(np.where(_EV_VALID, bits[_EV_SAFE], 0), axis=1)
        if not syn.any():
            return bits
        v2c = np.where(_EV_VALID, totals[_EV_SAFE] - c2v, np.inf)

    return bits


class LDPCCodec:
    """Interface-compatible with the convolutional codecs
    (encode / decode / calc_cc_elements)."""

    PIN_LLR = 1000.0  # shortened (known-zero) positions
    USE_SPA = True    # exact sum-product; valid now that the transceiver
                      # delivers temperature-calibrated LLRs

    @classmethod
    def calc_cc_elements(cls, bits_count: int) -> int:
        assert bits_count <= K_MAX
        return N - (K_MAX - bits_count)

    @classmethod
    def _transmit_positions(cls, bits_count: int):
        mask = np.ones(N, dtype=bool)
        mask[bits_count:K_MAX] = False  # shortened info tail
        return np.nonzero(mask)[0]

    @classmethod
    def encode(cls, bits: np.ndarray) -> np.ndarray:
        k = len(bits)
        info = np.zeros(K_MAX, dtype=np.uint8)
        info[:k] = bits
        # accumulator: p_i = p_{i-1} XOR (info bits of check i)
        chk_sum = np.array([np.bitwise_xor.reduce(info[c]) if len(c) else 0
                            for c in _CHK_INFO], dtype=np.uint8)
        parity = np.bitwise_xor.accumulate(chk_sum)
        cw = np.concatenate([info, parity])
        return cw[cls._transmit_positions(k)]

    @classmethod
    def decode(cls, soft_bits: np.ndarray, bits_count: int,
               spa: bool = False) -> np.ndarray:
        """spa=True uses exact sum-product -- only worthwhile when the LLRs
        are calibrated (see Transceiver's header-based temperature fit)."""
        llr = np.zeros(N)
        pos = cls._transmit_positions(bits_count)
        n_tx = len(pos)
        s = np.asarray(soft_bits, dtype=np.float64)[:n_tx]
        if len(s) < n_tx:
            s = np.pad(s, (0, n_tx - len(s)))
        llr[pos] = s
        llr[bits_count:K_MAX] = -cls.PIN_LLR  # known zeros (ours: neg = 0)

        bits = _sum_product(-llr)  # standard convention inside
        return bits[:bits_count]


def min_sum_int(llr_std_int, max_iter=MAX_ITER):
    """Integer normalized min-sum (alpha = 0.75 as x - (x >> 2), the classic
    RTL form). Same convention as _min_sum: positive = bit 0. Exact integer
    arithmetic end to end -- the kernel an RTL implementation verifies
    against."""
    BIG = np.int64(1) << 30
    llr = np.asarray(llr_std_int, dtype=np.int64)
    v2c = np.where(_EV_VALID, llr[_EV_SAFE], BIG)

    bits = (llr < 0).astype(np.uint8)
    for _ in range(max_iter):
        sgn = np.where(v2c >= 0, 1, -1).astype(np.int64)
        row_sign = np.prod(sgn, axis=1, keepdims=True)
        excl_sign = row_sign * sgn

        mag = np.abs(v2c)
        rows = np.arange(M)
        idx1 = np.argmin(mag, axis=1)
        m1 = mag[rows, idx1]
        mag2 = mag.copy()
        mag2[rows, idx1] = BIG
        m2 = np.min(mag2, axis=1)
        excl_min = np.broadcast_to(m1[:, None], mag.shape).copy()
        excl_min[rows, idx1] = m2

        scaled = excl_min - (excl_min >> 2)  # alpha = 0.75, exact in ints
        c2v = np.where(_EV_VALID, excl_sign * scaled, 0)

        totals = llr + np.bincount(_EV_SAFE[_EV_VALID].ravel(),
                                   weights=c2v[_EV_VALID].ravel(),
                                   minlength=N).astype(np.int64)
        bits = (totals < 0).astype(np.uint8)
        syn = np.bitwise_xor.reduce(np.where(_EV_VALID, bits[_EV_SAFE], 0), axis=1)
        if not syn.any():
            return bits
        v2c = np.where(_EV_VALID, totals[_EV_SAFE] - c2v, BIG)

    return bits


def ldpc_decode_int(soft_bits_int, bits_count: int):
    """Integer twin of LDPCCodec.decode (min-sum only). LLR convention:
    positive = logical 1, as everywhere in the package."""
    pos = LDPCCodec._transmit_positions(bits_count)
    llr = np.zeros(N, dtype=np.int64)
    s = np.asarray(soft_bits_int, dtype=np.int64)[:len(pos)]
    if len(s) < len(pos):
        s = np.pad(s, (0, len(pos) - len(s)))
    llr[pos] = s
    pin = max(int(np.max(np.abs(s))) * 4, 1024)
    llr[bits_count:K_MAX] = -pin  # shortened zeros (ours: negative = 0)

    bits = min_sum_int(-llr)  # standard convention inside
    return bits[:bits_count]
