"""Integer soft-decision Viterbi decoder.

LLRs are int8 (clamped to +-31, ~6-bit soft bits); branch metrics are pure
adds/subtracts because the expected symbols are +-1; path metrics fit int32
without renormalization for this protocol's frame sizes (<= 261 steps x
max branch metric 3*31 -> < 2^15), which an RTL implementation can keep or
replace with the standard modulo-normalization trick.
"""

import numpy as np

LLR_BITS = 6
LLR_MAX = (1 << (LLR_BITS - 1)) - 1  # 31


def quantize_llr(llr64, exp_align):
    """Align per-symbol block exponents and clamp to int8 soft bits.
    llr64: list of int64 arrays; exp_align: per-array left-shift deficit."""
    aligned = [np.asarray(v, dtype=np.int64) >> int(s) for v, s in zip(llr64, exp_align)]
    flat = np.concatenate(aligned)
    peak = int(np.max(np.abs(flat))) if len(flat) else 0
    shift = max(0, peak.bit_length() - (LLR_BITS - 1))
    return np.clip(flat >> shift, -LLR_MAX, LLR_MAX).astype(np.int64)


def viterbi_decode_int(codec, soft_bits_int, bits_count: int):
    """Integer twin of ConvCodec.decode()/ConvCodecPunctured.decode():
    depuncture (zeros for punctured positions), then max-log Viterbi."""
    soft = np.asarray(soft_bits_int, dtype=np.int64)
    if hasattr(codec, "depuncture"):
        soft = codec.depuncture(soft.astype(np.float64), bits_count).astype(np.int64)

    total_steps = bits_count + codec.PAD_LEN
    soft = soft[:total_steps * codec.SPEED]
    if len(soft) < total_steps * codec.SPEED:
        soft = np.pad(soft, (0, total_steps * codec.SPEED - len(soft)))

    num_states = codec.NUM_STATES
    expected = codec.EXPECTED_SYMS.astype(np.int64)  # (S, 2, SPEED) of +-1

    n = np.arange(num_states)
    in_bits = n & 1
    prev0 = n >> 1
    prev1 = prev0 + (num_states >> 1)

    path_metrics = np.full(num_states, np.iinfo(np.int32).min // 2, dtype=np.int64)
    path_metrics[0] = 0

    tb_prev = np.zeros((total_steps, num_states), dtype=np.int64)

    for step in range(total_steps):
        rx = soft[step * codec.SPEED: (step + 1) * codec.SPEED]
        branch = expected.dot(rx)  # (S, 2), adds/subtracts only

        m0 = path_metrics[prev0] + branch[prev0, in_bits]
        m1 = path_metrics[prev1] + branch[prev1, in_bits]
        take0 = m0 >= m1
        path_metrics = np.where(take0, m0, m1)
        tb_prev[step] = np.where(take0, prev0, prev1)

    decoded = np.zeros(total_steps, dtype=np.uint8)
    state = int(np.argmax(path_metrics))
    for step in range(total_steps - 1, -1, -1):
        decoded[step] = state & 1
        state = int(tb_prev[step, state])

    return decoded[:bits_count]
