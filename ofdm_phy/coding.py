"""Non-systematic rate-1/3 convolutional code (LTE/NASA polynomials 133, 171, 165
octal, K=7) with puncturing to rates 1/2, 2/3, 3/4 and soft-decision Viterbi
decoding.

LLR convention (consistent with the BPSK mapper, bit 1 -> +1): positive soft
bit = logical 1, negative = logical 0; the magnitude expresses confidence.

The Viterbi decoder is a vectorized (over trellis states) equivalent of the
article's reference implementation.
"""

import numpy as np
import numpy.typing as npt


class MetaConvCodec(type):
    """Precomputes trellis constants (SPEED, MASK, PAD_LEN, NUM_STATES,
    NEXT_STATES, EXPECTED_SYMS) from POLY / K / PUNCTURE_MASK."""

    def __init__(cls, name, bases, namespace):
        super().__init__(name, bases, namespace)

        poly = getattr(cls, "POLY", None)
        k = getattr(cls, "K", None)
        if not poly or not k:
            return

        cls.SPEED = len(poly)
        cls.MASK = (1 << k) - 1
        cls.PAD_LEN = k - 1
        cls.NUM_STATES = 1 << (k - 1)

        states = np.arange(cls.NUM_STATES)
        cls.NEXT_STATES = np.zeros((cls.NUM_STATES, 2), dtype=np.int64)
        cls.EXPECTED_SYMS = np.zeros((cls.NUM_STATES, 2, cls.SPEED), dtype=np.float64)

        for s in states:
            for bit in (0, 1):
                reg = ((int(s) << 1) | bit) & cls.MASK
                cls.NEXT_STATES[s, bit] = reg & (cls.NUM_STATES - 1)
                for p_idx, p in enumerate(poly):
                    out_bit = bin(reg & p).count("1") % 2
                    # +1 encodes logical 1, -1 encodes logical 0 (LLR convention)
                    cls.EXPECTED_SYMS[s, bit, p_idx] = 2.0 * out_bit - 1.0


class ConvCodec(metaclass=MetaConvCodec):
    POLY: list = []
    K: int = 0
    SOFT_BIT_SCALE = 1.0

    # --- encoding ----------------------------------------------------------

    @classmethod
    def _encode(cls, bits: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
        k = 0
        state = 0
        symbols = np.zeros(len(bits) * len(cls.POLY), dtype=np.uint8)

        for bit in bits:
            state = ((state << 1) | int(bit)) & cls.MASK

            for poly in cls.POLY:
                n = state & poly
                symbols[k] = bin(n).count("1") % 2
                k += 1

        return symbols

    @classmethod
    def calc_cc_elements(cls, bits_count: int) -> int:
        return (bits_count + cls.PAD_LEN) * cls.SPEED

    @classmethod
    def encode(cls, bits: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
        data_tailed = np.pad(bits, pad_width=(0, cls.PAD_LEN), mode="constant")
        cc_data = cls._encode(data_tailed)
        return cc_data

    # --- decoding ----------------------------------------------------------

    @classmethod
    def _decode(cls, soft_bits: npt.NDArray[np.float64], total_steps: int) -> npt.NDArray[np.uint8]:
        num_states = cls.NUM_STATES
        num_polys = cls.SPEED

        # For a shift-register trellis the state n = (last K-1 input bits),
        # so its LSB is the input bit and its two predecessors are
        # n >> 1 and (n >> 1) + NUM_STATES/2.
        n = np.arange(num_states)
        in_bits = n & 1
        prev0 = n >> 1
        prev1 = prev0 + (num_states >> 1)

        path_metrics = np.full(num_states, -np.inf)
        path_metrics[0] = 0.0

        tb_prev_states = np.zeros((total_steps, num_states), dtype=np.int64)

        for step in range(total_steps):
            rx_chunk = soft_bits[step * num_polys: (step + 1) * num_polys]
            if len(rx_chunk) < num_polys:
                rx_chunk = np.pad(rx_chunk, (0, num_polys - len(rx_chunk)))

            branch_metrics = cls.EXPECTED_SYMS.dot(rx_chunk)  # (num_states, 2)

            m0 = path_metrics[prev0] + branch_metrics[prev0, in_bits]
            m1 = path_metrics[prev1] + branch_metrics[prev1, in_bits]

            take0 = m0 >= m1
            path_metrics = np.where(take0, m0, m1)
            tb_prev_states[step] = np.where(take0, prev0, prev1)

        decoded_bits = np.zeros(total_steps, dtype=np.uint8)
        best_state = int(np.argmax(path_metrics))

        for step in range(total_steps - 1, -1, -1):
            decoded_bits[step] = best_state & 1
            best_state = int(tb_prev_states[step, best_state])

        return decoded_bits

    @classmethod
    def decode(cls, soft_bits: npt.NDArray[np.float64], bits_count: int) -> npt.NDArray[np.uint8]:
        soft_bits_cropped = soft_bits[:(bits_count + cls.PAD_LEN) * cls.SPEED]
        bits = cls._decode(soft_bits_cropped, bits_count + cls.PAD_LEN)
        bits_cropped = bits[:bits_count]
        return bits_cropped


class ConvCodecPunctured(ConvCodec):
    PUNCTURE_MASK: np.ndarray = None

    @classmethod
    def calc_cc_elements(cls, bits_count: int) -> int:
        k = cls.PUNCTURE_MASK.shape[1]
        n = int(np.sum(cls.PUNCTURE_MASK == 1))

        num_bits = bits_count + cls.PAD_LEN
        blocks = (num_bits + k - 1) // k
        return blocks * n

    @classmethod
    def puncture(cls, bits: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
        mask = cls.PUNCTURE_MASK

        period = mask.shape[1]

        stream = np.array(bits).reshape(-1, cls.SPEED).T
        num_elements = stream.shape[1]

        reps = (num_elements + period - 1) // period
        full_mask = np.tile(mask, (1, reps))[:, :num_elements]

        return stream.T[full_mask.T.astype(bool)]

    @classmethod
    def depuncture(cls, soft_bits: npt.NDArray[np.float64], bits_count: int) -> npt.NDArray[np.float64]:
        mask = cls.PUNCTURE_MASK
        base_streams = mask.shape[0]
        period = mask.shape[1]

        num_elements = bits_count + cls.PAD_LEN
        if (mod := num_elements % period) != 0:
            num_elements += period - mod

        reps = num_elements // period
        full_mask = np.tile(mask, (1, reps))

        depunct_T = np.zeros((num_elements, base_streams), dtype=soft_bits.dtype)

        bool_mask_T = full_mask.T.astype(bool)

        slots = np.sum(bool_mask_T)
        actual_bits = min(soft_bits.size, slots)

        if slots > actual_bits:
            indices = np.where(bool_mask_T)
            bool_mask_T[indices[0][actual_bits:], indices[1][actual_bits:]] = False

        depunct_T[bool_mask_T] = soft_bits[:actual_bits]
        return depunct_T.ravel()

    @classmethod
    def encode(cls, bits: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
        cc_data = super().encode(bits)
        cc_data_punct = cls.puncture(cc_data)
        return cc_data_punct

    @classmethod
    def decode(cls, soft_bits: npt.NDArray[np.float64], bits_count: int) -> npt.NDArray[np.uint8]:
        soft_bits_depunct = cls.depuncture(soft_bits, bits_count)
        bits = super().decode(soft_bits_depunct, bits_count)
        return bits


class CCLTEBPSK(ConvCodecPunctured, metaclass=MetaConvCodec):
    PUNCTURE_MASK = np.array([[1], [1], [1]])

    POLY = [0o133, 0o171, 0o165]
    K = 7
    SOFT_BIT_SCALE = 1.0


CCLTEBPSK_13 = CCLTEBPSK


class CCLTEBPSK_12(CCLTEBPSK, metaclass=MetaConvCodec):
    PUNCTURE_MASK = np.array([[1, 1], [1, 0], [0, 1]])


class CCLTEBPSK_23(CCLTEBPSK, metaclass=MetaConvCodec):
    PUNCTURE_MASK = np.array([[1, 1], [1, 0], [0, 0]])


class CCLTEBPSK_34(CCLTEBPSK, metaclass=MetaConvCodec):
    PUNCTURE_MASK = np.array([[1, 1, 1], [1, 0, 0], [0, 0, 0]])
