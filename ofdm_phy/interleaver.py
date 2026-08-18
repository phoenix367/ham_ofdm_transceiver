"""Transpose-based block interleaver over the data subcarriers."""

import numpy as np
import numpy.typing as npt


def interleave(bits: npt.NDArray, num_carriers: int) -> npt.NDArray:
    data_len = len(bits)

    num_rows = int(np.ceil(data_len / num_carriers))
    total_cells = num_rows * num_carriers

    indices = np.arange(total_cells)
    interleaved_indices = indices.reshape(num_rows, num_carriers).T.flatten()
    pruned_indices = interleaved_indices[interleaved_indices < data_len]

    return bits[pruned_indices]


def deinterleave(symbols: npt.NDArray, num_carriers: int) -> npt.NDArray:
    data_len = len(symbols)

    num_rows = int(np.ceil(data_len / num_carriers))
    total_cells = num_rows * num_carriers

    indices = np.arange(total_cells)
    interleaved_indices = indices.reshape(num_rows, num_carriers).T.flatten()
    pruned_indices = interleaved_indices[interleaved_indices < data_len]

    deinterleaved = np.zeros(data_len, dtype=symbols.dtype)
    deinterleaved[pruned_indices] = symbols

    return deinterleaved
