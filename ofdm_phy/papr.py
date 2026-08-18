"""Clip-and-Filter PAPR reduction (clip at RMS + threshold dB, then 8th-order
Butterworth low-pass)."""

import numpy as np
import numpy.typing as npt
from scipy import signal


def clip_and_filter(
        data: npt.NDArray[np.float64], papr_cutoff_db: float, cutoff_freq_hz: float,
        fs_hz: float, filter_order: int = 8
) -> npt.NDArray[np.float64]:
    rms = np.sqrt(np.mean(data ** 2))

    clip_threshold = rms * (10 ** (papr_cutoff_db / 20.0))

    clipped_data = np.clip(data, -clip_threshold, clip_threshold)

    nyquist = 0.5 * fs_hz
    normal_cutoff = cutoff_freq_hz / nyquist
    b, a = signal.butter(filter_order, normal_cutoff, btype='low', analog=False)

    filtered_data = signal.filtfilt(b, a, clipped_data)
    return filtered_data
