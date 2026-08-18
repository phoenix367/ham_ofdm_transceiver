"""FixedPHY: a float-Transceiver-compatible adapter over the fixed-point
TX/RX, so the link layer (LinkStation) can run the full fixed pipeline.

Interface parity with the float chain:
  build_frame(pkt, mode, mod, spd) -> float audio in [-1, 1)
  demod_frame_auto(samples, prev_data_llrs) -> (packet, stats, mode)
where stats is the FixedRxStats the receiver filled in (header, snr_db,
cfo_hz, harq_combined, start_sample) and mode auto-detection works exactly
like the float demod_frame_auto: try each link mode's receiver in turn --
only the transmitted mode's ZC preamble root locks."""

import numpy as np

from ..modes import LinkMode
from ..transceiver import DemodError
from .tx import FixedTransmitter
from .rx import FixedReceiver


class FixedPHY:
    def __init__(self, calibrate: bool = True):
        # calibrate=True mirrors the float station's llr_recal="auto": the
        # value is a stable cross-frame LLR scale (legitimate HARQ sums),
        # not extra PER.
        self.calibrate = calibrate
        self._tx: dict = {}
        self._rx: dict = {}

    def _get_tx(self, mode: LinkMode) -> FixedTransmitter:
        if mode not in self._tx:
            self._tx[mode] = FixedTransmitter(mode)
        return self._tx[mode]

    def _get_rx(self, mode: LinkMode) -> FixedReceiver:
        if mode not in self._rx:
            self._rx[mode] = FixedReceiver(mode, calibrate=self.calibrate)
        return self._rx[mode]

    def build_frame(self, pkt, mode: LinkMode, mod, spd) -> np.ndarray:
        sig16 = self._get_tx(mode).build_frame(pkt, mod=mod, spd=spd)
        return sig16.astype(np.float64) / 32768.0

    def demod_frame_auto(self, real_signal: np.ndarray, prev_data_llrs=None):
        x = np.asarray(real_signal, dtype=np.float64)
        peak = float(np.max(np.abs(x))) if len(x) else 0.0
        if peak <= 0.0:
            raise DemodError("no signal")
        x16 = np.clip(x / peak * 0.9 * 32767, -32768, 32767).astype(np.int16)

        last_exc = None
        for mode in LinkMode:
            rx = self._get_rx(mode)
            try:
                pkt, *_ = rx.receive(x16, prev_data_llrs=prev_data_llrs)
                return pkt, rx.last_stats, mode
            except DemodError as exc:
                if exc.data_llrs is not None or last_exc is None \
                        or last_exc.data_llrs is None:
                    last_exc = exc
        raise last_exc if last_exc is not None else DemodError("no preamble")
