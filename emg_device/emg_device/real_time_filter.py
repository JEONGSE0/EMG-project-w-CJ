# -*- coding: utf-8 -*-
import numpy as np
from scipy import signal
from emg_device.config import FS, N_CH

class RealtimeFilter:

    def __init__(self, fs: float = FS, lowcut: float = 20.0, highcut: float = 220.0,
                 order: int = 4, notch_f0: float = 60.0, notch_q: float = 30.0,
                 max_harmonics: int = 1):
        self.fs = float(fs)
        self.lowcut = float(lowcut)
        self.highcut = float(highcut)
        self.order = int(order)

        self.notch_f0 = float(notch_f0)
        self.notch_q = float(notch_q)
        self.max_harmonics = int(max_harmonics)

        # notch sos list (for harmonics)
        self.sos_notches = []
        for k in range(1, self.max_harmonics + 1):
            f0 = k * self.notch_f0
            if f0 < self.fs / 2:
                b, a = signal.iirnotch(w0=f0, Q=self.notch_q, fs=self.fs)
                sos = signal.tf2sos(b, a)
                self.sos_notches.append(sos)

        # bandpass sos
        self.sos_bp = signal.butter(
            self.order, [self.lowcut, self.highcut],
            btype="bandpass", fs=self.fs, output="sos"
        )

        self.reset()

    def reset(self):
        # zi per section per channel
        self.zi_notches = []
        for sos in self.sos_notches:
            zi = signal.sosfilt_zi(sos)              # (n_sections, 2)
            zi = np.repeat(zi[:, :, None], N_CH, axis=2)  # (n_sections, 2, ch)
            self.zi_notches.append(zi)

        zi_bp = signal.sosfilt_zi(self.sos_bp)
        self.zi_bp = np.repeat(zi_bp[:, :, None], N_CH, axis=2)

    def process(self, x: np.ndarray) -> np.ndarray:
        if x is None or len(x) == 0:
            return x

        filtered = x

        # notch (and harmonics if enabled)
        for i, sos in enumerate(self.sos_notches):
            filtered, self.zi_notches[i] = signal.sosfilt(sos, filtered, axis=0, zi=self.zi_notches[i])

        # bandpass
        filtered, self.zi_bp = signal.sosfilt(self.sos_bp, filtered, axis=0, zi=self.zi_bp)

        return filtered
