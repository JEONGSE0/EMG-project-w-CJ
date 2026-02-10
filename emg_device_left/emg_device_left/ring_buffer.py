import numpy as np
from emg_device_left.config import WIN_SAMPLES, SCOPE_SAMPLES, N_CH


class RingBuffer:
    """
    Ring buffer for real-time EMG streaming.

    - Maintains:
        1) inference window buffer (WIN_SAMPLES x N_CH)
        2) scope buffer for visualization (SCOPE_SAMPLES x N_CH)
    - Time-aware: stores timestamps aligned with samples
    """

    def __init__(self):
        self.reset()

    def reset(self):
        # inference window
        self._win_x = np.zeros((WIN_SAMPLES, N_CH), dtype=np.float64)
        self._win_t = np.zeros((WIN_SAMPLES,), dtype=np.float64)
        self._win_filled = 0

        # scope buffer
        self._scope_x = np.zeros((SCOPE_SAMPLES, N_CH), dtype=np.float64)
        self._scope_t = np.zeros((SCOPE_SAMPLES,), dtype=np.float64)
        self._scope_filled = 0

    # -------------------------------------------------
    # push new samples
    # -------------------------------------------------
    def push(self, x: np.ndarray, t: np.ndarray):
        """
        x: (N, N_CH)
        t: (N,)
        """
        if x is None or len(x) == 0:
            return

        x = np.asarray(x)
        t = np.asarray(t)

        n = len(x)

        # ---- inference window (ring)
        if n >= WIN_SAMPLES:
            self._win_x[:] = x[-WIN_SAMPLES:]
            self._win_t[:] = t[-WIN_SAMPLES:]
            self._win_filled = WIN_SAMPLES
        else:
            self._win_x = np.roll(self._win_x, -n, axis=0)
            self._win_t = np.roll(self._win_t, -n, axis=0)
            self._win_x[-n:] = x
            self._win_t[-n:] = t
            self._win_filled = min(WIN_SAMPLES, self._win_filled + n)

        # ---- scope buffer (ring)
        if n >= SCOPE_SAMPLES:
            self._scope_x[:] = x[-SCOPE_SAMPLES:]
            self._scope_t[:] = t[-SCOPE_SAMPLES:]
            self._scope_filled = SCOPE_SAMPLES
        else:
            self._scope_x = np.roll(self._scope_x, -n, axis=0)
            self._scope_t = np.roll(self._scope_t, -n, axis=0)
            self._scope_x[-n:] = x
            self._scope_t[-n:] = t
            self._scope_filled = min(SCOPE_SAMPLES, self._scope_filled + n)

    # -------------------------------------------------
    # properties / getters
    # -------------------------------------------------
    @property
    def ready(self) -> bool:
        """True if inference window is fully filled"""
        return self._win_filled >= WIN_SAMPLES

    def get_window(self):
        """
        Returns:
            win_x: (WIN_SAMPLES, N_CH)
            win_t: (WIN_SAMPLES,)
        """
        if self._win_filled < WIN_SAMPLES:
            return None, None
        return self._win_x.copy(), self._win_t.copy()

    def get_scope(self):
        """
        Returns:
            scope_x: (M, N_CH)
            scope_t: (M,)
        """
        if self._scope_filled == 0:
            return None, None

        return (
            self._scope_x[-self._scope_filled:].copy(),
            self._scope_t[-self._scope_filled:].copy()
        )
