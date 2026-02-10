"""
blesource.py

Production-ready BLE EMG streaming source designed to plug into your existing StreamWorker:

    source = BleSource(client, emg_char_uuid=..., fs=FS)
    await source.start()
    x, t, eof = source.poll()

Key properties:
- No labels (classification-only stream).
- Thread-safe queue for cross-thread use (Bleak notify callback runs on asyncio loop; StreamWorker polls in QThread).
- Parses ADS1299-style packed 24-bit samples: 8 channels × 3 bytes = 24 bytes per sample.
- Supports multiple samples per notification (payload length is a multiple of 24).
- Timestamps are generated deterministically from sample index and fs (t = t0 + idx/fs).
- Handles disconnects: eof=True once a disconnect is detected (or stop() called).
"""

from __future__ import annotations

import time
import threading
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import numpy as np
from bleak import BleakClient


@dataclass
class BleSourceStats:
    """Lightweight stats for debugging/monitoring."""
    notifications: int = 0
    samples_parsed: int = 0
    bytes_received: int = 0
    last_notify_time: float = 0.0
    disconnected: bool = False
    parse_errors: int = 0


class BleSource:
    """
    BLE streaming source providing (x, t, eof) via poll().

    - x: np.ndarray, shape (N, 8), dtype float32 (uV by default)
    - t: np.ndarray, shape (N,), dtype float64 (seconds)
    - eof: bool, True if disconnected or stopped

    Notes:
    - poll() is synchronous and thread-safe.
    - start()/stop() are async because Bleak uses asyncio.
    """

    def __init__(
        self,
        client: BleakClient,
        emg_char_uuid: str,
        fs: int,
        *,
        vref: float = 4.5,
        gain: float = 24.0,
        channels: int = 8,
        endian: str = "big",
        output_unit: str = "uV",  # "uV" or "raw"
        max_queue_samples: int = 50000,
    ):
        if channels != 8:
            raise ValueError("This implementation expects 8 channels (ADS1299-style).")

        if endian not in ("big", "little"):
            raise ValueError("endian must be 'big' or 'little'.")

        if output_unit not in ("uV", "raw"):
            raise ValueError("output_unit must be 'uV' or 'raw'.")

        self.client = client
        self.emg_char_uuid = emg_char_uuid
        self.fs = int(fs)

        self.vref = float(vref)
        self.gain = float(gain)
        self.channels = int(channels)
        self.endian = endian
        self.output_unit = output_unit

        # Keep your previous convention for exact continuity with your parser:
        self._lsb_volt = (2.0 * self.vref) / (self.gain * (2**24 - 1))

        # Thread-safe sample queues (sample-major)
        self._qx: Deque[np.ndarray] = deque()
        self._qt: Deque[float] = deque()
        self._lock = threading.Lock()

        # Control/state
        self._running = False
        self._stopped = False
        self._eof = False
        self._sample_idx = 0
        self._t0 = 0.0

        self.stats = BleSourceStats()

        # Bound memory if GUI stalls
        self._max_queue_samples = int(max_queue_samples)

    async def start(self) -> None:
        """Start notifications. Call after client.connect()."""
        if self._running:
            return
        if not self.client or not getattr(self.client, "is_connected", False):
            raise RuntimeError("BleSource.start() requires an already-connected BleakClient.")

        self._running = True
        self._stopped = False
        self._eof = False
        self.stats = BleSourceStats(disconnected=False)
        self._sample_idx = 0
        self._t0 = time.time()

        # Optional: attach disconnect callback if available
        try:
            self.client.set_disconnected_callback(self._on_disconnect)
        except Exception:
            pass

        async def _handler(_: int, data: bytearray):
            try:
                self.stats.notifications += 1
                self.stats.bytes_received += len(data)
                self.stats.last_notify_time = time.time()

                xs = self._parse_emg_bundle(bytes(data))
                if xs is None or xs.size == 0:
                    return

                n = xs.shape[0]
                t = self._t0 + (np.arange(self._sample_idx, self._sample_idx + n, dtype=np.float64) / self.fs)
                self._sample_idx += n

                with self._lock:
                    while len(self._qt) + n > self._max_queue_samples and self._qt:
                        self._qt.popleft()
                        self._qx.popleft()

                    for i in range(n):
                        self._qt.append(float(t[i]))
                        self._qx.append(xs[i])

                self.stats.samples_parsed += n

            except Exception:
                self.stats.parse_errors += 1

        await self.client.start_notify(self.emg_char_uuid, _handler)

    async def stop(self) -> None:
        """Stop notifications and mark eof. Does not disconnect client by default."""
        if self._stopped:
            return
        self._stopped = True
        self._running = False
        self._eof = True

        try:
            if self.client and getattr(self.client, "is_connected", False):
                await self.client.stop_notify(self.emg_char_uuid)
        except Exception:
            pass

    def poll(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], bool]:
        """
        Non-async poll for StreamWorker loop.

        Returns:
            x: (N,8) float32, or None if no data
            t: (N,) float64, or None if no data
            eof: True if disconnected/stopped
        """
        if self._eof:
            return None, None, True

        with self._lock:
            if not self._qt:
                if self.client and hasattr(self.client, "is_connected") and not self.client.is_connected:
                    self._eof = True
                    self.stats.disconnected = True
                    return None, None, True
                return None, None, False

            n = len(self._qt)
            t = np.fromiter((self._qt.popleft() for _ in range(n)), dtype=np.float64, count=n)
            x = np.vstack([self._qx.popleft() for _ in range(n)]).astype(np.float32, copy=False)

        return x, t, False

    def _on_disconnect(self, _client: BleakClient) -> None:
        self._eof = True
        self._running = False
        self.stats.disconnected = True

    def _parse_emg_bundle(self, data: bytes) -> np.ndarray:
        """Parse payload to (N,8). Format: repeated frames of 24 bytes (8ch × 3 bytes)."""
        if not data:
            return np.empty((0, self.channels), dtype=np.float32)

        frame_bytes = self.channels * 3  # 24
        n_frames = len(data) // frame_bytes
        if n_frames <= 0:
            return np.empty((0, self.channels), dtype=np.float32)

        data = data[: n_frames * frame_bytes]  # drop trailing partial

        out = np.empty((n_frames, self.channels), dtype=np.float32)
        for i in range(n_frames):
            base = i * frame_bytes
            for ch in range(self.channels):
                raw3 = data[base + ch * 3 : base + ch * 3 + 3]
                val = int.from_bytes(raw3, byteorder=self.endian, signed=False)
                if val & 0x800000:
                    val -= (1 << 24)

                if self.output_unit == "raw":
                    out[i, ch] = float(val)
                else:
                    out[i, ch] = float(val * self._lsb_volt * 1e6)  # uV

        return out
