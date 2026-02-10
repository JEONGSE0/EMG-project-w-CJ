import time
import numpy as np
import pandas as pd
from emg_device.config import FS, N_CH

class FileReplaySource:
    def __init__(self, csv_path, speed=1.0, batch_max=None):
        self.csv_path = csv_path
        self.speed = speed
        self.batch_max = batch_max
        self.df = None
        self.idx = 0
        self.t0 = None

    def start(self):
        self.df = pd.read_csv(self.csv_path)
        self.x = self.df[[f"ch{i}" for i in range(1, N_CH+1)]].values
        self.t = self.df["time"].values if "time" in self.df else np.arange(len(self.df)) / FS
        self.idx = 0
        self.t0 = time.perf_counter()

    def poll(self):
        if self.idx >= len(self.t):
            return None, None, True

        elapsed = (time.perf_counter() - self.t0) * self.speed
        target_time = self.t[0] + elapsed

        start = self.idx
        while self.idx < len(self.t) and self.t[self.idx] <= target_time:
            self.idx += 1
            if self.batch_max is not None and (self.idx - start) >= self.batch_max:
                break

        if self.idx == start:
            return None, None, False

        return self.x[start:self.idx], self.t[start:self.idx], False
