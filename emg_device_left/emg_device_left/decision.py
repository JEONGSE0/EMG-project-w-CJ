from dataclasses import dataclass
from collections import deque
import numpy as np
from emg_device_left.config import CONFIRM_N, HOLD_HOPS


@dataclass
class DecisionState:
    raw_pred: int
    raw_conf: float
    final_cmd: int
    confirm_count: int
    hold_remaining_hops: int


class DecisionLayer:
    def __init__(self, confirm_n: int = CONFIRM_N, hold_hops: int = HOLD_HOPS):
        self.confirm_n = int(confirm_n)
        self.hold_hops = int(hold_hops)
        self.reset()

    def reset(self):
        self._recent = deque(maxlen=self.confirm_n)
        self._final_cmd = 0
        self._hold_remaining = 0
        self._confirm_count = 0

    def step(self, proba: np.ndarray) -> DecisionState:
        proba = np.asarray(proba, dtype=np.float64)
        raw_pred = int(np.argmax(proba))
        raw_conf = float(np.max(proba))

        # hold 중이면 final_cmd 유지
        if self._hold_remaining > 0:
            self._hold_remaining -= 1
            return DecisionState(
                raw_pred=raw_pred,
                raw_conf=raw_conf,
                final_cmd=self._final_cmd,
                confirm_count=self._confirm_count,
                hold_remaining_hops=self._hold_remaining
            )

        # hold가 끝난 상태: confirm 누적
        self._recent.append(raw_pred)

        # 최근 동일 예측 길이 계산
        # (deque 안의 뒤에서부터 동일한 값 개수)
        cnt = 0
        for v in reversed(self._recent):
            if v == raw_pred:
                cnt += 1
            else:
                break
        self._confirm_count = cnt

        # confirm 만족하면 final_cmd 확정 + hold 시작
        if len(self._recent) == self.confirm_n and len(set(self._recent)) == 1:
            self._final_cmd = raw_pred
            self._hold_remaining = self.hold_hops
            self._recent.clear()
            self._confirm_count = self.confirm_n

        return DecisionState(
            raw_pred=raw_pred,
            raw_conf=raw_conf,
            final_cmd=self._final_cmd,
            confirm_count=self._confirm_count,
            hold_remaining_hops=self._hold_remaining
        )
