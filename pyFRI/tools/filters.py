import abc
import numpy as np
from collections import deque


class StateFilter(abc.ABC):
    def __init__(self, window_size):
        self._window_size = window_size
        self.reset()

    def reset(self):
        self._window = deque([], maxlen=self._window_size)

    def append(self, x):
        self._window.append(x.tolist())

    @abc.abstractmethod
    def filter(self, x):
        pass


class ExponentialStateFilter(StateFilter):
    def __init__(self, smooth=0.02):
        super().__init__(1)
        self._smooth = smooth

    def filter(self, x):
        if len(self._window) == 0:
            self.append(x)
        xp = np.array(self._window[0])
        xf = self._smooth * x + (1.0 - self._smooth) * xp
        self.append(xf)
        return xf


class MovingAverageFilter(StateFilter):
    def filter(self, x):
        self.append(x)
        return np.mean(self._window, axis=0)
