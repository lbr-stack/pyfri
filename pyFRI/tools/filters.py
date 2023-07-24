import abc
import numpy as np
from scipy import signal
from collections import deque
from pyFRI import LBRState


class JointStateFilter(abc.ABC):
    def __init__(self):
        self._x = np.zeros(LBRState.NUMBER_OF_JOINTS)

    def set_initial(self, x):
        self._x = x.copy()

    @abc.abstractmethod
    def filter(self, x):
        pass


class ExponentialJointStateFilter(JointStateFilter):
    def __init__(self, smooth=0.02):
        super().__init__()
        self._smooth = smooth

    def filter(self, x):
        self._x = self._smooth * x + (1.0 - self._smooth) * self._x
        return self._x.copy()


class MovingAverageFilter(JointStateFilter):
    def __init__(self, N):
        self._data = deque([], maxlen=N)

    def filter(self, x):
        self._data.append(x.tolist())
        return np.mean(self._data, axis=0)


class ButterworthFilter(JointStateFilter):
    def __init__(self, N, Wn, Ndata):
        self._b, self._a = signal.butter(N, Wn)
        self._zi = signal.lfilter_zi(self._b, self._a)
        self._data = deque([], maxlen=Ndata)

    def _filter_joint_axis(self, j):
        data = np.array(self._data)
        y = signal.filtfilt(self._b, self._a, data[:, j])
        return y[-1]

    def filter(self, x):
        self._data.append(x.tolist())
        for i in range(LBRState.NUMBER_OF_JOINTS):
            x[i] = self._filter_joint_axis(i)
        return x
