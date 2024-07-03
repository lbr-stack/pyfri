import abc
import numpy as np
from collections import deque


class StateFilter(abc.ABC):
    """
    Initializes a window of fixed size and appends elements to it using the
    `append()` method. It then filters the elements in the window using an abstract
    method.

    Attributes:
        _window_size (int): Used to control the size of the sliding window used
            for filtering.
        reset (instance): Used to reset the internal buffer of the object to its
            initial state by clearing it of all elements.

    """
    def __init__(self, window_size):
        """
        Sets the `window_size` attribute of an instance of `StateFilter`, which
        is used to control the size of the moving window for filtering states. The
        `reset()` method is also defined within the function, which resets the
        internal state of the filter.

        Args:
            window_size (int): Used to set the size of the sliding window used for
                computing moving averages.

        """
        self._window_size = window_size
        self.reset()

    def reset(self):
        self._window = deque([], maxlen=self._window_size)

    def append(self, x):
        self._window.append(x.tolist())

    @abc.abstractmethod
    def filter(self, x):
        """
        In the `StateFilter` class inherited from `abc.ABC`, does not perform any
        operation as it is marked as `@abc.abstractmethod`.

        Args:
            x (abcobject): Used to represent any object that can be passed through
                an abstraction method.

        """
        pass


class ExponentialStateFilter(StateFilter):
    """
    Filters time-series data by taking an exponentially smoothed average of the
    previous values, with a smoothing parameter to control the degree of smoothing.

    Attributes:
        _smooth (float): Set to a value of 0.02 during initialization, which
            represents the smoothing parameter used to blend the input values with
            the previous values in the window.

    """
    def __init__(self, smooth=0.02):
        """
        Sets up an instance of the `ExponentialStateFilter` class by initializing
        its internal variable `smooth` to a value passed as an argument, and then
        calls the parent class's `__init__` method to initialize the filter with
        a default value of 1.

        Args:
            smooth (float): Set to `0.02`. Its value influences the initial position
                of the random walk.

        """
        super().__init__(1)
        self._smooth = smooth

    def filter(self, x):
        """
        Takes an input `x` and applies an exponential smoothing filter to it,
        appending the smoothed value to a internal list. The filter uses a parameter
        `smooth` to control the degree of smoothing.

        Args:
            x (object): Used to represent an input value that is to be filtered
                using a smoothing window.

        Returns:
            nparray: A smoothed version of the input value `x`, created by multiplying
            it with a smoothing factor and adding the initial value of the window
            array.

        """
        if len(self._window) == 0:
            self.append(x)
        xp = np.array(self._window[0])
        xf = self._smooth * x + (1.0 - self._smooth) * xp
        self.append(xf)
        return xf


class MovingAverageFilter(StateFilter):
    """
    Computes and returns the moving average of a sequence of values using the
    provided window size.

    """
    def filter(self, x):
        """
        Appends an input value to a internal buffer and calculates the mean of the
        buffer along the axis 0.

        Args:
            x (object): Passed to the method for filtering.

        Returns:
            npmean: A measure of central tendency of a set of numbers.

        """
        self.append(x)
        return np.mean(self._window, axis=0)
