import abc
import numpy as np
from collections import deque


class StateFilter(abc.ABC):
    """
    Provides a base implementation for filtering state data over a fixed window
    size. It  initializes with a specified window size and allows appending new
    data points, maintaining a deque to store the recent values within the defined
    window.

    Attributes:
        _window_size (int): Set during initialization of an instance of this class
            through its constructor (`__init__`). It specifies the maximum number
            of values that a filter window can hold.
        reset (Callable[[None],None]): A method that resets the internal state of
            the filter by clearing its window to the specified size.

    """
    def __init__(self, window_size):
        """
        Initializes an instance with a specified window size and calls the `reset`
        method to set the initial state. It stores the provided window size as an
        attribute `_window_size`.

        Args:
            window_size (int): Mandatory for initialization of the object. It
                represents the size of the window and it determines the scope or
                range of data considered by the object.

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
        Takes one argument, an object x, and returns nothing by default, indicating
        that it must be implemented by any subclass of this abstract base class.
        Its purpose is to filter or modify objects based on specific criteria.

        Args:
            x (Any): Positional-only, indicating it must be passed as an argument
                without any keyword prefix. It represents the input value to be filtered.

        """
        pass


class ExponentialStateFilter(StateFilter):
    """
    Implements an exponential moving average filter, which  calculates a weighted
    average between new and previous values, with a smoothing factor to control
    the rate at which the filtered value adapts to changes in the input.

    Attributes:
        _smooth (float): Initialized with a default value of 0.02. It represents
            the smoothing coefficient for exponential moving average calculations.

    """
    def __init__(self, smooth=0.02):
        """
        Initializes an instance with optional smoothing parameter, defaulting to
        0.02, and calls its superclass's constructor with a fixed value of 1. It
        stores the smoothing parameter in an attribute named `_smooth`.

        Args:
            smooth (float): 0.02 by default. It is used to initialize an instance
                variable `_smooth`.

        """
        super().__init__(1)
        self._smooth = smooth

    def filter(self, x):
        """
        Updates an exponential moving average (EMA) filter with a new data point
        and returns the filtered value. The EMA combines past values using a
        smoothing factor, giving more weight to recent data points.

        Args:
            x (float | numpy.ndarray): A value to be filtered by the exponential
                moving average (EMA) algorithm. It represents the new data point
                to be included in the EMA computation.

        Returns:
            float: A smoothed version of the input `x`, calculated using a weighted
            average between `x` and the oldest element in the `_window` list.

        """
        if len(self._window) == 0:
            self.append(x)
        xp = np.array(self._window[0])
        xf = self._smooth * x + (1.0 - self._smooth) * xp
        self.append(xf)
        return xf


class MovingAverageFilter(StateFilter):
    """
    Filters data by maintaining a window of values and computing their mean at
    each time step. It extends the `StateFilter` class, overriding its `filter`
    method to append new input and return the current window's average as output.

    """
    def filter(self, x):
        """
        Calculates and returns the mean value of the current window's elements
        along axis 0, simultaneously updating the internal state by appending a
        new element to it.

        Args:
            x (array_like): 1-dimensional. It represents a data point or sample
                to be processed by the filter algorithm. The exact structure and
                content depend on the context in which this code snippet is used.

        Returns:
            numpyndarray: The mean of all values currently storedinself._window.
            The axis=0 argument indicates that the mean should be calculated along
            each column, resulting in a one-dimensional array with the same number
            of elements as there are columns in self._window.

        """
        self.append(x)
        return np.mean(self._window, axis=0)
