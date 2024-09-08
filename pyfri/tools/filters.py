import abc
import numpy as np
from collections import deque


class StateFilter(abc.ABC):
    """
    Initializes a window of fixed size for state tracking. It  resets the window
    on initialization and when reset() is called. The `append()` method adds data
    to the window, and the `filter()` method, which must be implemented by subclasses,
    processes the data in the window.

    Attributes:
        _window_size (int): Set to window_size during instantiation. It determines
            the maximum number of elements that can be stored in the _window deque,
            which implements a bounded-length queue data structure with a specified
            maxlen.
        reset (Callable[[],None]): Defined as a method named reset. It clears the
            internal window when called, preparing it for use with a new sequence
            or dataset. The default implementation resets the _window deque.

    """
    def __init__(self, window_size):
        """
        Initializes an instance with a given window size and calls the `reset`
        method to set the internal state accordingly, suggesting it maintains some
        sort of memory or history based on this size.

        Args:
            window_size (int): Passed to the class constructor. It initializes an
                instance variable `_window_size` which stores this value, determining
                the size of some window or buffer in subsequent operations.

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
        Defines an abstract interface for filtering input values. Implementing
        classes must provide their own concrete filter implementation, while this
        method signature serves as a contract specifying the required filter
        operation. It takes one argument, x.

        Args:
            x (Any): Required to be provided when implementing this abstract method
                in any subclass, but its usage or structure is not specified in
                this declaration.

        """
        pass


class ExponentialStateFilter(StateFilter):
    """
    Applies an exponential smoothing filter to a time series data. It uses a window
    to store previous values, and each new value is calculated as a weighted average
    between the current value and the exponentially smoothed previous value.

    Attributes:
        _smooth (float): Set to a default value of 0.02 during initialization. It
            controls the smoothing effect by determining the proportion of new
            values to be used for filtering.

    """
    def __init__(self, smooth=0.02):
        """
        Initializes its instance variables: it calls the superclass's constructor
        with an argument of 1, and sets the _smooth attribute to a specified value
        by default equal to 0.02.

        Args:
            smooth (float | int): 0.02 by default. It appears to be used for
                smoothing purposes, possibly related to animation or easing effects,
                and its value affects the behavior of the class instance.

        """
        super().__init__(1)
        self._smooth = smooth

    def filter(self, x):
        """
        Calculates an exponentially smoothed value of input x, replacing the oldest
        window element with the new value at each step. It maintains a sliding
        window of values and returns the most recent smoothed value.

        Args:
            x (float | np.ndarray): 1 value representing new input data to be
                processed by the filter, appended to the internal window and
                smoothed according to the specified smoothing coefficient.

        Returns:
            float: A filtered version of input parameter `x`.

        """
        if len(self._window) == 0:
            self.append(x)
        xp = np.array(self._window[0])
        xf = self._smooth * x + (1.0 - self._smooth) * xp
        self.append(xf)
        return xf


class MovingAverageFilter(StateFilter):
    """
    Calculates the moving average of input data. It stores the input values in a
    window and returns their mean at each step, effectively smoothing out noise
    and providing a running average of the data. The axis=0 parameter indicates
    that mean is calculated along columns.

    """
    def filter(self, x):
        """
        Appends new data to an internal window and returns the mean value of all
        elements within that window, computed along each dimension (axis=0).

        Args:
            x (Union[List[float], float, np.ndarray]): Expected to be a single
                data point or multiple data points that are used for filtering.
                The type annotation indicates it can take various input formats.

        Returns:
            numpyndarray: 1-dimensional array representing the mean of a specified
            window of values stored in the object's _window attribute after appending
            new input x.

        """
        self.append(x)
        return np.mean(self._window, axis=0)
