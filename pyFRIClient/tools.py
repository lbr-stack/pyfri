import abc

import numpy as np


#
# Estimate wrench at a given tip link.
#


class WrenchEstimator(abc.ABC):

    """

    WrenchEstimator
    ===============

    Sub-classes of the WrenchEstimator can be used to estimate the
    external wrench applied at a given link on the robot, i.e. the tip
    link. These methods map the measured external joint torques to the
    given task space. An offset is applied to remove bias in the
    measurements. How that offset is estimated is implemented by the
    respective sub-classes.

    Additionally, a smoothing exponential filter can be applied (by
    default the filter is not applied).

    """

    _rcond = 0.05  # Cutoff for small singular values in pseudo-inverse calculation.

    def __init__(
        self,
        client,
        robot_model,            
        tip_link,
        base_link=None,
        n_data=100,
        smooth=1.0,
    ):
        self._client = client
        self._n_data = n_data
        self._smooth = smooth
        if base_link is None:
            self._jacobian = robot_model.get_global_link_geometric_jacobian_function(
                tip_link,
                numpy_output=True,
            )
        elif isinstance(base_link, str):
            self._jacobian = robot_model.get_link_geometric_jacobian_function(
                tip_link,
                base_link,
                numpy_output=True,
            )
        else:
            raise ValueError(f"{base_link=} is not recognized.")
        self._data = []
        self._offset = None
        self._wrench_estimate = np.zeros(6)

    def _q(self):
        return self._client.robotState().getMeasuredJointPosition().flatten()

    def _tau_ext(self):
        return self._client.robotState().getExternalTorque().flatten()

    def _inverse_jacobian(self):
        return np.linalg.pinv(self._jacobian(self._q()), rcond=self._rcond)

    def ready(self):
        return len(self._data) >= self._n_data

    def update(self):
        n_data = len(self._data)
        if n_data < self._n_data:
            self._update_data()
        elif (n_data == self._n_data) and (self._offset is None):
            self._offset = np.mean(self._data, axis=0)

    @abc.abstractmethod
    def _update_data(self):
        pass

    @abc.abstractmethod
    def _get_wrench_estimate(self):
        pass

    def get_wrench_estimate(self):
        # Get current wrench estimate
        wrench_estimate = self._get_wrench_estimate()

        # Apply exponential filter to wrench estimate
        self._wrench_estimate = (
            self._smooth * wrench_estimate
            + (1.0 - self._smooth) * self._wrench_estimate
        )
        return self._wrench_estimate.copy()


class WrenchEstimatorJointOffset(WrenchEstimator):

    """

    WrenchEstimatorJointOffset
    ==========================

    The offset is computed in the joint space and applied prior to the
    wrench being estimated.

    """

    def _update_data(self):
        self._data.append(self._tau_ext().tolist())

    def _get_wrench_estimate(self):
        tau_ext = self._tau_ext() - self._offset
        Jinv = self._inverse_jacobian()
        return Jinv.T @ tau_ext


class WrenchEstimatorTaskOffset(WrenchEstimator):

    """

    WrenchEstimatorTaskOffset
    =========================

    The offset is computed in the task space and applied after the raw
    joint torque values are projected to the task space.

    """

    def _update_data(self):
        f_ext = self._inverse_jacobian().T @ self._tau_ext()
        self._data.append(f_ext.flatten().tolist())

    def _get_wrench_estimate(self):
        return self._inverse_jacobian().T @ self._tau_ext() - self._offset
