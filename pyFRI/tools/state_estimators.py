import abc
import numpy as np

#
# Joint state estimator
#


class JointStateEstimator:
    def __init__(
        self,
        client,
    ):
        # Set class attributes/variables
        self._client = client
        self._q = None
        self._qp = None
        self._qpp = None
        self._dt = None
        self._dq = None
        self._ddq = None

        # Monkey patch update_window into command method
        orig_command = self._client.command

        def command(*args, **kwargs):
            self._update_window()
            orig_command(*args, **kwargs)

        self._client.command = command

    def _update_window(self):
        self._dt = self._client.robotState().getSampleTime()
        q = self._client.robotState().getMeasuredJointPosition()

        if self._qp is None:
            self._qp = q.copy()
            if self._filter_q is not None:
                self._filter_q.set_initial(q)

        if self._qpp is None:
            self._qpp = q.copy()

        self._qpp = self._qp.copy()
        self._qp = self._q.copy()
        self._q = q.copy()

        dqp = (self._qp - self._qpp) / self._dt
        self._dq = (self._q - self._qp) / self._dt
        self._ddq = (self._dq - dqp) / self._dt

    def get_position(self):
        return self._q.copy()

    def get_velocity(self):
        return self._dq.copy()

    def get_acceleration(self):
        return self._ddq.copy()


class TaskSpaceStateEstimator:
    def __init__(
        self, joint_space_state_estimator, robot_model, ee_link, base_link=None
    ):
        self._joint_space_state_estimator = joint_space_state_estimator

        # Retrieve kinematics models function
        if base_link is None:
            self._T = robot_model.get_global_link_transform_function(
                ee_link, numpy_output=True
            )
            self._J = robot_model.get_global_link_geometric_jacobian_function(
                ee_link, numpy_output=True
            )
        elif isinstance(base_link, str):
            self._T = robot_model.get_link_transform_function(
                ee_link, base_link, numpy_output=True
            )
            self._J = robot_model.get_link_geometric_jacobian_function(
                ee_link, base_link, numpy_output=True
            )
        else:
            raise ValueError(f"{base_link=} was not recognized")

    def get_end_effector_transform(self):
        q = self._joint_space_state_estimator.get_joint_position()
        return self._T(q)

    def get_end_effector_velocity(self):
        q = self._joint_space_state_estimator.get_joint_position()
        dq = self._joint_space_state_estimator.get_joint_velocity()
        J = self._J(q)
        return J @ dq


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
        tau_ext = self._tau_ext() - np.mean(self._data, axis=0)
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
        return self._inverse_jacobian().T @ self._tau_ext() - np.mean(
            self._data, axis=0
        )
