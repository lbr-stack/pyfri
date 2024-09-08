import abc
import numpy as np
from pyfri import LBRState
from collections import deque


#
# Joint state estimator
#


class JointStateEstimator:
    """

    JointStateEstimator
    ===================

    The JointStateEstimator class keeps track of the joint position,
    velocity, and acceleration using the finite difference method. The
    joint velocities and accelerations are estimated using a window of
    the previous three joint positions.

    """

    n_window = 3

    def __init__(
        self,
        client,
    ):
        # Set class attributes/variables
        self._client = client
        self._first_update = True
        self._q = deque([], maxlen=self.n_window)
        self._dq = deque([], maxlen=self.n_window - 1)
        self._ddq = deque([], maxlen=self.n_window - 2)

        # Monkey patch update_window into command method
        orig_command = self._client.command

        def command(*args, **kwargs):
            self._update_window()
            orig_command(*args, **kwargs)

        self._client.command = command

    def q(self, idx):
        return np.array(self._q[idx])

    def dq(self, idx):
        return np.array(self._dq[idx])

    def ddq(self, idx):
        return np.array(self._ddq[idx])

    def _update_window(self):
        # Retrieve state
        dt = self._client.robotState().getSampleTime()
        q = self._client.robotState().getMeasuredJointPosition().flatten().tolist()

        # Update window
        self._q.append(q)
        if self._first_update:
            for _ in range(self.n_window):
                self._q.append(q)
                self._dq.append([0.0] * LBRState.NUMBER_OF_JOINTS)
                self._ddq.append([0.0] * LBRState.NUMBER_OF_JOINTS)
            self._first_update = False

        dq = (self.q(-1) - self.q(-2)) / dt
        self._dq.append(dq.tolist())

        ddq = (self.dq(-1) - self.dq(-2)) / dt
        self._ddq.append(ddq.tolist())

    def get_position(self):
        return self.q(-1)

    def get_velocity(self):
        return self.dq(-1)

    def get_acceleration(self):
        return self.ddq(-1)


class TaskSpaceStateEstimator:
    """

    TaskSpaceStateEstimator
    =======================

    The TaskSpaceStateEstimator class allows you to estimate a given
    end-effector transform (position and orientation), velocity, and
    acceleration.

    """

    def __init__(
        self, client, joint_state_estimator, robot_model, ee_link, base_link=None
    ):
        self._client = client
        self._joint_state_estimator = joint_state_estimator

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

    def get_transform(self):
        q = self._joint_state_estimator.get_position()
        return self._T(q)

    def get_velocity(self):
        q = self._joint_state_estimator.get_position()
        dq = self._joint_state_estimator.get_velocity()
        J = self._J(q)
        return J @ dq

    def get_acceleration(self):
        # Retreive joint states
        q = self._joint_state_estimator.q(-1)
        qp = self._joint_state_estimator.q(-2)
        dq = self._joint_state_estimator.dq(-1)
        dqp = self._joint_state_estimator.dq(-2)

        # Compute end-effector current and previous velocity
        v = self._J(q) @ dq
        vp = self._J(qp) @ dqp

        # Compute and return end-effector acceleration
        dt = self._client.robotState().getSampleTime()
        return (v - vp) / dt


class ExternalTorqueEstimator(abc.ABC):
    @abc.abstractmethod
    def get_external_torque(self):
        pass


class FRIExternalTorqueEstimator(ExternalTorqueEstimator):
    def __init__(self, client):
        self._client = client

    def get_external_torque(self):
        return self._client.robotState().getExternalTorque().flatten()


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

    """

    _rcond = 0.05  # Cutoff for small singular values in pseudo-inverse calculation.

    def __init__(
        self,
        client,
        joint_state_estimator,
        external_torque_estimator,
        robot_model,
        tip_link,
        base_link=None,
        n_data=50,
    ):
        # Create class attributes
        self._joint_state_estimator = joint_state_estimator
        self._external_torque_estimator = external_torque_estimator

        # Setup jacobian function
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

        # Setup data collector
        self._n_data = n_data
        self._data = []

    def _inverse_jacobian(self):
        q = self._joint_state_estimator.get_position()
        return np.linalg.pinv(self._jacobian(q), rcond=self._rcond)

    def ready(self):
        return len(self._data) >= self._n_data

    def update(self):
        if len(self._data) < self._n_data:
            self._update_data()

    @abc.abstractmethod
    def _update_data(self):
        pass

    @abc.abstractmethod
    def get_wrench(self):
        pass


class WrenchEstimatorJointOffset(WrenchEstimator):
    """

    WrenchEstimatorJointOffset
    ==========================

    The offset is computed in the joint space and applied prior to the
    wrench being estimated.

    """

    def _update_data(self):
        tau_ext = self._external_torque_estimator.get_external_torque()
        self._data.append(tau_ext.tolist())

    def get_wrench(self):
        offset = np.mean(self._data, axis=0)
        tau_ext = self._external_torque_estimator.get_external_torque() - offset
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
        tau_ext = self._external_torque_estimator.get_external_torque()
        f_ext = self._inverse_jacobian().T @ tau_ext
        self._data.append(f_ext.flatten().tolist())

    def get_wrench(self):
        offset = np.mean(self._data, axis=0)
        tau_ext = self._external_torque_estimator.get_external_torque()
        return self._inverse_jacobian().T @ tau_ext - offset
