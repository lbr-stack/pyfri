import abc
import numpy as np
from pyFRI import LBRState
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
        """
        Intercepts incoming commands sent by the client, updates the estimator's
        internal window data structures as necessary before executing the original
        command.

        Args:
            client (object): Expected to have an attribute or method named `command`,
                which is replaced with the decorated version in this code.

        """
        # Set class attributes/variables
        self._client = client
        self._first_update = True
        self._q = deque([], maxlen=self.n_window)
        self._dq = deque([], maxlen=self.n_window - 1)
        self._ddq = deque([], maxlen=self.n_window - 2)

        # Monkey patch update_window into command method
        orig_command = self._client.command

        def command(*args, **kwargs):
            """
            Wraps another function called `orig_command`. It updates a window
            before calling `orig_command`, allowing modifications to be made to
            the state of the application before executing the original command.

            Args:
                *args (list): List of positional arguments
                **kwargs (dict): Dictionary of keyword arguments

            """
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
        """
        Accumulates data from robot state messages, estimating joint velocity and
        acceleration by computing differences between consecutive measurements.

        """
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
        """
        Initializes an instance by setting client and joint state estimator
        properties, and computing functions to transform link coordinates and
        calculate the geometric Jacobian based on a robot model and end-effector
        or specified base link.

        Args:
            client (object): Assigned to an instance variable `_client`. No further
                information about its purpose or functionality is provided within
                this code snippet.
            joint_state_estimator (object): Used to estimate joint states likely
                from sensor data or other sources and can be expected to produce
                values which are passed into various functions within the class.
            robot_model (object): Used to access methods related to transforms and
                Jacobians of robot links based on its properties. It provides
                functions to compute global link transform and geometric Jacobian.
            ee_link (str | object): Required as it represents the end effector or
                the link of interest of the robot model.
            base_link (None | str): Optional by default. It specifies the base
                link to which the end-effector's transform and Jacobian are
                referenced, if not specified globally through the robot model.

        """
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
        """
        Retrieves the current position from a joint state estimator and returns a
        transformation matrix calculated from that position using the `_T(q)` function.

        Returns:
            Transform: A representation of the current position of the robot,
            computed from its joint state estimator. It calls another internal
            method `_T(q)` that likely applies some transformation based on the
            input pose `q`.

        """
        q = self._joint_state_estimator.get_position()
        return self._T(q)

    def get_velocity(self):
        """
        Computes the velocity in the task space by taking the product of the
        Jacobian matrix and the joint velocities. This allows for the transformation
        of joint space velocities to task space velocities.

        Returns:
            numpyndarray: A vector representing the velocity of the system,
            calculated as the product of the Jacobian matrix and the joint velocities.
            The returned object has dimensions equal to those of self._J(q).

        """
        q = self._joint_state_estimator.get_position()
        dq = self._joint_state_estimator.get_velocity()
        J = self._J(q)
        return J @ dq

    def get_acceleration(self):
        """
        Calculates the acceleration of an end-effector in a robot's task space by
        estimating joint velocities and accelerations from state estimators,
        differentiating them with respect to time, and computing their difference
        over a sample period.

        Returns:
            ndarray: Estimated acceleration of the robot, computed as a change in
            velocity over time.

        """
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
    """
    Provides an abstract interface for estimating external torques, which are
    forces that act on a system from outside its boundaries. It defines a single
    abstract method, `get_external_torque`, to be implemented by concrete subclasses.

    """
    @abc.abstractmethod
    def get_external_torque(self):
        """
        Returns the estimated external torque value, which is assumed to be
        implemented by concrete subclasses that inherit from this abstract base class.

        """
        pass


class FRIExternalTorqueEstimator(ExternalTorqueEstimator):
    """
    Estimates external torques on a robot based on data from an FRI (Fieldbus
    Remote I/O) client, which interacts with a robotic system to retrieve state
    information, including the estimated external torque experienced by the robot.

    Attributes:
        _client (RobotStateClient|object): Assigned a value in the `__init__`
            method. It represents a client object that interacts with a robot's
            state, used to retrieve external torque information.

    """
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
        """
        Initializes its internal state by setting various attributes, including
        estimators for joint and external torques, a robot model, and Jacobian
        functions, depending on input parameters such as base link and data length.

        Args:
            client (object | ClientType): Required. Its specific role or purpose
                within the class is not explicitly described, suggesting it may
                be a dependency for other functionality.
            joint_state_estimator (object): Not explicitly defined within this
                code snippet. However, it is expected to be an estimator capable
                of providing joint state estimates.
            external_torque_estimator (object): Stored as an attribute named
                `_external_torque_estimator`. Its exact usage depends on its
                implementation, but it likely estimates external torques affecting
                the robot.
            robot_model (object): Used to calculate geometric Jacobians for a robot
                model, likely an instance of a class that provides methods to
                compute kinematic quantities.
            tip_link (str | int): Expected to be a reference to the end-effector
                or last link of the robot arm, which is used by some methods of
                the class.
            base_link (str | NoneType): Used to specify the base link of the
                Jacobian calculation. If not provided, it defaults to None, resulting
                in global Jacobian calculation.
            n_data (int): 50 by default. It determines the number of data points
                to be stored internally for processing.

        """
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
        """
        Computes an approximate inverse of the Jacobian matrix associated with the
        current joint configuration, represented by q. This is achieved through
        the use of NumPy's pinv function with a specified condition number threshold
        (rcond).

        """
        q = self._joint_state_estimator.get_position()
        return np.linalg.pinv(self._jacobian(q), rcond=self._rcond)

    def ready(self):
        return len(self._data) >= self._n_data

    def update(self):
        """
        Checks if the available data is less than the required data threshold
        `_n_data`. If so, it calls the `_update_data` method to fetch or generate
        additional data. This ensures that the estimator has sufficient data before
        proceeding with calculations.

        """
        if len(self._data) < self._n_data:
            self._update_data()

    @abc.abstractmethod
    def _update_data(self):
        """
        Defines an abstract operation that must be implemented by concrete subclasses,
        providing a way to update internal data without being called directly from
        outside the class hierarchy. It is intended for internal use within the
        class or its subclasses.

        """
        pass

    @abc.abstractmethod
    def get_wrench(self):
        """
        Returns an estimate of a wrench, but its specific implementation is not
        provided and must be defined by a subclass, making it an abstract method
        according to ABC protocol.

        """
        pass


class WrenchEstimatorJointOffset(WrenchEstimator):
    """

    WrenchEstimatorJointOffset
    ==========================

    The offset is computed in the joint space and applied prior to the
    wrench being estimated.

    """

    def _update_data(self):
        """
        Updates the internal data list with the estimated external torque calculated
        by the `tau_ext` variable, which is retrieved from an estimator object and
        converted to a list before being appended.

        """
        tau_ext = self._external_torque_estimator.get_external_torque()
        self._data.append(tau_ext.tolist())

    def get_wrench(self):
        """
        Calculates an estimated wrench acting on a joint, accounting for external
        torques and a joint offset. It returns the product of the inverse Jacobian
        matrix transpose and the net external torque.

        Returns:
            numpyndarray: 2D matrix representing a wrench at the end-effector,
            calculated by taking the dot product of Jacobian inverse transpose and
            the external torque minus offset.

        """
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
        """
        Updates internal data with estimated external wrenches, computing the
        wrench as the product of the inverse jacobian and an estimate of the
        external torque. The result is appended to a list as a flat vector.

        """
        tau_ext = self._external_torque_estimator.get_external_torque()
        f_ext = self._inverse_jacobian().T @ tau_ext
        self._data.append(f_ext.flatten().tolist())

    def get_wrench(self):
        """
        Calculates an estimated wrench (force + torque) by applying an inverse
        Jacobian transformation to an external torque estimate and subtracting an
        offset calculated from mean data.

        Returns:
            numpyndarray: A wrench (moment + force) applied by the robot's end
            effector, calculated as the difference between the external torque and
            the offset derived from the data.

        """
        offset = np.mean(self._data, axis=0)
        tau_ext = self._external_torque_estimator.get_external_torque()
        return self._inverse_jacobian().T @ tau_ext - offset
