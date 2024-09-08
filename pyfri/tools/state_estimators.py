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
        """
        Intercepts and modifies the client's command function to update a moving
        window of queue data before executing the original command, thereby updating
        the estimator's internal state.

        Args:
            client (object | ClientType): Assigned to the instance variable `_client`.

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
            Executes an original command after updating the window state. It takes
            variable arguments and keyword arguments from the caller, delegates
            them to the original `orig_command`, and handles any changes that may
            require a window update.

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
        Updates a window of joint states and their derivatives, using data from a
        robot state client. It calculates and stores joint velocities (dq) and
        accelerations (ddq) based on consecutive measured positions.

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
        Initializes its attributes by setting up various link and Jacobian functions
        for a robot model based on given inputs. It uses these to prepare a task
        space state estimator.

        Args:
            client (object | ClientType): Assigned to self._client, indicating
                that it is likely an external interface or connection object that
                enables interactions with an external system.
            joint_state_estimator (object): Referenced by its instance or class,
                indicating it's an estimator used to calculate joint states of a
                robot arm from sensor data.
            robot_model (object): Expected to provide methods for calculating
                transformation functions (`get_global_link_transform_function`,
                `get_link_transform_function`) and geometric Jacobian functions
                (`get_global_link_geometric_jacobian_function`, `get_link_geometric_jacobian_function`).
            ee_link (str | int): Required, representing the end-effector link of
                the robot model. It identifies the specific link from which
                transformations and Jacobians are obtained.
            base_link (str | NoneType): Used to specify the link with respect to
                which forward kinematics and Jacobian transformations are calculated.
                It defaults to None if not provided.

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
        Computes the transformation matrix given the current joint positions, which
        are obtained from the _joint_state_estimator object using its get_position
        method. The computed transformation is then returned.

        Returns:
            numpyndarray: 4x4 homogenous transformation matrix T, calculated from
            joint state q using self._T(q). This transform represents position and
            orientation of a rigid body in 3D space.

        """
        q = self._joint_state_estimator.get_position()
        return self._T(q)

    def get_velocity(self):
        """
        Computes and returns the velocity of an object in task space by taking the
        Jacobian matrix `J` and multiplying it with the joint velocities `dq`,
        effectively transforming joint velocities to task-space velocities.

        Returns:
            numpyndarray: The velocity of the end effector expressed in world coordinates.

        """
        q = self._joint_state_estimator.get_position()
        dq = self._joint_state_estimator.get_velocity()
        J = self._J(q)
        return J @ dq

    def get_acceleration(self):
        """
        Computes the acceleration of the end-effector in task space by taking the
        difference between consecutive velocity samples, dividing it by the sample
        time, and returning the result.

        Returns:
            numpyndarray: 6-element acceleration of a robotic arm expressed as
            [ax, ay, az, wx, wy, wz].

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
    Defines an abstract base class for estimating external torques. It has a single
    abstract method, `get_external_torque`, which must be implemented by any
    concrete subclass to return an estimate of the external torque acting on a
    system or object.

    """
    @abc.abstractmethod
    def get_external_torque(self):
        """
        Estimates or calculates an external torque value, likely associated with
        an object's movement or rotation. It is abstract and intended to be
        implemented by concrete subclass implementations.

        """
        pass


class FRIExternalTorqueEstimator(ExternalTorqueEstimator):
    """
    Estimates external torques applied to a robot by retrieving and flattening the
    external torque data from a client. It serves as an estimator for external
    forces acting on a robot, enabling calculation of system dynamics without
    explicit force sensing.

    Attributes:
        _client (RobotStateClient|RobotClient): Referenced from outside this class
            through an external dependency. It provides access to a robot's state,
            including its external torque.

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
        Initializes its attributes, setting up estimators for joint states and
        external torques, computing a Jacobian matrix based on robot model parameters,
        and allocating storage for data. It raises an error if base_link is not recognized.

        Args:
            client (object): Required to be provided by the user when instantiating
                this class, but its purpose or usage within the class is not described.
            joint_state_estimator (object | JointStateEstimator |
                AnyOtherJointStateEstimatorType): Used to estimate the state of
                robot joints. Its actual implementation and parameters depend on
                the provided estimator.
            external_torque_estimator (object): Referenced as an attribute
                `_external_torque_estimator`. It represents an estimator for
                external torques acting on the robot. Its exact functionality
                depends on its implementation.
            robot_model (object): Used to interact with a robot model, accessing
                functions related to geometric Jacobians of the robot's links.
            tip_link (str | int): Used to specify the tip link (end effector) of
                the robot model.
            base_link (None | str): Optional, with a default value of None. It
                specifies the base link of a robotic arm to which Jacobian
                computations are referenced, or its name if known.
            n_data (int): 50 by default. It represents the initial size of an
                internal data structure (`self._data`) that will hold collected
                data. Its value can be changed at initialization.

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
        Computes an approximate pseudoinverse of the Jacobian matrix associated
        with the joint state estimator at a given position, based on NumPy's
        linalg.pinv function and specified conditioning parameter rcond.

        """
        q = self._joint_state_estimator.get_position()
        return np.linalg.pinv(self._jacobian(q), rcond=self._rcond)

    def ready(self):
        return len(self._data) >= self._n_data

    def update(self):
        """
        Checks if the data length is less than the specified n_data threshold. If
        true, it calls the _update_data method to update the data. This indicates
        a sequential or iterative process where data needs to be replenished periodically.

        """
        if len(self._data) < self._n_data:
            self._update_data()

    @abc.abstractmethod
    def _update_data(self):
        """
        Defines an abstract function that must be implemented by any subclass of
        WrenchEstimator, ensuring it provides functionality to update data. The
        underscore prefix indicates it is intended for internal use within the
        class and should not be overridden directly.

        """
        pass

    @abc.abstractmethod
    def get_wrench(self):
        """
        Returns a wrench object.

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
        Updates internal data by appending an estimated external torque to a list,
        converting it from a NumPy array to a list using `tolist()`.

        """
        tau_ext = self._external_torque_estimator.get_external_torque()
        self._data.append(tau_ext.tolist())

    def get_wrench(self):
        """
        Calculates the wrench, an external force and torque applied to a robot
        joint, by subtracting an offset from estimated external torque and
        transforming it using inverse jacobian.

        Returns:
            numpyndarray: A wrench, calculated as the transpose of the inverse
            Jacobian multiplied by the external torque minus an offset. The result
            represents the equivalent force at each joint.

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
        Updates internal data by computing the external wrench and appending it
        to a list, using an external torque estimator and inverse jacobian matrix
        for computation.

        """
        tau_ext = self._external_torque_estimator.get_external_torque()
        f_ext = self._inverse_jacobian().T @ tau_ext
        self._data.append(f_ext.flatten().tolist())

    def get_wrench(self):
        """
        Computes an estimated wrench by subtracting an offset from the product of
        the inverse jacobian and external torque estimate. This result is derived
        using NumPy operations on data stored within the instance.

        Returns:
            ndarray: A vector representing an external force and moment exerted
            on a rigid body, calculated based on estimated external torque, inverse
            jacobian, and data mean offset.

        """
        offset = np.mean(self._data, axis=0)
        tau_ext = self._external_torque_estimator.get_external_torque()
        return self._inverse_jacobian().T @ tau_ext - offset
