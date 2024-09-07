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
        Updates the client's command function to call `_update_window` before
        executing the original command, thereby updating the estimator's internal
        window with new data.

        Args:
            client (object | ClientClass): Assigned to instance variable `_client`.
                It appears to represent an external client or API, which is being
                wrapped by the current class for monitoring.

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
            Updates a window before executing an original command. It does this
            by calling `_update_window` and then passing any arguments to the
            `orig_command`. This allows window updates to be performed consistently
            before executing user commands.

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
        Updates internal state variables with new joint position, velocity, and
        acceleration data from a robot state client. It also performs data padding
        for initial window samples and calculates derivatives using finite differences.

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
        Initializes an object by setting up transforms and Jacobians for task space
        estimation based on provided robot model, end-effector link, client, and
        optional base link.

        Args:
            client (object): Required for initialization, but its actual usage or
                purpose is not specified within this code snippet.
            joint_state_estimator (object): Required for the class initialization.
                It likely refers to an estimator used to estimate joint states
                (positions, velocities) from other sensor data or measurements.
            robot_model (object): Required for initialization. It represents a
                model of a robot, likely used to retrieve transformation functions.
            ee_link (str): An end effector link, typically the last joint or tool
                attached to a robot arm, specifying its location in the kinematic
                tree.
            base_link (Union[NoneType, str]): Optional. It represents a reference
                frame from which to compute the transform and Jacobian of an
                end-effector link.

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
        Computes and returns a transformation matrix, likely a homogeneous
        transformation matrix, based on the current position as estimated by the
        joint state estimator. The actual transformation is performed by calling
        the _T function with the q argument.

        Returns:
            numpyndarray: 4x4 matrix representing a transformation that is calculated
            based on the current position. This transformation is derived from an
            object's internal state estimator, which is likely a component of a
            larger robotic system.

        """
        q = self._joint_state_estimator.get_position()
        return self._T(q)

    def get_velocity(self):
        """
        Computes the velocity in task space by taking the dot product of the
        Jacobian matrix `J` and the joint velocities `dq`, with both derived from
        a joint state estimator.

        Returns:
            numpyndarray: A vector representing the linear velocity of a rigid
            body at its center of mass, calculated based on joint velocities and
            their Jacobian transformation.

        """
        q = self._joint_state_estimator.get_position()
        dq = self._joint_state_estimator.get_velocity()
        J = self._J(q)
        return J @ dq

    def get_acceleration(self):
        """
        Calculates the acceleration of the end effector in task space by numerically
        differentiating the velocity vector with respect to time using two consecutive
        sample times and joint state estimates.

        Returns:
            numpyndarray: A representation of the acceleration of the robot's
            joints based on previous velocity measurements and sample time.

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
    Defines an abstract base class that must be implemented by subclasses to
    estimate external torques. It includes a single method, `get_external_torque`,
    which is not implemented and must be overridden in concrete implementations
    to return the estimated external torque.

    """
    @abc.abstractmethod
    def get_external_torque(self):
        """
        Returns an estimate of the external torque acting on a system or object.
        It is declared as abstract, requiring concrete implementations from its
        subclasses to provide actual torque estimation logic.

        """
        pass


class FRIExternalTorqueEstimator(ExternalTorqueEstimator):
    """
    Provides an estimate of external torques acting on a robot, utilizing data
    from a client that interfaces with the robot's state information. It retrieves
    and flattens the external torque values obtained through the client's API.

    Attributes:
        _client (object|FRIProtocolClient): Initialized by passing it to the
            constructor during object creation through the parameter client. It
            holds a reference to a client object used for communication with a robot.

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
        Initializes object attributes, including estimators and robot model
        components, for computing wrenches based on joint states and external
        torques, given a tip link and base link configuration.

        Args:
            client (object | None): Referenced but not used within the method. Its
                purpose or usage is not clear from this snippet of code.
            joint_state_estimator (object): Stored as an instance variable
                `_joint_state_estimator`. It appears to be an estimator for joint
                states, possibly from sensor data or other sources.
            external_torque_estimator (object): Expected to implement an external
                torque estimation capability. Its purpose is not explicitly stated,
                but its name suggests it estimates torques from sources outside
                the robot's mechanical system.
            robot_model (RobotModel): Used to get Jacobian functions for geometric
                calculations. It provides methods such as `get_global_link_geometric_jacobian_function`
                or `get_link_geometric_jacobian_function`.
            tip_link (str | int): Required for initializing an instance of this
                class. It represents the final link on the robot's kinematic chain
                that is used as a reference point for various calculations.
            base_link (str | NoneType): Optional. It can either be omitted (None)
                or set to a string representing the name of a link in the robot
                model, used as an alternative base for calculating the Jacobian.
            n_data (int | None): 50 by default, representing the number of data
                points or samples to be stored within the class instance.

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
        Calculates the Moore-Penrose pseudoinverse of the Jacobian matrix, given
        by the input position from the joint state estimator and a conditioning
        parameter `rcond`. The pseudoinverse is used to invert the Jacobian.

        """
        q = self._joint_state_estimator.get_position()
        return np.linalg.pinv(self._jacobian(q), rcond=self._rcond)

    def ready(self):
        return len(self._data) >= self._n_data

    def update(self):
        """
        Checks if the current data length is less than the required data length.
        If true, it calls the `_update_data` method to update the internal state,
        ensuring that the required amount of data is always available for estimation.

        """
        if len(self._data) < self._n_data:
            self._update_data()

    @abc.abstractmethod
    def _update_data(self):
        """
        Serves as an abstract method, requiring derived classes to implement it.
        This method likely updates internal data used by the estimator to perform
        calculations or operations. Its implementation is not provided and must
        be defined elsewhere in the codebase.

        """
        pass

    @abc.abstractmethod
    def get_wrench(self):
        """
        Returns a wrench object, which represents an estimation or calculation of
        a wrench force or torque in a mechanical system. The actual implementation
        is abstract and must be provided by concrete subclasses.

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
        Retrieves an external torque estimate from its associated estimator,
        converts it to a list, and appends this value to the data stored by the
        object for subsequent use or analysis.

        """
        tau_ext = self._external_torque_estimator.get_external_torque()
        self._data.append(tau_ext.tolist())

    def get_wrench(self):
        """
        Calculates the estimated wrench (torque and force) acting on a joint by
        subtracting an offset from the external torque, then multiplying by the
        inverse Jacobian transpose. This is used to correct for bias in the torque
        estimator.

        Returns:
            ndarray: 1D array representing the wrench applied to a robotic arm or
            end effector, calculated by taking the transpose of the inverse Jacobian
            matrix and multiplying it with the estimated external torque.

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
        Updates the internal data with the estimated external force. It retrieves
        the estimated external torque, computes the corresponding force through a
        matrix multiplication and flattens it into a list before appending it to
        the internal data.

        """
        tau_ext = self._external_torque_estimator.get_external_torque()
        f_ext = self._inverse_jacobian().T @ tau_ext
        self._data.append(f_ext.flatten().tolist())

    def get_wrench(self):
        """
        Computes an estimate of the wrench acting on an end-effector by subtracting
        a task space offset from the product of the inverse jacobian and external
        torque.

        Returns:
            ndarray: 1-dimensional, representing a wrench (force and torque) exerted
            on an object, calculated by combining external torque and inverse
            jacobian transformation with data offset adjustment.

        """
        offset = np.mean(self._data, axis=0)
        tau_ext = self._external_torque_estimator.get_external_torque()
        return self._inverse_jacobian().T @ tau_ext - offset
