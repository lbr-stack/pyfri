import argparse
import sys

import numpy as np
from admittance import AdmittanceController

import pyfri as fri
from pyfri.tools.filters import ExponentialStateFilter
from pyfri.tools.state_estimators import (
    FRIExternalTorqueEstimator,
    JointStateEstimator,
    WrenchEstimatorTaskOffset,
)

if fri.FRI_CLIENT_VERSION_MAJOR == 1:
    POSITION = fri.EClientCommandMode.POSITION
elif fri.FRI_CLIENT_VERSION_MAJOR == 2:
    POSITION = fri.EClientCommandMode.JOINT_POSITION


class HandGuideClient(fri.LBRClient):
    """
    Simulates an admittance controller for a robot arm's end-effector, estimating
    and filtering wrenches to adjust joint positions in real-time based on external
    forces applied to the arm. It ensures a specific client command mode is used.

    Attributes:
        controller (AdmittanceController): Initialized with a parameter lbr_ver
            in the `__init__` method. It represents an admittance control strategy
            used to update joint positions based on estimated wrenches and robot
            state.
        joint_state_estimator (JointStateEstimator|None): Initialized with a
            JointStateEstimator instance that takes self as its argument, suggesting
            it estimates the state of the robot's joints.
        external_torque_estimator (FRIExternalTorqueEstimator|None): Initialized
            with the HandGuideClient instance as its self reference in the `__init__`
            method. It estimates external torques on the robot arm.
        wrench_estimator (WrenchEstimatorTaskOffset|None): Initialized with a set
            of four arguments: the `self`, its `joint_state_estimator`,
            `external_torque_estimator`, `controller.robot`, and `controller.ee_link`.
        wrench_filter (ExponentialStateFilter|None): Initialized as such:
            `self.wrench_filter = ExponentialStateFilter()`. It appears to be a
            filter for exponential smoothing of wrenches.

    """
    def __init__(self, lbr_ver):
        """
        Initializes various components including an admittance controller, joint
        state estimator, external torque estimator, wrench estimator, and exponential
        state filter for a Franka robot arm with specified LBR version.

        Args:
            lbr_ver (str | int): An identifier that specifies the type or version
                of the LBR (Lightweight Robot) being used. It is passed to various
                classes for specific initialization, configuration, or control purposes.

        """
        super().__init__()
        self.controller = AdmittanceController(lbr_ver)
        self.joint_state_estimator = JointStateEstimator(self)
        self.external_torque_estimator = FRIExternalTorqueEstimator(self)
        self.wrench_estimator = WrenchEstimatorTaskOffset(
            self,
            self.joint_state_estimator,
            self.external_torque_estimator,
            self.controller.robot,
            self.controller.ee_link,
        )
        self.wrench_filter = ExponentialStateFilter()

    def command_position(self):
        self.robotCommand().setJointPosition(self.q.astype(np.float32))

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        print(f"State changed from {old_state} to {new_state}")

    def waitForCommand(self):
        """
        Ensures that the client command mode is set to POSITION before proceeding
        with further operations. If not, it raises an exit system. Otherwise, it
        retrieves the current Ipo joint position and calls the `command_position`
        method.

        """
        if self.robotState().getClientCommandMode() != POSITION:
            print(
                f"[ERROR] hand guide example requires {POSITION.name} client command mode"
            )
            raise SystemExit

        self.q = self.robotState().getIpoJointPosition()
        self.command_position()

    def command(self):
        """
        Generates and sends joint position commands to the robot based on filtered
        wrench data from the environment, using a control strategy defined by the
        controller function.

        """
        if not self.wrench_estimator.ready():
            self.wrench_estimator.update()
            self.robotCommand().setJointPosition(self.q.astype(np.float32))
            return

        # Get robot state
        wr = self.wrench_estimator.get_wrench()
        dt = self.robotState().getSampleTime()

        # Filter wrench
        wf = self.wrench_filter.filter(wr)

        # Compute goal using admittance controller
        self.q = self.controller(self.q, wf, dt)

        # Command robot
        self.command_position()


def args_factory():
    """
    Parses command-line arguments using `argparse`. It defines three required
    arguments: hostname, port, and lbr-ver, with specific data types and validation
    rules for each. The parsed arguments are then returned as a namespace object.

    Returns:
        argparseNamespace: An object containing all the command-line arguments
        parsed by the ArgumentParser instance.

    """
    parser = argparse.ArgumentParser(description="LRBJointSineOverlay example.")
    parser.add_argument(
        "--hostname",
        dest="hostname",
        default=None,
        help="The hostname used to communicate with the KUKA Sunrise Controller.",
    )
    parser.add_argument(
        "--port",
        dest="port",
        type=int,
        default=30200,
        help="The port number used to communicate with the KUKA Sunrise Controller.",
    )
    parser.add_argument(
        "--lbr-ver",
        dest="lbr_ver",
        type=int,
        choices=[7, 14],
        required=True,
        help="The KUKA LBR Med version number.",
    )

    return parser.parse_args()


def main():
    """
    Establishes a connection to a KUKA Sunrise controller, runs an application
    loop, and ensures proper cleanup upon exit or termination. It also displays
    version information and error messages as necessary. The connection is maintained
    until the controller enters an idle state.

    Returns:
        int: 0 on successful execution and 1 on failure to connect to KUKA Sunrise
        controller.

    """
    print("Running FRI Version:", fri.FRI_CLIENT_VERSION)

    args = args_factory()
    client = HandGuideClient(args.lbr_ver)
    app = fri.ClientApplication(client)
    success = app.connect(args.port, args.hostname)

    if not success:
        print("Connection to KUKA Sunrise controller failed.")
        return 1

    try:
        while success:
            success = app.step()

            if client.robotState().getSessionState() == fri.ESessionState.IDLE:
                break

    except KeyboardInterrupt:
        pass

    except SystemExit:
        pass

    finally:
        app.disconnect()
        print("Goodbye")

    return 0


if __name__ == "__main__":
    sys.exit(main())
