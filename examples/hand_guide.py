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
    Initializes and manages a robotic arm's state, commanding its position based
    on estimated joint states and external wrenches. It updates the robot's pose,
    checks command mode, and filters wrench estimates to ensure smooth motion control.

    Attributes:
        controller (AdmittanceController|None): Initialized with an instance of
            AdmittanceController in the constructor. It represents a controller
            used for controlling the robot's joints based on estimated wrenches
            and other factors.
        joint_state_estimator (JointStateEstimator|None): Initialized with the
            instance itself as its argument, indicating it's responsible for
            estimating joint states based on current conditions.
        external_torque_estimator (FRIExternalTorqueEstimator|None): Initialized
            with its constructor being called on self, which suggests it estimates
            external torques acting on the robot.
        wrench_estimator (WrenchEstimatorTaskOffset|JointStateEstimator|FRIExternalTorqueEstimator):
            Associated with a robot, ee link, and joint state estimator. It estimates
            task-space forces and torques from joint sensor readings and external
            torque estimations.
        wrench_filter (ExponentialStateFilter): Used to filter wrench data from
            the `WrenchEstimatorTaskOffset`. It appears to be a state estimation
            method to provide more accurate results.

    """
    def __init__(self, lbr_ver):
        """
        Initializes objects for admittance control, joint state estimation, external
        torque estimation, wrench estimation, and exponential state filtering based
        on provided parameters. It also establishes relationships between these
        components for coordinated functioning.

        Args:
            lbr_ver (str | int): Used to specify the version or identifier of the
                robot. This value is passed to the AdmittanceController class
                during initialization. It appears to be related to the robotic
                arm's model.

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
        Waits for a command from the client to be executed. It checks if the current
        client mode matches POSITION, updates the wrench estimator, retrieves the
        current joint position, and executes the command position.

        """
        if self.robotState().getClientCommandMode() != POSITION:
            print(
                f"[ERROR] hand guide example requires {POSITION.name} client command mode"
            )
            raise SystemExit

        self.wrench_estimator.update()
        self.q = self.robotState().getIpoJointPosition()
        self.command_position()

    def command(self):
        """
        Updates robot position commands by filtering and processing wrench data
        from the wrench estimator, then calls the `controller` function to compute
        new joint positions. It also sets the joint position command of the robot.

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
    Creates an instance of the ArgumentParser class and defines arguments for
    parsing command-line input. It returns a Namespace object containing user-specified
    values for hostname, port, and LBR Med version number.

    Returns:
        argparseNamespace: An object containing arguments parsed from the command
        line. This object holds attributes for `hostname`, `port`, and `lbr_ver`.

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
    Initializes a connection to a KUKA Sunrise controller,  attempts to establish
    communication, and repeatedly polls the robot's state until idle or an error
    occurs. It also handles keyboard interrupts and system exits before disconnecting
    from the controller and terminating.

    Returns:
        int: 1 when connection to KUKA Sunrise controller fails, and 0 otherwise.

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
