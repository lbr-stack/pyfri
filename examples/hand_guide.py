import argparse
import sys

import numpy as np
from admittance import AdmittanceController

import pyFRI as fri
from pyFRI.tools.filters import ExponentialStateFilter
from pyFRI.tools.state_estimators import (
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
    Initializes a hand guide client for an FRI (Force-Controlled Robot Interface)
    compliant robot arm. It estimates joint states and wrenches, applies filters
    to stabilize control inputs, and issues position commands to the robot based
    on estimated values and controller feedback.

    Attributes:
        controller (AdmittanceController|None): Initialized with lbr_ver in the
            __init__ method, representing a controller object for admittance control
            of a robot manipulator.
        joint_state_estimator (JointStateEstimator): Initialized with `self` as
            its argument, suggesting that it is used to estimate the joint states
            of the robot based on data from the client itself.
        external_torque_estimator (FRIExternalTorqueEstimator|None): Created by
            calling the constructor FRIExternalTorqueEstimator with a reference
            to the current object as its argument.
        wrench_estimator (WrenchEstimatorTaskOffset|None): An instance of
            WrenchEstimatorTaskOffset class. It estimates the wrench applied to
            the end-effector of the robot.
        wrench_filter (ExponentialStateFilter): Not used directly within the methods
            in this snippet, but its filter method is called to modify a wrench
            measurement before passing it through the controller.

    """
    def __init__(self, lbr_ver):
        """
        Initializes various components for admittance control, including estimators
        and filters, setting up relationships between them based on their dependencies.
        It sets up a complete structure for controlling an LBR robot arm using
        admittance control methods.

        Args:
            lbr_ver (str | int): Used to specify the version or variant of the
                KUKA LBR robot arm model being controlled by the AdmittanceController
                class.

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
        Waits for a specific client command mode to be enabled, updates the wrench
        estimator, retrieves the current joint position and stores it as self.q,
        then calls the command_position method to send a command based on self.q.

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
        Updates the robot's joint position based on filtered wrench data from an
        estimator, controller and filter. It only proceeds with this calculation
        if the estimator is ready. Otherwise, it updates the estimator and resets
        the joint position.

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


def get_arguments():
    """
    Initializes an argument parser and defines command-line arguments for a script,
    including hostname, port, and KUKA LBR Med version number, returning parsed
    arguments as an object.

    Returns:
        argparseNamespace: An object containing parsed arguments from the command
        line, specifically hostname, port, and lbr-ver (KUKA LBR Med version number)
        along with their corresponding values.

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
    Initializes and runs a client application for interacting with a KUKA Sunrise
    controller, executing steps until the robot reaches an idle state or interrupted
    by user input. It handles connections, disconnections, and exceptions.

    Returns:
        int: 0 on successful execution and 1 if connection to KUKA Sunrise controller
        fails, indicating an unsuccessful run. The returned value is used as a
        return code for the program.

    """
    print("Running FRI Version:", fri.FRI_CLIENT_VERSION)

    args = get_arguments()
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
