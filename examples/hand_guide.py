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
    Initializes a hand guide client for interacting with a robot arm. It estimates
    joint states and wrenches, filters sensor data, and updates robot commands
    based on the controller's output to maintain stable movement.

    Attributes:
        controller (AdmittanceController|None): Initialized with lbr_ver as a
            parameter in its constructor method, likely controlling the robot's admittance.
        joint_state_estimator (JointStateEstimator): Initialized with a reference
            to the current `HandGuideClient` instance as its argument. It estimates
            joint states of the robot.
        external_torque_estimator (FRIExternalTorqueEstimator|None): Initialized
            with a reference to the client object itself during construction.
        wrench_estimator (WrenchEstimatorTaskOffset|None): Initialized with four
            parameters: the instance itself, its associated joint state estimator,
            external torque estimator, and controller. It filters and estimates
            wrench values based on task offsets.
        wrench_filter (ExponentialStateFilter): Used to filter the wrench estimates
            from the WrenchEstimatorTaskOffset instance associated with it.

    """
    def __init__(self, lbr_ver):
        """
        Initializes various components necessary for the controller and estimators,
        including an admittance controller, joint state estimator, external torque
        estimator, wrench estimator, and wrench filter. These are likely used to
        control a robot arm's movements.

        Args:
            lbr_ver (str | int | float): Used to specify a version or other version
                information about the LBR (Lightweight Robot) being controlled by
                this class instance.

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
        Checks if the robot's client command mode is set to POSITION, otherwise
        it prints an error message and exits. If the mode is correct, it retrieves
        the current joint position from the robot state and calls the `command_position`
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
        Updates the robot's joint position based on estimated wrench and filtered
        feedback, using a controller to compute the new position. It checks if the
        estimator is ready before proceeding with the update process.

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
    Parses command-line arguments using argparse. It defines three arguments:
    hostname, port, and lbr-ver, which are then returned as parsed arguments. The
    lbr-ver argument requires a specific integer value from the list [7, 14] and
    is mandatory.

    Returns:
        argparseNamespace: An instance that stores the parsed command-line arguments
        as attributes, allowing access to them as a dictionary-like object.

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
    Initializes a FRI client, connects to a KUKA Sunrise controller, and executes
    a sequence of steps. It continuously checks if the robot session is idle; if
    so, it breaks out of the loop. The function gracefully handles keyboard
    interrupts and system exits.

    Returns:
        int|None: 1 when connection to KUKA Sunrise controller fails or 0 on
        successful execution.

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
