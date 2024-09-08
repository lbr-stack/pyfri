import argparse
import math
import sys

import numpy as np

import pyfri as fri


class LBRWrenchSineOverlayClient(fri.LBRClient):
    """
    Extends an LBR client, allowing it to generate a wrench command that simulates
    sine wave oscillations on two axes based on input frequencies and amplitudes.
    It updates the wrench command at each state change and sends it to the robot
    if in wrench mode.

    Attributes:
        frequencyX (float): Initialized with a user-provided value in the constructor
            (`__init__`). It represents the frequency of the sine wave applied to
            one axis.
        frequencyY (float): Initialized by the user through the constructor method,
            along with its corresponding amplitude and phase values for generating
            a sine wave in the Y-direction.
        amplitudeX (float): Used to represent the amplitude of a sine wave applied
            along axis X when generating wrench commands. It is set by the user
            via the class constructor.
        amplitudeY (float): 2.0 by default. It represents the amplitude or magnitude
            of the sine wave applied to joint Y in the wrench command.
        stepWidthX (float): 0.0 by default. It represents the increment in phase
            angle phiX for each time step, calculated as twice pi times frequencyX
            times the sample time.
        stepWidthY (float): Initialized to zero during initialization. It is updated
            in the `onStateChange` method by calculating it as twice pi times the
            product of the frequency Y, the sample time, and math constant pi.
        phiX (float): 0 by default. It represents a phase angle used to calculate
            the sine wave amplitude for the X-axis (wrench 0).
        phiY (float): Initialized to 0.0. It represents the phase angle of the
            sine wave for joint Y and is updated incrementally in the `command`
            method based on its step width.
        wrench (npndarray[float32]): 6 elements long, representing a wrench force
            with six degrees of freedom (three for force and three for torque).

    """
    def __init__(self, frequencyX, frequencyY, amplitudeX, amplitudeY):
        """
        Initializes an object by setting its attributes based on the provided
        frequencies and amplitudes for sine wave overlays in X and Y directions,
        as well as initializing other variables to represent wrench values and angles.

        Args:
            frequencyX (float): Set to represent the frequency in the x-direction.
            frequencyY (float): Initialized with the value passed to it in the
                function call. It represents a frequency associated with one
                dimension of an oscillation or vibration, likely in a two-dimensional
                context.
            amplitudeX (float): Initialized with a value provided by the user
                during instantiation of an object. It represents the amplitude of
                a sinusoidal wave in the X direction.
            amplitudeY (float): Initialized to represent an amplitude in a
                two-dimensional oscillation or motion along the Y-axis, typically
                used for mathematical models or simulation purposes.

        """
        super().__init__()
        self.frequencyX = frequencyX
        self.frequencyY = frequencyY
        self.amplitudeX = amplitudeX
        self.amplitudeY = amplitudeY
        self.stepWidthX = 0.0
        self.stepWidthY = 0.0
        self.phiX = 0.0
        self.phiY = 0.0
        self.wrench = np.zeros(6, dtype=np.float32)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        """
        Resets and recalculates certain internal parameters when the robot's state
        transitions to MONITORING_READY.

        Args:
            old_state (fri.ESessionState): Not used within the provided code
                snippet. Its purpose appears to be a reference to a previous state,
                although its value is immediately discarded upon comparison with
                the new state.
            new_state (EnumMember[fri.ESessionState]): Used to identify the state
                of a session. It holds the new state value after an event or
                transition occurs. In this case, it checks for the 'MONITORING_READY'
                state.

        """
        if new_state == fri.ESessionState.MONITORING_READY:
            self.phiX = 0.0
            self.phiY = 0.0
            self.stepWidthX = (
                2.0 * math.pi * self.frequencyX * self.robotState().getSampleTime()
            )
            self.stepWidthY = (
                2.0 * math.pi * self.frequencyY * self.robotState().getSampleTime()
            )

    def waitForCommand(self):
        """
        Synchronizes robot joint positions and wrench settings with current client
        command mode, updating joint positions according to IPO (Intermediate Pose
        Option) data when in WRENCH mode.

        """
        self.robotCommand().setJointPosition(self.robotState().getIpoJointPosition())
        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.WRENCH:
            self.robotCommand().setWrench(self.wrench)

    def command(self):
        """
        Generates and sends joint position commands to the robot, implementing a
        sine wave pattern for the wrench commands when the client command mode is
        set to WRENCH.

        """
        self.robotCommand().setJointPosition(self.robotState().getIpoJointPosition())
        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.WRENCH:
            self.wrench[0] = self.amplitudeX * math.sin(self.phiX)
            self.wrench[1] = self.amplitudeY * math.sin(self.phiY)

            self.phiX += self.stepWidthX
            self.phiY += self.stepWidthY

            if self.phiX >= 2.0 * math.pi:
                self.phiX -= 2.0 * math.pi

            if self.phiY >= 2.0 * math.pi:
                self.phiY -= 2.0 * math.pi

            self.robotCommand().setWrench(self.wrench)


def args_factory():
    """
    Creates an argument parser to extract user input from command-line arguments.
    It defines several optional parameters, including hostname, port, frequencies,
    and amplitudes for sine waves in x- and y-axes, and returns a parsed namespace
    object containing these values.

    Returns:
        argparseNamespace: An object containing all arguments parsed from command
        line and their values. This includes hostname, port, frequencyX, frequencyY,
        amplitudeX, and amplitudeY.

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
        "--frequencyx",
        dest="frequencyX",
        type=float,
        default=0.25,
        help="The frequency of sine wave in x-axis.",
    )
    parser.add_argument(
        "--frequencyy",
        dest="frequencyY",
        type=float,
        default=0.25,
        help="The frequency of sine wave in y-axis.",
    )
    parser.add_argument(
        "--amplitudex",
        dest="amplitudeX",
        type=float,
        default=5.0,
        help="The amplitude of sine wave in x-axis.",
    )
    parser.add_argument(
        "--amplitudey",
        dest="amplitudeY",
        type=float,
        default=5.0,
        help="The amplitude of sine wave in y-axis.",
    )

    return parser.parse_args()


def main():
    """
    Initializes a client application to connect to a KUKA Sunrise controller, sets
    up an overlay for sine movement, and runs indefinitely until idle or interrupted
    by a KeyboardInterrupt.

    Returns:
        int: 1 if connection to KUKA Sunrise controller failed, and 0 otherwise
        indicating successful execution.

    """
    print("Running FRI Version:", fri.FRI_CLIENT_VERSION)

    args = args_factory()
    print(args)
    client = LBRWrenchSineOverlayClient(
        args.frequencyX, args.frequencyY, args.amplitudeX, args.amplitudeY
    )
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

    finally:
        app.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
