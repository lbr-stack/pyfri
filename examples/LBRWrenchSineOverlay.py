import argparse
import math
import sys

import numpy as np

import pyFRI as fri


class LBRWrenchSineOverlayClient(fri.LBRClient):
    """
    Initializes and controls a robot arm to apply sinusoidal wrenches along two
    axes, while allowing for monitoring and command mode switching. It updates
    wrench values based on frequencies, amplitudes, and phase offsets, ensuring
    continuous motion within a periodic range.

    Attributes:
        frequencyX (float|None): Initialized in the constructor to hold a frequency
            value for the sine overlay along the x-axis. It stores a floating-point
            number representing the frequency in Hertz.
        frequencyY (float): Assigned a value through the constructor `__init__`.
            It represents the frequency of a sine wave along the Y axis.
        amplitudeX (float): Used to represent the amplitude of the sine wave in
            the x-direction. It determines the magnitude of the oscillation applied
            by the wrench command.
        amplitudeY (float): Set through the constructor, initialized with a value
            passed to the class instantiation. It defines the amplitude of sine
            wave for the y-axis component of the wrench signal.
        stepWidthX (float): 0 by default. It represents the angle increment for
            the sine wave generation on the X-axis, calculated as 2π times the
            frequency of oscillation multiplied by the sample time of the robot state.
        stepWidthY (float): 0.0 by default. It stores the step width for updating
            the Y-component of a wrench, calculated as 2 * pi times frequency Y
            times sample time.
        phiX (float): 0 by default. It stores a phase offset for sine wave generation
            used to calculate wrench components, and is updated based on the
            frequency and sample time in the `onStateChange` method.
        phiY (float): 0 by default. It accumulates phase changes when commanded
            to do so in methods like `command`. Its initial value is reset to zero
            when the client state changes to MONITORING_READY.
        wrench (ndarray[npfloat32,6]): Initialized to a zero vector with six
            elements in its constructor. It represents a wrench applied to the
            robot, composed of torque and force components along each axis.

    """
    def __init__(self, frequencyX, frequencyY, amplitudeX, amplitudeY):
        """
        Initializes an instance with parameters that define a sinusoidal wrench,
        including frequencies and amplitudes for both X and Y axes, initializing
        various internal attributes.

        Args:
            frequencyX (float): Used to set the frequency of oscillation in the
                X-direction. It represents how often the signal repeats itself
                along this axis.
            frequencyY (float): Set as an attribute of the class instance under
                the same name. It is assigned the value passed to it when an object
                of the class is created.
            amplitudeX (float): 0 by default when not specified. It represents the
                magnitude or size of a movement in the X-direction, likely for a
                sinusoidal motion or oscillation.
            amplitudeY (float): Used to set the amplitude of the movement in the
                Y direction.

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
        Resets certain variables when the robot's state changes to MONITORING_READY,
        preparing it for new movements with specified frequencies and sample times.

        Args:
            old_state (ESessionState | None): Used to represent the old state of
                the system before it changes, which in this case is expected to
                be updated when the new state transitions into MONITORING_READY.
            new_state (EnumMember[fri.ESessionState]): Checked against the value
                fri.ESessionState.MONITORING_READY indicating that the session has
                reached a state of readiness for monitoring.

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
        Sets the joint positions of the robot to its IPO (In-Positioning Override)
        joint positions and optionally applies a wrench command based on the
        client's current command mode. It is likely called after receiving a new
        command from the server.

        """
        self.robotCommand().setJointPosition(self.robotState().getIpoJointPosition())
        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.WRENCH:
            self.robotCommand().setWrench(self.wrench)

    def command(self):
        """
        Generates and sends wrench commands to a robot in a specific client mode.
        It sets joint positions based on IPO data, calculates sinusoidal wrench
        values for X and Y axes, and updates these values over time by incrementing
        phase angles modulo 2π.

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


def get_arguments():
    """
    Parses command-line arguments using the `argparse` library, extracting and
    returning their values as a structured object. It defines several optional
    arguments with default values for various settings related to communication
    and sine wave generation.

    Returns:
        argparseNamespace: An object containing all the arguments passed as a
        dictionary-like object, where each key is the argument name and the
        corresponding value is the parsed argument from user input.

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
    Initializes and runs an FRI client application, connecting to a KUKA Sunrise
    controller, executing a wrench-sine-overlay motion plan, and handling any
    errors or interruptions that may occur during execution.

    Returns:
        int|None: 1 if connection to KUKA Sunrise controller fails and 0 otherwise,
        or None if the function terminates due to an exception being caught.

    """
    print("Running FRI Version:", fri.FRI_CLIENT_VERSION)

    args = get_arguments()
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
