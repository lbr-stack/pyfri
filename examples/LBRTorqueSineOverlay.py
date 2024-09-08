import argparse
import math
import sys

import numpy as np

import pyfri as fri


class LBRTorqueSineOverlayClient(fri.LBRClient):
    """
    Implements a client for an LBR robot, specifically designed to apply sinusoidal
    torques to joints masked by the `joint_mask` attribute. It overrides default
    behavior to control joint positions and torques during monitoring and command
    phases.

    Attributes:
        joint_mask (Sequence[int]): Used to specify which joints are subject to
            the sinusoidal torque command. It appears to be a subset of the robot's
            total number of joints.
        freq_hz (float): Initialized with a specific value during object creation,
            which represents the frequency of the sine wave used to modulate the
            torques applied to the robot's joints.
        torque_ampl (float): Initialized by the `__init__` method with the value
            provided to it as an argument. It represents the amplitude of the sine
            wave used in torque calculations.
        phi (float): Initialized to 0. It keeps track of a phase angle used in
            calculating torque offsets for sine wave-like motion, incremented by
            `self.step_width` on each call to `command`.
        step_width (float): 0 by default. It stores the angular step size of a
            sine wave used to modulate torque amplitudes, calculated as twice the
            product of frequency and sample time.
        torques (npndarray[float32,ndim1]): Initialized as a zero-filled array
            with the size corresponding to the number of joints of the robot. It
            stores torques values for each joint during the command execution.

    """
    def __init__(self, joint_mask, freq_hz, torque_amplitude):
        """
        Initializes its attributes, including joint mask, frequency, and torque
        amplitude, and sets default values for phase, step width, and torques. It
        also calls the parent class's constructor using super().

        Args:
            joint_mask (np.ndarray | List[int]): 1D array or list containing boolean
                values that determine which joints to consider for torque application.
            freq_hz (float): Used to represent the frequency in Hertz. It determines
                the speed or oscillation rate of an oscillatory signal in this context.
            torque_amplitude (float): Used to specify the amplitude of torque.
                This value determines the maximum amount of torque that can be
                applied during motion.

        """
        super().__init__()
        self.joint_mask = joint_mask
        self.freq_hz = freq_hz
        self.torque_ampl = torque_amplitude
        self.phi = 0.0
        self.step_width = 0.0
        self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS, dtype=np.float32)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        """
        Resets internal state when the robot enters the MONITORING_READY state,
        setting torque and phi values to zero and recalculating a step width based
        on frequency and sample time.

        Args:
            old_state (fri.ESessionState | None): The state of the system before
                the change occurred.
            new_state (Enum[fri.ESessionState] | Enum[fri.LBRState]): Initialized
                with a value from fri.ESessionState or fri.LBRState based on whether
                the state changed is due to monitoring ready event or any other event.

        """
        print(f"State changed from {old_state} to {new_state}")

        if new_state == fri.ESessionState.MONITORING_READY:
            self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS, dtype=np.float32)
            self.phi = 0.0
            self.step_width = (
                2 * math.pi * self.freq_hz * self.robotState().getSampleTime()
            )

    def waitForCommand(self):
        """
        Waits for and applies a joint position command to the robot, with or without
        torque commands depending on the client's mode.

        """
        self.robotCommand().setJointPosition(self.robotState().getIpoJointPosition())

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques)

    def command(self):
        """
        Generates torque commands for the robot based on a sine wave pattern, where
        the amplitude and phase are modulated by user-defined parameters. It sets
        joint positions and torques accordingly to execute this motion command.

        """
        self.robotCommand().setJointPosition(self.robotState().getIpoJointPosition())

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            offset = self.torque_ampl * math.sin(self.phi)
            self.phi += self.step_width

            if self.phi >= 2 * math.pi:
                self.phi -= 2 * math.pi

            self.torques[self.joint_mask] = offset

            self.robotCommand().setTorque(self.torques)


def args_factory():
    """
    Parses command-line arguments using argparse, validating and converting input
    values as necessary. It returns a Namespace object containing the parsed
    arguments. The function defines type conversion for specific argument types,
    including joint masks within a range of 0-7.

    Returns:
        argparseNamespace: An object that has several named attributes (parsed
        arguments) such as hostname, port, joint_mask, freq_hz and torque_amplitude.

    """
    def cvt_joint_mask(value):
        """
        Converts a given value to an integer and checks if it falls within the
        specified range [0, 7). If valid, it returns the integer; otherwise, it
        raises an error with a message indicating that the value is out of range.

        Args:
            value (Union[int, str] | float): Used to represent a joint mask index.
                It can be an integer, float or a string that represents a valid
                integer value for a joint mask in Open3D.

        Returns:
            int: 1 digit integer from 0 to 6 representing joint mask if input is
            a valid number in that range, otherwise it raises an error.

        """
        int_value = int(value)
        if 0 <= int_value < 7:
            return int_value
        else:
            raise argparse.ArgumentTypeError(f"{value} is not in the range [0, 7).")

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
        "--joint-mask",
        dest="joint_mask",
        type=cvt_joint_mask,
        default=3,
        help="The joint to move.",
    )
    parser.add_argument(
        "--freq-hz",
        dest="freq_hz",
        type=float,
        default=0.25,
        help="The frequency of the sine wave.",
    )
    parser.add_argument(
        "--torque-amplitude",
        dest="torque_amplitude",
        type=float,
        default=15.0,
        help="Applitude of the sine wave.",
    )

    return parser.parse_args()


def main():
    """
    Initializes and connects to a KUKA Sunrise controller, runs a client application
    that applies a sine wave torque overlay to a robot, and exits when the robot
    session state becomes IDLE or upon keyboard interrupt.

    Returns:
        int|None: 1 if connection to KUKA Sunrise controller fails or 0 otherwise,
        indicating successful execution of the application.

    """
    print("Running FRI Version:", fri.FRI_CLIENT_VERSION)

    args = args_factory()
    client = LBRTorqueSineOverlayClient(
        args.joint_mask, args.freq_hz, args.torque_amplitude
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
        print("Goodbye")

    return 0


if __name__ == "__main__":
    sys.exit(main())
