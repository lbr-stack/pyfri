import argparse
import math
import sys

import numpy as np

import pyfri as fri


class LBRTorqueSineOverlayClient(fri.LBRClient):
    """
    Monitors and controls a robot arm by applying sinusoidal torques to specific
    joints, synchronized with the robot's sampling frequency. It updates torque
    values based on the current phase of the sine wave and sends these commands
    to the robot.

    Attributes:
        joint_mask (numpyndarray|int): Used to specify a subset of joints on which
            the torque overlay should be applied. It filters the joints for which
            the torques will be calculated.
        freq_hz (float): Used to represent a frequency value in Hertz. It appears
            to be used as a factor for generating sinusoidal torques in the robot's
            joints.
        torque_ampl (float): Initialized during object creation with a value
            specified by the user, representing the amplitude of the torque signal
            to be applied to the joints.
        phi (float): Initialized to zero. It represents a phase variable used for
            generating sine wave offsets applied to joint torques during
            torque-controlled operations.
        step_width (float): 0 by default. It is set to a value that represents the
            step width for the sine wave function, calculated as twice pi times
            the frequency in Hz times the sample time of the robot state.
        torques (npndarray[float32]): Initialized as a NumPy array with zeros. It
            stores the torques to be applied to specific joints based on a sine
            wave pattern.

    """
    def __init__(self, joint_mask, freq_hz, torque_amplitude):
        """
        Initializes the object's attributes: joint mask, frequency, torque amplitude,
        and state variables (phi, step width, and torques) with default values or
        those provided as arguments. It also calls the parent class's `__init__`
        method using super().

        Args:
            joint_mask (numpy.ndarray | List[bool]): 1D array-like object containing
                boolean values, where each value corresponds to a joint and indicates
                whether it should be included or excluded from the simulation.
            freq_hz (float): Used to specify the frequency in Hertz of the torques
                applied by the system.
            torque_amplitude (float): 3 times larger than `freq_hz`.

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
        Monitors state changes and resets internal variables when transitioning
        to the MONITORING_READY state. It initializes torques, phase angle, and
        step width based on the robot's sample time and frequency.

        Args:
            old_state (fri.ESessionState): Used to record the state of the session
                before the current change, which is then printed by the function
                as it changes.
            new_state (Union[fri.ESessionState, str]): Not fully specified in this
                code snippet, indicating that it may contain different values from
                the fri.ESessionState enum, which represents various states in the
                robotics session.

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
        Sets joint positions and torques based on current state before sending new
        commands to the robot, ensuring a smooth transition between commands when
        client command mode is torque control.

        """
        self.robotCommand().setJointPosition(self.robotState().getIpoJointPosition())

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques)

    def command(self):
        """
        Sends torque commands to a robot, applying a sine wave pattern to selected
        joints. It updates joint positions and torques based on the current client
        command mode and phase of the sine wave.

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
    Initializes an argument parser for a command-line interface, validating user
    input and converting it into a format suitable for further processing by a
    KUKA Sunrise Controller. It returns parsed arguments as namespace objects.

    Returns:
        argparseNamespace: A container object that stores all arguments passed to
        it, along with their default values if provided. It encapsulates all parsed
        command-line arguments into a structured format for further processing and
        use within the application.

    """
    def cvt_joint_mask(value):
        """
        Converts a given value to an integer within the range 0 to 6 and returns
        it if valid. It  checks for values outside this range, raising an error
        with an informative message if encountered.

        Args:
            value (Union[int, str]): Implicitly defined by argparse which typically
                expects it to be a string that represents an integer.

        Returns:
            int|None: An integer value within the range from 0 to 6 (inclusive),
            if the input is valid; otherwise, it raises a TypeError with a message
            describing the invalid input.

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
    Initializes a KUKA Sunrise controller client, connects to it, and enters a
    loop where it steps the application until the session state becomes idle or a
    keyboard interrupt occurs.

    Returns:
        int: 1 if a connection to KUKA Sunrise controller fails, and 0 otherwise,
        indicating successful execution.

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
