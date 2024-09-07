import argparse
import math
import sys

import numpy as np

import pyFRI as fri


class LBRTorqueSineOverlayClient(fri.LBRClient):
    """
    Implements a client for Lab Robot (LBR) torque control with a sine wave overlay
    on specified joints. It monitors state changes, waits for commands, and sends
    joint position and torque commands to the robot based on user-defined frequency
    and amplitude parameters.

    Attributes:
        joint_mask (Sequence[int]): Initialized in the constructor method, but its
            exact nature or how it is used is unclear from this code snippet alone.
        freq_hz (float): Initialized with a frequency value that will be used to
            generate a sine wave for the torque command. It determines the number
            of cycles per second in the sinusoidal pattern.
        torque_ampl (float): Initialized with the torque amplitude value provided
            to its constructor. It stores the maximum torque magnitude applied to
            joints during oscillation.
        phi (float): Initialized to zero. It represents a phase angle used for
            generating sine wave amplitudes in the `command` method.
        step_width (float): 0 by default. It represents the time increment for
            updating the torque offset based on the sine wave frequency and sample
            time.
        torques (npndarray[float32,ndim1]): 0-indexed with a length equal to
            `fri.LBRState.NUMBER_OF_JOINTS`, representing a numerical array
            containing torques applied to each joint.

    """
    def __init__(self, joint_mask, freq_hz, torque_amplitude):
        """
        Initializes instance variables for joint mask, frequency, torque amplitude,
        phase angle, step width, and torques array with zeros, setting default
        values where applicable.

        Args:
            joint_mask (np.ndarray[bool]): Used to specify which joints are being
                controlled by the object. It has boolean values indicating whether
                each joint should be included (True) or not (False).
            freq_hz (float): Intended to represent a frequency value measured in
                Hertz. It is used as an input to initialize the object with its
                specified value.
            torque_amplitude (float): Assigned to the instance attribute
                `self.torque_ampl`. It represents an amplitude value related to
                torque, but its precise purpose is not clearly stated in this code
                snippet.

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
        Updates internal state variables based on changes to the robot's session
        state. It resets torque values and certain parameters when the session
        enters a monitoring-ready state.

        Args:
            old_state (fri.ESessionState): Passed to indicate the state that
                occurred before the change. It represents the current state of the
                system or robot session when it transitioned from one state to another.
            new_state (fri.ESessionState): Compared to a value MONITORING_READY,
                which suggests it represents an enumeration or state machine that
                tracks the current session state of a robotic system.

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
        Synchronizes joint positions with the robot's current IPO position, then
        sets the robot's torque to a specified value if it is in Torque mode.

        """
        self.robotCommand().setJointPosition(self.robotState().getIpoJointPosition())

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques)

    def command(self):
        """
        Updates the robot's joint position to match its IPO (in-positioning
        operation) trajectory and applies a torque offset to specific joints based
        on a sinusoidal pattern, with an adjustable amplitude and phase.

        """
        self.robotCommand().setJointPosition(self.robotState().getIpoJointPosition())

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            offset = self.torque_ampl * math.sin(self.phi)
            self.phi += self.step_width

            if self.phi >= 2 * math.pi:
                self.phi -= 2 * math.pi

            self.torques[self.joint_mask] = offset

            self.robotCommand().setTorque(self.torques)


def get_arguments():
    """
    Parses command line arguments for a KUKA Sunrise Controller program, including
    hostname, port number, joint to move, and parameters for sine wave generation.
    It validates input using type conversions and range checks, raising exceptions
    on invalid values.

    Returns:
        argparseNamespace: A container object holding and providing access to the
        arguments passed in as positional and optional (named) arguments parsed
        from the command line or other input source.

    """
    def cvt_joint_mask(value):
        """
        Converts a given value into an integer that represents a joint mask value
        within a specified range (0 to 7). It raises an error if the input is
        outside this range. The conversion is implicit for values already in the
        valid range.

        Args:
            value (Union[int, str]): Specified as an input to the function when
                it is called. It may be either an integer or a string representation
                of an integer that can be converted to an integer.

        Returns:
            int|None: The index of a joint mask. If the input value is within the
            range [0, 7), it returns the integer value. Otherwise, it raises an
            ArgumentTypeError with a message describing the valid range.

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
    Runs a FRI client application, connecting to a KUKA Sunrise controller over a
    specified port and hostname. It then enters a loop where it steps through the
    application until the session becomes idle or interrupted by a keyboard interrupt.

    Returns:
        int: 1 if connection to KUKA Sunrise controller fails, and 0 otherwise.
        The specific return value indicates whether the function executed successfully
        or encountered an error.

    """
    print("Running FRI Version:", fri.FRI_CLIENT_VERSION)

    args = get_arguments()
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
