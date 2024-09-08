import argparse
import math
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import pyfri as fri


class LBRJointSineOverlayClient(fri.LBRClient):
    """
    Generates a sinusoidal joint position offset for specific joints on an industrial
    robot, controlled by a sine wave with adjustable frequency and amplitude,
    filtering the output to smooth out oscillations.

    Attributes:
        joint_mask (numpyndarray|int): Used to specify which joints of a robot are
            affected by the sine wave overlay.
        freq_hz (float): Intended to represent the frequency of a sine wave,
            measured in Hertz (Hz). It appears to be used for generating a sinusoidal
            offset.
        ampl_rad (float): A parameter controlling the amplitude (in radians) of
            the sine wave used to modulate joint positions. It determines the
            maximum deviation from the base joint position.
        filter_coeff (float): 0 by default, used for low-pass filtering the offset
            value of the sine wave, effectively reducing its amplitude over time.
        offset (float): Initialized to 0.0. It represents a value used to modify
            joint positions based on a sine wave pattern, filtered over time with
            a specified coefficient.
        phi (float): Initialized to zero. It increments by a calculated step width
            during each command execution, simulating rotation at the specified frequency.
        step_width (float): Set in the `onStateChange` method to twice pi times
            the frequency (in Hz) times the sample time, essentially defining a
            phase increment for each sampling step.

    """
    def __init__(self, joint_mask, freq_hz, ampl_rad, filter_coeff):
        """
        Initializes its attributes, including joint mask, frequency, amplitude,
        filter coefficient, offset, phase angle, and step width from input parameters.
        It calls the parent class's `__init__` method using `super().__init__()`.

        Args:
            joint_mask (np.ndarray | List[int]): 2D with shape (n_joints, n_joints),
                representing the adjacency matrix for joints used to connect and
                filter movement data.
            freq_hz (float): Required for initialization, specifying the frequency
                in Hertz.
            ampl_rad (float): 3rd argument passed to this method, it represents
                the amplitude of an oscillation in radians. It seems to be related
                to angular measurements in a sinusoidal signal.
            filter_coeff (float): Set by the user when creating an instance of
                this class. It likely represents a coefficient used for filtering
                data, possibly as part of a low-pass or high-pass filter calculation.

        """
        super().__init__()
        self.joint_mask = joint_mask
        self.freq_hz = freq_hz
        self.ampl_rad = ampl_rad
        self.filter_coeff = filter_coeff
        self.offset = 0.0
        self.phi = 0.0
        self.step_width = 0.0

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        """
        Updates its internal state when the robot's session state changes to
        MONITORING_READY, resetting certain parameters such as offset, phi, and
        step width based on the robot's frequency and sample time.

        Args:
            old_state (fri.ESessionState): Used to store the previous state of an
                object or system before its current state changes. Its value is
                passed as an argument from another function call.
            new_state (Enum[fri.ESessionState]): Updated to MONITORING_READY on
                state change, which triggers resetting several attributes of the
                class instance.

        """
        print(f"Changed state from {old_state} to {new_state}")
        if new_state == fri.ESessionState.MONITORING_READY:
            self.offset = 0.0
            self.phi = 0.0
            self.step_width = (
                2 * math.pi * self.freq_hz * self.robotState().getSampleTime()
            )

    def waitForCommand(self):
        """
        Updates the robot's joint positions using values retrieved from an IPO
        (In-Place Optimization) calculation, converting them to float32 data type
        before applying them.

        """
        self.robotCommand().setJointPosition(
            self.robotState().getIpoJointPosition().astype(np.float32)
        )

    def command(self):
        """
        Updates the offset value based on filtered sine wave input, adjusts the
        phase angle to ensure it remains within a specific range, and sends the
        updated joint position command to the robot.

        """
        new_offset = self.ampl_rad * math.sin(self.phi)
        self.offset = (self.offset * self.filter_coeff) + (
            new_offset * (1.0 - self.filter_coeff)
        )
        self.phi += self.step_width
        if self.phi >= (2 * math.pi):
            self.phi -= 2 * math.pi
        joint_pos = self.robotState().getIpoJointPosition()
        joint_pos[self.joint_mask] += self.offset
        self.robotCommand().setJointPosition(joint_pos.astype(np.float32))


def args_factory():
    """
    Creates an argument parser for a program that interacts with a KUKA Sunrise
    Controller, accepting various parameters such as hostname, port number, joint
    mask, frequency and amplitude of a sine wave, filter coefficient, and data
    saving flag.

    Returns:
        argparseNamespace: An object that stores the parsed command line arguments
        as attributes. It encapsulates all argument values provided to the script.

    """
    def cvt_joint_mask(value):
        """
        Converts a given input value to an integer and checks if it falls within
        the specified range (0-6 inclusive). If valid, it returns the integer;
        otherwise, it raises an error with a descriptive message.

        Args:
            value (Union[int, str]): Expected to be within the range of integer
                values from 0 to 7 exclusive.

        Returns:
            int|None: Integer value within the range [0, 7) or raises an error if
            not within this specified range.

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
        "--ampl-rad",
        dest="ampl_rad",
        type=float,
        default=0.04,
        help="Applitude of the sine wave.",
    )
    parser.add_argument(
        "--filter-coeff",
        dest="filter_coeff",
        type=float,
        default=0.99,
        help="Exponential smoothing coeficient.",
    )
    parser.add_argument(
        "--save-data",
        dest="save_data",
        action="store_true",
        default=False,
        help="Set this flag to save the data.",
    )

    return parser.parse_args()


def main():
    """
    Initializes a KUKA Sunrise controller client, collects data and performs a
    joint sine overlay test. It handles connections, disconnections, and data
    saving, then displays a plot of collected data using matplotlib.

    Returns:
        int|None: 0 when no exceptions occur and when the application runs
        successfully, otherwise it returns a non-zero value, specifically 1 if the
        connection to KUKA Sunrise controller fails.

    """
    print("Running FRI Version:", fri.FRI_CLIENT_VERSION)

    args = args_factory()
    client = LBRJointSineOverlayClient(
        args.joint_mask, args.freq_hz, args.ampl_rad, args.filter_coeff
    )
    app = fri.ClientApplication(client)
    if args.save_data:
        app.collect_data("lbr_joint_sine_overlay.csv")
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
        if args.save_data:
            df = pd.read_csv("lbr_joint_sine_overlay.csv")

            fig, ax = plt.subplots(4, 1, sharex=True)

            dim2name = {
                "mp": "Measured Position",
                "ip": "Ipo Position",
                "mt": "Measured Torque",
                "et": "External Torque",
            }

            for i, dim in enumerate(["mp", "ip", "mt", "et"]):
                df.plot(x="time", y=[dim + str(i + 1) for i in range(7)], ax=ax[i])
                ax[i].set_ylabel(dim2name[dim])
            ax[-1].set_xlabel("Time (s)")

            plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
