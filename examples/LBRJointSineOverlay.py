import argparse
import math
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import pyFRI as fri


class LBRJointSineOverlayClient(fri.LBRClient):
    """
    Simulates a sine wave overlay on specific joints of a robot, with adjustable
    frequency, amplitude, and filtering. It monitors state changes to update its
    parameters and applies the simulated movement when ready. It also waits for
    commands and adjusts joint positions accordingly.

    Attributes:
        joint_mask (numpyndarray|int): Used to select a subset of joints for which
            sine overlay is applied, ignoring others.
        freq_hz (float): Initialized by the user when creating an instance of the
            class. It represents a frequency value in Hertz that determines the
            oscillation period of a sine wave overlay for joint positions.
        ampl_rad (float): Used to scale the sine wave offset values in radians.
            It controls the amplitude of the sinusoidal motion added to the joint
            positions.
        filter_coeff (float): Used as a coefficient for a simple exponential
            smoothing filter to smooth out the oscillation amplitude.
        offset (float): Initialized to 0. It represents a filtered sine wave offset
            value used to update joint positions during the command method.
        phi (float): Used as a phase angle in a mathematical function, specifically
            the sine function. It increments by `self.step_width` each time the
            command method is called.
        step_width (float): 0 by default. It calculates the angular step for sine
            wave overlay based on frequency, sample time, and is updated in `onStateChange`.

    """
    def __init__(self, joint_mask, freq_hz, ampl_rad, filter_coeff):
        """
        Initializes an object with specified parameters, including joint mask,
        frequency, amplitude, and filter coefficient, as well as additional
        attributes for offset, phase, and step width.

        Args:
            joint_mask (bool | np.ndarray): 2-dimensional. It represents joint
                (common) mask, likely used to specify which elements are valid for
                further calculations or filtering processes.
            freq_hz (float): Expected to be set to a value representing frequency
                in hertz. It initializes an instance variable `freq_hz` of the class.
            ampl_rad (float): 0.0 by default. It appears to represent an amplitude
                value in radians, suggesting it could be related to oscillatory
                behavior or wave properties.
            filter_coeff (float | int): Assigned from an unknown source. Its purpose
                is unclear without more context, but its name suggests it could
                be related to filter coefficients used in signal processing.

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
        Resets certain parameters when the robot transitions to the 'MONITORING_READY'
        state, and prints a message indicating the change in state.

        Args:
            old_state (str | Enum): Described by the documentation as "the previous
                state". It represents the state of the object before it was changed.
            new_state (Enum[fri.ESessionState]): Passed to the function when the
                state of an object changes. The specific value depends on the
                context in which the function is called.

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
        Sets the joint position of the robot based on the current Ipo joint position,
        converting it to a float32 format before applying it. It appears to be
        waiting for a command from an external source or system.

        """
        self.robotCommand().setJointPosition(
            self.robotState().getIpoJointPosition().astype(np.float32)
        )

    def command(self):
        """
        Updates the offset for joint movements, calculates the new position based
        on the updated offset and applies it to the robot's joints. It uses a
        low-pass filter to smoothly transition between new and previous offsets.

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


def get_arguments():
    """
    Parses command-line arguments for a program that communicates with a KUKA
    Sunrise Controller, including hostname, port number, joint mask, frequency,
    amplitude, filter coefficient, and data saving flag. It uses type conversion
    and error checking to validate input values.

    Returns:
        argparseNamespace: An object containing all the command line arguments
        parsed by ArgumentParser. This object has attributes for each argument
        added to the parser.

    """
    def cvt_joint_mask(value):
        """
        Converts a given value into an integer that represents a joint mask and
        checks if it falls within the specified valid range (0 to 6). If not, it
        raises an error with a descriptive message indicating the invalid value
        and the acceptable range.

        Args:
            value (Union[int, str]): Expected to be either an integer within the
                specified range or a string that can be converted to an integer
                within this range.

        Returns:
            int|None: A integer value representing joint mask index from 0 to 6
            if valid input is provided otherwise it raises an ArgumentTypeError exception.

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
    Initializes a KUKA Sunrise controller connection, runs an LBRJointSineOverlay
    client application, collects data when specified, and plots it as a 4x1 subplot
    after disconnection, displaying measured and IPO positions along with torque
    values over time.

    Returns:
        int|None: 0 on successful completion and non-zero if connection to KUKA
        Sunrise controller fails.

    """
    print("Running FRI Version:", fri.FRI_CLIENT_VERSION)

    args = get_arguments()
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
