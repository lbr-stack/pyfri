import argparse
import math
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import pyfri as fri


class LBRJointSineOverlayClient(fri.LBRClient):
    """
    Extends the `fri.LBRClient` to add a sine wave overlay to joint positions,
    with customizable frequency, amplitude, and filter coefficients. It synchronizes
    with the robot's state changes and commands new joint positions based on the
    calculated offset values.

    Attributes:
        joint_mask (npndarray[bool]): Used to specify which joints of a robot are
            masked for the sine overlay operation. It filters out unwanted joint
            positions from being modified by the sine wave.
        freq_hz (float): Used to specify a frequency value in Hertz, representing
            the rate at which the sine wave is generated. It is stored as a class
            instance variable.
        ampl_rad (float): Used to calculate the amplitude of a sine wave that will
            be applied to specific joints of a robot, with units of radians.
        filter_coeff (float): Used as a coefficient for a low-pass filter in the
            generation of the sine wave offset values.
            It appears to be used to smoothly ramp up or down the amplitude of the
            sine wave over time.
        offset (float|npfloat32): Updated using a low-pass filter, with its current
            value being mixed (filtered) with a new calculated offset value based
            on sine wave amplitude.
        phi (float): 0 by default, representing a phase angle incremented over
            time to generate sine wave joint position offsets. It resets when the
            session state changes to MONITORING_READY.
        step_width (float|None): Calculated based on the frequency (freq_hz),
            sample time, and 2 * pi in radians. It represents a small angle increment
            for sine wave generation.

    """
    def __init__(self, joint_mask, freq_hz, ampl_rad, filter_coeff):
        """
        Initializes its attributes, including joint mask, frequency, amplitude,
        filter coefficient, and phase offsets. It also calls the parent class's
        constructor through super().

        Args:
            joint_mask (bool | np.ndarray): 2-dimensional, where each value
                represents whether a joint should be considered or not for frequency
                filtering. It is typically used to select which joints are relevant
                for analysis.
            freq_hz (float): Associated with frequency measurement, indicating a
                signal's oscillation rate measured in Hertz (Hz).
            ampl_rad (float): Named `ampl_rad`. It represents an amplitude value
                expressed in radians, suggesting it will be used for calculations
                involving rotational or circular quantities.
            filter_coeff (float): Used as a coefficient for filtering purposes,
                likely related to a low-pass filter or a similar mathematical
                operation involving signal processing.

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
        Resets certain parameters when the robot session enters the MONITORING_READY
        state, preparing it for sine wave motion generation. It also updates a
        step width calculation based on frequency and sample time.

        Args:
            old_state (float | int): An enumeration member from fri.ESessionState.
                It represents the previous state of the session before it was
                changed to the new state passed as `new_state`.
            new_state (EnumMember[fri.ESessionState] | None): Implicitly a state
                variable representing a session state, updated from an old state,
                indicating a change in the system's operational status.

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
        Sets the joint positions of a robot to match current Ipo joint positions
        using numpy float32 data type, presumably preparing the robot for a commanded
        action or synchronization with a sine wave motion.

        """
        self.robotCommand().setJointPosition(
            self.robotState().getIpoJointPosition().astype(np.float32)
        )

    def command(self):
        """
        Updates the joint position of a robot by applying a sinusoidal offset to
        each specified joint, filtered using an exponential decay coefficient. The
        updated joint positions are then sent as a command to the robot.

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
    Defines a command-line interface for an application using the `argparse`
    library. It creates arguments for various parameters, such as hostnames, ports,
    joint masks, and filter coefficients, along with their data types and default
    values.

    Returns:
        argparseNamespace: A collection of arguments parsed from command-line
        input, including their values and help information. This object can be
        accessed like an attribute, e.g., `args.hostname`.

    """
    def cvt_joint_mask(value):
        """
        Converts an input value to an integer and checks if it falls within a
        specified range of 0 to 6. If the value is valid, it returns the corresponding
        integer; otherwise, it raises an error with a descriptive message.

        Args:
            value (Union[int, str]): Required as an argument when this function
                is called from the command line using argparse. It represents a
                joint mask value to be converted.

        Returns:
            int: A representation of the joint position within a predefined range
            of [0, 7) if it falls within this range. Otherwise, it raises an error.
            The returned integer corresponds to one of seven possible positions.

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
    Initializes and controls a robot client application, allowing it to connect
    to a KUKA Sunrise controller and perform an LBR Joint Sine Overlay test while
    collecting data for analysis and plotting after successful completion.

    Returns:
        int: 0 if the program completes normally and 1 if a connection to the KUKA
        Sunrise controller fails.

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
