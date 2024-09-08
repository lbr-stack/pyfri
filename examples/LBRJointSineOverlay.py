import argparse
import math
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import pyfri as fri


class LBRJointSineOverlayClient(fri.LBRClient):
    def __init__(self, joint_mask, freq_hz, ampl_rad, filter_coeff):
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
        print(f"Changed state from {old_state} to {new_state}")
        if new_state == fri.ESessionState.MONITORING_READY:
            self.offset = 0.0
            self.phi = 0.0
            self.step_width = (
                2 * math.pi * self.freq_hz * self.robotState().getSampleTime()
            )

    def waitForCommand(self):
        self.robotCommand().setJointPosition(
            self.robotState().getIpoJointPosition().astype(np.float32)
        )

    def command(self):
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
    def cvt_joint_mask(value):
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
