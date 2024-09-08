import argparse
import math
import sys

import numpy as np

import pyfri as fri


class LBRTorqueSineOverlayClient(fri.LBRClient):
    def __init__(self, joint_mask, freq_hz, torque_amplitude):
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
        print(f"State changed from {old_state} to {new_state}")

        if new_state == fri.ESessionState.MONITORING_READY:
            self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS, dtype=np.float32)
            self.phi = 0.0
            self.step_width = (
                2 * math.pi * self.freq_hz * self.robotState().getSampleTime()
            )

    def waitForCommand(self):
        self.robotCommand().setJointPosition(self.robotState().getIpoJointPosition())

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques)

    def command(self):
        self.robotCommand().setJointPosition(self.robotState().getIpoJointPosition())

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            offset = self.torque_ampl * math.sin(self.phi)
            self.phi += self.step_width

            if self.phi >= 2 * math.pi:
                self.phi -= 2 * math.pi

            self.torques[self.joint_mask] = offset

            self.robotCommand().setTorque(self.torques)


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
        "--torque-amplitude",
        dest="torque_amplitude",
        type=float,
        default=15.0,
        help="Applitude of the sine wave.",
    )

    return parser.parse_args()


def main():
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
