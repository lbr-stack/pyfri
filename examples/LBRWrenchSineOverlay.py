import argparse
import math
import sys

import numpy as np

import pyfri as fri


class LBRWrenchSineOverlayClient(fri.LBRClient):
    def __init__(self, frequencyX, frequencyY, amplitudeX, amplitudeY):
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
        self.robotCommand().setJointPosition(self.robotState().getIpoJointPosition())
        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.WRENCH:
            self.robotCommand().setWrench(self.wrench)

    def command(self):
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
