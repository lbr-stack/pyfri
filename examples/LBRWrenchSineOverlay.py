import sys
import math
import pyFRIClient as fri

import numpy as np


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


def main():
    print("Running FRI Version:", fri.FRI_VERSION)

    frequencyX = 0.25
    frequencyY = 0.25
    amplitudeX = 5.0
    amplitudeY = 5.0
    client = LBRWrenchSineOverlayClient(frequencyX, frequencyY, amplitudeX, amplitudeY)

    app = fri.ClientApplication(client)

    port = 30200
    hostname = None  # i.e. use default hostname
    success = app.connect(port, hostname)

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
