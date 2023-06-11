import sys
import math
import pyFRIClient as fri

from admittance import AdmittanceController

import numpy as np


class HandGuideClient(fri.LBRClient):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        print(f"State changed from {old_state} to {new_state}")

    def waitForCommand(self):
        self.qc = self.robotState().getIpoJointPosition()
        self.robotCommand().setJointPosition(self.qc)

    def command(self):
        # Get robot state
        te = self.robotState().getExternalTorque()
        dt = self.robotState().getSampleTime()

        # Compute goal using admittance controller
        qg = self.controller(self.qc, te, dt)

        # Command robot
        self.robotCommand().setJointPosition(qg.astype(np.float32))
        self.qc = qg.copy()


def main():
    print("Running FRI Version:", fri.FRI_VERSION)

    try:
        lbr_med_num = int(sys.argv[1])
    except IndexError:
        print("You need to supply a LBR Med version number. Either 7 or 14.")
        return 1

    if lbr_med_num not in {7, 14}:
        print("You need to supply a LBR Med version number. Either 7 or 14.")
        return 1

    con_ee_z = True
    controller = AdmittanceController(lbr_med_num, con_ee_z)
    client = HandGuideClient(controller)

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
        print("Goodbye")

    return 0


if __name__ == "__main__":
    sys.exit(main())
p
