import sys
import math
import pyFRIClient as fri
from pyFRIClient.tools import WrenchEstimatorTaskOffset, WrenchEstimatorJointOffset

from admittance import AdmittanceController

import numpy as np

if fri.FRI_VERSION_MAJOR == 1:
    POSITION = fri.EClientCommandMode.POSITION
elif fri.FRI_VERSION_MAJOR == 2:
    POSITION = fri.EClientCommandMode.JOINT_POSITION


class HandGuideClient(fri.LBRClient):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.wrench_estimator = WrenchEstimatorTaskOffset(
            self,
            self.controller.robot,
            self.controller.ee_link,
            smooth=0.02,
        )
        # self.wrench_estimator = WrenchEstimatorJointOffset(
        #     self,
        #     self.controller.ee_link,
        #     smooth=0.02,
        # )

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        print(f"State changed from {old_state} to {new_state}")

    def waitForCommand(self):
        if self.robotState().getClientCommandMode() != POSITION:
            print(
                f"[ERROR] hand guide example requires {POSITION.name} client command mode"
            )
            raise SystemExit

        self.wrench_estimator.update()
        self.q = self.robotState().getIpoJointPosition()
        self.robotCommand().setJointPosition(self.q.astype(np.float32))

    def command(self):
        if not self.wrench_estimator.ready():
            self.wrench_estimator.update()
            self.robotCommand().setJointPosition(self.q.astype(np.float32))
            return

        # Get robot state
        wr = self.wrench_estimator.get_wrench_estimate()
        dt = self.robotState().getSampleTime()

        # Compute goal using admittance controller
        qg = self.controller(self.qc, wr, dt)

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

    controller = AdmittanceController(lbr_med_num)
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

    except SystemExit:
        pass

    finally:
        app.disconnect()
        print("Goodbye")

    return 0


if __name__ == "__main__":
    sys.exit(main())
