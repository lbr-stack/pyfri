import sys
import math
import pyFRIClient as fri

import numpy as np


class LBRTorqueSineOverlayClient(fri.LBRClient):
    def __init__(self, joint_mask, freq_hz, torque_amplitude):
        super().__init__()
        self.joint_mask = joint_mask
        self.freq_hz = freq_hz
        self.torque_ampl = torque_amplitude
        self.phi = 0.0
        self.step_width = 0.0
        self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        print(f"State changed from {old_state} to {new_state}")

        if new_state == fri.ESessionState.MONITORING_READY:
            self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)
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


def main():
    print("Running FRI Version:", fri.FRI_VERSION)

    joint_mask = 3
    freq_hz = 0.25
    torque_amplitude = 15.0
    client = LBRTorqueSineOverlayClient(joint_mask, freq_hz, torque_amplitude)

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
