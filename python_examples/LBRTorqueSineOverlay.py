import sys
import math
import pyFRIClient as fri


class LBRTorqueSineOverlayClient(fri.LBRClient):
    def __init__(self, joint_mask, freq_hz, torque_amplitude):
        super().__init__()
        self.joint_mask = joint_mask
        self.freq_hz = freq_hz
        self.torque_ampl = torque_amplitude
        self.phi = 0.0
        self.step_width = 0.0
        self.torques = [0.0] * fri.LBRState.NUMBER_OF_JOINTS

    def onStateChange(self, old_state, new_state):
        super().onStateChange(old_state, new_state)

        if new_state == fri.ESessionState.MONITORING_READY:
            self.torques = [0.0] * fri.LBRState.NUMBER_OF_JOINTS
            self.phi = 0.0
            self.step_width = (
                2 * math.pi * self.freq_hz * self.robotState().getSampleTime()
            )

    def waitForCommand():
        super().waitForCommand()
        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques)

    def command():
        super().command()

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            offset = self.torque_ampl * math.sin(self.phi)
            self.phi += self.step_width

            if self.phi >= 2 * math.pi:
                self.phi -= 2 * math.pi

            for i in range(fri.LBRState.NUMBER_OF_JOINTS):
                if self.joint_mask == i:
                    self.torques[i] = offset

            self.robotCommand().setTorque(self.torques)


def main():
    joint_mask = 5
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

            if trafo_client.robotState().getSessionState() == fri.ESessionState.IDLE:
                break

    except KeyboardInterrupt:
        pass

    finally:
        app.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
