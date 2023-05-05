import sys
import math
import pyFRIClient as fri


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

    def onStateChange(self, old_state, new_state):
        super().onStateChange(old_state, new_state)

        if new_state == fri.ESessionState.MONITORING_READY:
            self.offset = 0.0
            self.phi = 0.0
            self.step_idth = (
                2 * math.pi * self.freq_hz * self.robotState().getSampleTime()
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

        for i in range(fri.LBRState.NUMBER_OF_JOINTS):
            if self.joint_mask == i:
                joint_pos[i] += self.offset

        self.robotCommand().setJointPosition(joint_pos)


def main():
    joint_mask = 5
    freq_hz = 0.25
    ampl_rad = 0.04
    filter_coeff = 0.99
    client = LBRJointSineOverlayClient(joint_mask, freq_hz, ampl_rad, filter_coeff)

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
