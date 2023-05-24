import sys
import time
import math
import pyFRIClient as fri

import numpy as np


class MoveToClient(fri.LBRClient):
    def __init__(self, goal, duration):
        super().__init__()
        self.start = None
        self.goal = goal
        self.duration_pad = 1.0
        self.duration = duration

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        print(f"State changed from {old_state} to {new_state}")

        if new_state == fri.ESessionState.COMMANDING_ACTIVE:
            self.t = 0.0

    def waitForCommand(self):
        if self.start is None:
            self.start = self.robotState().getIpoJointPosition()

        self.robotCommand().setJointPosition(self.start)

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.WRENCH:
            self.robotCommand().setWrench(np.zeros(6, dtype=np.float32))
        elif self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(
                np.zeros(fri.LBRState.NUMBER_OF_JOINTS, dtype=np.float32)
            )

    def command(self):
        if self.t < self.duration:
            alpha = self.t / self.duration
            target = (1 - alpha) * self.start + alpha * self.goal
        elif self.duration <= self.t < self.duration + self.duration_pad:
            target = self.goal
        else:
            raise SystemExit

        self.robotCommand().setJointPosition(target)

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.WRENCH:
            self.robotCommand().setWrench(np.zeros(6, dtype=np.float32))
        elif self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(
                np.zeros(fri.LBRState.NUMBER_OF_JOINTS, dtype=np.float32)
            )

        self.t += self.robotState().getSampleTime()


def main():
    print("Running FRI Version:", fri.FRI_VERSION)

    configurations = {
        "candlestick": np.zeros(fri.LBRState.NUMBER_OF_JOINTS, dtype=np.float32),
        "home": np.deg2rad([0, 45, 0, -90, 0, 45, 0], dtype=np.float32),
    }

    if len(sys.argv) == 1:
        goal = configurations["candlestick"]
    elif len(sys.argv) == 2:
        try:
            goal = configurations[sys.argv[1]]
        except KeyError:
            print(f"Did not recognize configuration label '{sys.argv[1]}'")
            return 1
    else:
        assert (
            len(sys.argv) == 8
        ), "when specifying angles, you need to supply 7 values for each joint."
        goal = np.deg2rad([float(v) for v in sys.argv[1:]], dtype=np.float32)

    print("Moving to")
    for i in range(1, fri.LBRState.NUMBER_OF_JOINTS + 1):
        print(f"Joint {i}:", np.rad2deg(goal[i - 1]))

    duration = 30.0  # secs
    client = MoveToClient(goal, duration)

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
