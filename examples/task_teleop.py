import sys
from collections import OrderedDict

# FRI Client: https://github.com/cmower/FRI-Client-SDK_Python
import pyFRIClient as fri

# PyGame: https://www.pygame.org/news
import pygame

pygame.init()

# NumPy: https://numpy.org/
import numpy as np

np.set_printoptions(precision=5, suppress=True, linewidth=1000)

# Local scripts
from ik import IK


def print_instructions():
    print("\n")
    print("-" * 65)
    print("-- Control robot joints using LEFT/RIGHT direction keys.       --")
    print("-- Press keys x, y, z, r, p, a to enable a specific task axis. --")
    print("-- The PyGame window must be in focus.                         --")
    print("-" * 65, end="\n\n\n")


class Keyboard:
    def __init__(self):
        pygame.display.set_mode((300, 300))

        self.max_task_velocity = 0.04

        self.task_index = None
        self.task_velocity = 0.0

        self.key_to_dir_map = {
            pygame.K_LEFT: 1.0,
            pygame.K_RIGHT: -1.0,
        }

        self.key_task_map = OrderedDict()
        self.key_task_map[pygame.K_x] = "x"
        self.key_task_map[pygame.K_y] = "y"
        self.key_task_map[pygame.K_z] = "z"
        self.key_task_map[pygame.K_r] = "rx"
        self.key_task_map[pygame.K_p] = "ry"
        self.key_task_map[pygame.K_a] = "rz"

    def __call__(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                # User closed pygame window -> shutdown
                pygame.quit()
                raise SystemExit

            if event.type == pygame.KEYDOWN:
                # Keydown event

                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    raise SystemExit

                if event.key in self.key_task_map:
                    task_index = list(self.key_task_map.keys()).index(event.key)
                    task_label = self.key_task_map[event.key]

                    if self.task_index is None:
                        self.task_index = task_index
                        print(f"Turned ON task {task_label}")
                    else:
                        if task_index == self.task_index:
                            print(f"Turned OFF task {task_label}")
                            self.task_index = None

                elif event.key in self.key_to_dir_map:
                    self.task_velocity += self.key_to_dir_map[event.key]

            elif event.type == pygame.KEYUP:
                # Keyup event

                if event.key in self.key_to_dir_map:
                    self.task_velocity -= self.key_to_dir_map[event.key]

        return self.task_index, self.max_task_velocity * self.task_velocity


class TeleopClient(fri.LBRClient):
    def __init__(self, ik, keyboard):
        super().__init__()
        self.ik = ik
        self.keyboard = keyboard
        self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        print(f"State changed from {old_state} to {new_state}")

        if new_state == fri.ESessionState.MONITORING_READY:
            self.q = None
            self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

            print_instructions()

    def waitForCommand(self):
        self.q = self.robotState().getIpoJointPosition()
        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))

    def command(self):
        task_index, vgoal = self.keyboard()

        if isinstance(task_index, int):
            vg = np.zeros(len(self.keyboard.key_task_map))
            vg[task_index] = vgoal

            self.q = self.ik(self.q, vg, self.robotState().getSampleTime())

        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))


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

    ik = IK(lbr_med_num)

    keyboard = Keyboard()

    client = TeleopClient(ik, keyboard)
    app = fri.ClientApplication(client)

    port = 30200
    hostname = None  # i.e. use default hostname
    success = app.connect(port, hostname)

    if not success:
        print("Connection to KUKA Sunrise controller failed.")
        return 1

    print("Connection to KUKA Sunrise controller established.")

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
