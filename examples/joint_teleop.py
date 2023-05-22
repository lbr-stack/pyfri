import sys

# FRI Client: https://github.com/cmower/FRI-Client-SDK_Python
import pyFRIClient as fri

# PyGame: https://www.pygame.org/news
import pygame

pygame.init()

# NumPy: https://numpy.org/
import numpy as np


class Keyboard:
    def __init__(self):
        pygame.display.set_mode((300, 300))

        self.max_joint_velocity = np.deg2rad(5)

        self.joint_index = None
        self.joint_velocity = 0.0

        self.key_to_dir_map = {
            pygame.K_LEFT: 1.0,
            pygame.K_RIGHT: -1.0,
        }

        self.key_joint_map = [
            pygame.K_1,
            pygame.K_2,
            pygame.K_3,
            pygame.K_4,
            pygame.K_5,
            pygame.K_6,
            pygame.K_7,
        ]

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

                if event.key in self.key_joint_map:
                    joint_index = self.key_joint_map.index(event.key)

                    if self.joint_index is None:
                        self.joint_index = joint_index
                        print(f"Turned ON joint {self.joint_index+1}")
                    else:
                        if joint_index == self.joint_index:
                            print(f"Turned OFF joint {self.joint_index+1}")
                            self.joint_index = None

                elif event.key in self.key_to_dir_map:
                    self.joint_velocity += self.key_to_dir_map[event.key]

            elif event.type == pygame.KEYUP:
                # Keyup event

                if event.key in self.key_to_dir_map:
                    self.joint_velocity -= self.key_to_dir_map[event.key]

        return self.joint_index, self.max_joint_velocity * self.joint_velocity


class TeleopClient(fri.LBRClient):
    def __init__(self, keyboard):
        super().__init__()
        self.keyboard = keyboard
        self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        print(f"State changed from {old_state} to {new_state}")

        if new_state == fri.ESessionState.MONITORING_READY:
            self.q = None
            self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

            print("\n")
            print("-----------------------------------------------------------")
            print("-- Control robot joints using LEFT/RIGHT direction keys. --")
            print("-- Press keys 1, ..., 7 to enable a specific joint.      --")
            print(
                "-----------------------------------------------------------",
                end="\n\n\n",
            )

    def waitForCommand(self):
        self.q = self.robotState().getIpoJointPosition()
        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))

    def command(self):
        joint_index, vgoal = self.keyboard()

        if isinstance(joint_index, int):
            self.q[joint_index] += self.robotState().getSampleTime() * vgoal

        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))


def main():
    print("Running FRI Version:", fri.FRI_VERSION)

    keyboard = Keyboard()

    client = TeleopClient(keyboard)
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
