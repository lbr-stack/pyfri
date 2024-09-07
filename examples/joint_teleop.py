import sys

# PyGame: https://www.pygame.org/news
import pygame

# FRI Client: https://github.com/cmower/FRI-Client-SDK_Python
import pyfri as fri

pygame.init()

# NumPy: https://numpy.org/
import numpy as np


class Keyboard:
    """
    Initializes a pygame display and sets up event handling for keyboard input.
    It manages joint activation and direction control using keys 1-7 and left/right
    arrow keys, respectively, with velocity limits.

    Attributes:
        max_joint_velocity (float): 5 degrees in radians converted to a radian
            measure using numpy's `deg2rad` function, limiting the maximum velocity
            of the joint when controlled by keyboard input.
        joint_index (Optional[int]|NoneType): Initialized to None. It keeps track
            of the index of a joint that has been turned on, allowing the program
            to control multiple joints with different keys.
        joint_velocity (float|NoneType): 0.0 by default. It represents the current
            velocity of a mechanical joint, initially at rest (0.0), which can be
            changed based on user input to control the joint's movement speed.
        key_to_dir_map (Dict[pygameK_LEFT,float]|Dict[pygameK_RIGHT,float]): Used
            to map keyboard keys (LEFT, RIGHT) to direction values (1.0, -1.0).
            It determines how movement is controlled when using these keys.
        key_joint_map (List[int]): Mapped to a list of Pygame key constants,
            representing the keys on the keyboard that control the joints.

    """
    def __init__(self):
        """
        Initializes Pygame's display mode and sets up various variables for handling
        keyboard input, including joint velocity limits, direction mappings, and
        key associations with joints.

        """
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
        """
        Handles events from the pygame library, allowing the user to control joints
        and change their velocity through keyboard input. It also allows the user
        to quit the application by pressing the ESC key or closing the window.

        Returns:
            Tuple[int,float]: A tuple containing two values:
            
            1/ The index of the currently active joint (or None if no joint is active).
            2/ A scalar value representing the current velocity of all joints,
            scaled by a factor that can be accessed through self.max_joint_velocity.

        """
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
    """
    Handles teleoperation of a robot using keyboard input. It monitors the robot's
    state, receives user commands through keyboard inputs, and sends joint position
    and torque commands to the robot based on these inputs.

    Attributes:
        keyboard (Keyboard): Associated with a keyboard interface. It appears to
            be used to track user input, such as key presses or releases, that
            control robot joint movements.
        torques (npndarray[float32]): Initialized to zero with a size corresponding
            to the number of joints in the robot (7). It stores torque values for
            each joint.

    """
    def __init__(self, keyboard):
        """
        Initializes an instance with a keyboard object and sets up its attributes,
        including a torques array initialized as zeros with the number of joints
        specified by fri.LBRState.NUMBER_OF_JOINTS.

        Args:
            keyboard (object): Referenced but not defined within this snippet. It
                is expected to be an instance of a class that has been imported
                from elsewhere.

        """
        super().__init__()
        self.keyboard = keyboard
        self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        """
        Prints state change information, sets robot control parameters when the
        robot reaches a specific monitoring ready state, and prints control
        instructions to the user.

        Args:
            old_state (fri.ESessionState | str): Used to store the previous state
                before it was changed to `new_state`. It represents the previous
                session state or status of the robot system.
            new_state (Enum[fri.ESessionState]): An enumeration value representing
                the new state of the robot session, specifically the "MONITORING_READY"
                state when changed.

        """
        print(f"State changed from {old_state} to {new_state}")

        if new_state == fri.ESessionState.MONITORING_READY:
            self.q = None
            self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

            print("\n")
            print("-----------------------------------------------------------")
            print("-- Control robot joints using LEFT/RIGHT direction keys. --")
            print("-- Press keys 1, ..., 7 to enable a specific joint.      --")
            print("-- The PyGame window must be in focus.                   --")
            print(
                "-----------------------------------------------------------",
                end="\n\n\n",
            )

    def waitForCommand(self):
        """
        Waits for joint position and torque commands from the robot, then sends
        these values to the robot's state machine. The function updates the robot's
        joint positions and applies torques according to the client command mode.

        """
        self.q = self.robotState().getIpoJointPosition()
        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))

    def command(self):
        """
        Sends joint position and torque commands to a robotic arm, based on user
        input from a keyboard or other device. It updates the robot's joint positions
        and torques accordingly.

        """
        joint_index, vgoal = self.keyboard()

        if isinstance(joint_index, int):
            self.q[joint_index] += self.robotState().getSampleTime() * vgoal

        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))


def args_factory():
    """
    Parses command-line arguments using the `argparse` library. It defines two
    optional arguments: `hostname` and `port`, both with default values, to be
    used for communication with a KUKA Sunrise Controller. The parsed arguments
    are returned as an object.

    Returns:
        Namespace[hostnamestr,portint]: A named collection of arguments that were
        passed to the script. It contains the hostname and port number parsed from
        command-line arguments or default values if not provided.

    """
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

    return parser.parse_args()


def main():
    """
    Initializes a KUKA Sunrise controller teleoperation application, connects to
    it, and enters a loop where it continuously steps through the robot's state
    until idle or interrupted. It then disconnects and exits.

    Returns:
        int: 0 if the execution completes successfully and 1 otherwise, indicating
        failure to connect to the KUKA Sunrise controller.

    """
    print("Running FRI Version:", fri.FRI_CLIENT_VERSION)

    args = args_factory()
    keyboard = Keyboard()
    client = TeleopClient(keyboard)
    app = fri.ClientApplication(client)
    success = app.connect(args.port, args.hostname)

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
