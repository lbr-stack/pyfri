import argparse
import sys

# FRI Client: https://github.com/cmower/FRI-Client-SDK_Python
import pyfri as fri
# PyGame: https://www.pygame.org/news
import pygame

pygame.init()

# NumPy: https://numpy.org/
import numpy as np


class Keyboard:
    """
    Handles keyboard input to control a joint's direction and velocity. It tracks
    which joint is currently active, updates its velocity based on key presses,
    and returns the current joint index and its updated velocity for further processing.

    Attributes:
        max_joint_velocity (float): 5 degrees converted to radians using NumPy's
            `deg2rad` function, limiting the maximum speed at which a joint can
            be rotated.
        joint_index (NoneType|int): Used to track which joint (out of a maximum
            of six) is currently active for movement. It's initialized as None and
            updated when a specific key is pressed or released.
        joint_velocity (float): 0 by default. It represents the velocity of a
            joint, updated based on user input from the keyboard with directions
            mapped to specific keys. The maximum allowed value is determined by `max_joint_velocity`.
        key_to_dir_map (Dict[int,float]): Initialized with two key-value pairs.
            It maps Pygame keys to numerical values, specifically left arrow (1.0)
            and right arrow (-1.0), used for joint rotation control.
        key_joint_map (List[pygameK_]): Defined with values from 1 to 7 on keys
            K_1, K_2, ..., K_7, used as a mapping between keyboard keys and joint
            indices.

    """
    def __init__(self):
        """
        Initializes Pygame display, sets up joint movement parameters, and defines
        mappings for keyboard input to control joystick direction and selection
        of joints. It configures internal state variables with default values.

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
        Processes Pygame events to control the movement of a joint in real-time
        simulation. It handles QUIT, KEYDOWN, and KEYUP events to toggle joint
        activation, change direction, and update joint velocity based on user input.

        Returns:
            tuple[int,float]: 2-element list containing an integer and a float.
            The integer represents the currently selected joint index, or None if
            no joint is selected. The float represents the joint's velocity scaled
            by its maximum allowed value.

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
    Initializes a client for teleoperation of a robot, interacting with keyboard
    input to command joint positions and torques, handling state changes, and
    updating commands based on keyboard input and current robot state.

    Attributes:
        keyboard (object): Used to retrieve joint index and velocity goal from
            user input, presumably through a graphical interface or a library like
            PyGame.
        torques (numpyndarray[float32]): Initialized as a zeros array with a length
            of fri.LBRState.NUMBER_OF_JOINTS, which represents the number of joints
            in the robot. It stores torques values for each joint.

    """
    def __init__(self, keyboard):
        """
        Initializes an object by setting its keyboard attribute to the passed
        keyboard argument and creating a vector of zeros representing joint torques
        with the specified number of joints from the LBRState class.

        Args:
            keyboard (object): Set as an attribute of the class instance, referring
                to an external keyboard device or its interface that interacts
                with the robotic system.

        """
        super().__init__()
        self.keyboard = keyboard
        self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        """
        Updates the state of the robot and initializes data structures for control
        when the robot transitions into monitoring-ready state.

        Args:
            old_state (fri.ESessionState | None): An indication of the robot
                session's previous state before the change occurred. It represents
                the old state of the system.
            new_state (fri.ESessionState): Checked to determine whether it equals
                MONITORING_READY. This state indicates that the robot is ready for
                control.

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
        Updates the robot's joint positions and torques based on received commands
        from a client, ensuring consistency between joint positions and torque
        values when the client is in TORQUE mode.

        """
        self.q = self.robotState().getIpoJointPosition()
        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))

    def command(self):
        """
        Executes user input to control the robot's joints and apply torques based
        on current state and desired goals, updating the robot's joint positions
        and torque commands accordingly.

        """
        joint_index, vgoal = self.keyboard()

        if isinstance(joint_index, int):
            self.q[joint_index] += self.robotState().getSampleTime() * vgoal

        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))


def args_factory():
    """
    Constructs an argument parser using the `argparse` library and returns the
    parsed command line arguments. It defines two arguments: `hostname` with a
    default value of `None`, and `port` as an integer with a default value of `30200`.

    Returns:
        argparseNamespace: An object that holds all the arguments passed through
        the command-line interface, parsed by the ArgumentParser instance.

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
    Establishes a connection to a KUKA Sunrise controller, allowing for teleoperation
    and control of the robot through keyboard input. It runs indefinitely until
    interrupted by user input or system exit, then disconnects and prints a farewell
    message before returning successfully.

    Returns:
        int: 1 if the connection to KUKA Sunrise controller fails and 0 otherwise,
        indicating successful execution or an intentional exit due to a keyboard
        interrupt.

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
