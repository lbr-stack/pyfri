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
    Handles user input to control a joint's movement. It sets up a Pygame display
    and responds to key presses and releases to turn joints on/off, change direction,
    and adjust velocity. The current state is returned along with the maximum
    allowed velocity.

    Attributes:
        max_joint_velocity (float): Set to `np.deg2rad(5)`, which is equivalent
            to 0.0872665 radians, representing the maximum velocity allowed for a
            joint.
        joint_index (NoneType|int): Initialized to None. It keeps track of the
            currently selected joint, with indices starting from 1, incrementing
            from left to right for the number keys.
        joint_velocity (float): 0 by default. It is used to represent the speed
            at which a joint is being turned, updated based on user input from
            keys mapped to direction changes.
        key_to_dir_map (Dict[int,float]): Used to map Pygame key codes to
            floating-point values representing direction. It maps two keys (LEFT,
            RIGHT) to -1.0 and 1.0 respectively.
        key_joint_map (List[int]): Mapped to a list of integers representing keys
            on the keyboard, specifically numeric keys from 1 to 7. These keys are
            used to turn individual joints on or off.

    """
    def __init__(self):
        """
        Initializes various attributes, including display mode, maximum joint
        velocity, and key mapping dictionaries. These settings configure the
        keyboard controls for a game or interactive application. It prepares the
        environment for further interactions.

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
        Processes events from Pygame and updates the state of a joint based on
        user input, such as key presses or releases, to control its velocity. It
        also handles quit and escape events to terminate the program.

        Returns:
            Tuple[int,float]: A combination of an integer representing the index
            of the currently active joint and a float indicating the velocity of
            that joint, scaled to a maximum allowed value.

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
    Provides a teleoperation interface for a robotic arm, allowing users to control
    joint positions and torques using keyboard input and PyGame window interaction.
    It handles state changes, command processing, and torque application accordingly.

    Attributes:
        keyboard (Callable[[],Tuple[int|None,float]]): Associated with a keyboard
            input handling mechanism, likely related to PyGame or similar library.
            It returns a joint index and velocity goal upon key press.
        torques (npndarray[float32]): Initialized to zeros with a length equal to
            the number of joints, as defined by fri.LBRState.NUMBER_OF_JOINTS.

    """
    def __init__(self, keyboard):
        """
        Initializes an instance by calling its superclass's constructor, assigning
        the keyboard object to an attribute, and initializing a torques array with
        zeros based on the LBRState NUMBERS_OF_JOINTS constant.

        Args:
            keyboard (fri.Keyboard | None): Used to store the keyboard object
                passed from the environment.

        """
        super().__init__()
        self.keyboard = keyboard
        self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        """
        Prints state change messages and performs specific actions when the robot
        session enters monitoring-ready state, such as resetting joint torques and
        printing control instructions.

        Args:
            old_state (fri.ESessionState | None): Used to represent the state of
                the robot session before the change occurred. It stores the previous
                state when the new state changes.
            new_state (Enum[fri.ESessionState]): Set to fri.ESessionState.MONITORING_READY
                upon state change, indicating that the robot is now in monitoring
                mode ready to execute tasks.

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
        Synchronizes the robot's state with its commanded state by setting joint
        positions and torque values based on current IPo (Inverse Positioning
        Operation) joint position data from the robot.

        """
        self.q = self.robotState().getIpoJointPosition()
        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))

    def command(self):
        """
        Updates the robot's joint positions and torques based on user input from
        the keyboard. The new joint position is calculated by adding a velocity
        goal to the current joint angle, then sent to the robot for execution.

        """
        joint_index, vgoal = self.keyboard()

        if isinstance(joint_index, int):
            self.q[joint_index] += self.robotState().getSampleTime() * vgoal

        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))


def args_factory():
    """
    Parses command-line arguments using argparse. It creates a parser, defines two
    optional arguments (hostname and port) with default values, and returns the
    parsed arguments as an object containing these values.

    Returns:
        argparseNamespace: An object that holds all arguments passed to the script.
        It contains key-value pairs for hostname and port, among others specified
        by add_argument calls.

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
    Initializes a FRI client application, connects to a KUKA Sunrise controller,
    and runs an infinite loop where it continuously checks for robot state changes
    until the session becomes idle or interrupted by the user.

    Returns:
        int|str: 1 on failure and 0 on success indicating whether the connection
        to KUKA Sunrise controller was established or not.

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
