import sys

# PyGame: https://www.pygame.org/news
import pygame

# FRI Client: https://github.com/cmower/FRI-Client-SDK_Python
import pyFRI as fri

pygame.init()

# NumPy: https://numpy.org/
import numpy as np


class Keyboard:
    """
    Initializes a Pygame window and controls a virtual joint with key presses. It
    can be turned on or off by pressing numbered keys, and its direction is
    controlled by left and right arrow keys, with an escape key for exit.

    Attributes:
        max_joint_velocity (float): Used to represent the maximum allowed velocity
            for a joint, set to 5 degrees radian when initialized using the
            `np.deg2rad(5)` function.
        joint_index (NoneType|int): Initialized to None in the __init__ method.
            It stores the index of the currently active joint, which ranges from
            1 to 7 (inclusive) when one of the number keys 1-7 is pressed.
        joint_velocity (float): 0 by default. It accumulates and stores the net
            velocity change applied to a joint based on the direction keys pressed,
            bounded within a maximum limit defined by `max_joint_velocity`.
        key_to_dir_map (Dict[int,float]): Initialized with a dictionary that maps
            keys from Pygame's constant for left arrow to a float value of 1.0,
            and right arrow key to -1.0.
        key_joint_map (List[pygameK_]|pygameK_ESCAPE): 7 elements long, which maps
            keyboard keys to their corresponding joints. It stores a list of
            integers representing the key codes of the numbered keys on the keyboard
            (1-7).

    """
    def __init__(self):
        """
        Initializes several attributes for handling joint movements using keyboard
        input, including display settings, joint velocity limits, and mapping key
        presses to direction and joint values.

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
        Manages user input from Pygame events. It handles window close, keyboard
        key presses and releases, and updates joint velocity accordingly.

        Returns:
            tuple[int,float]: A pair containing an integer indicating the currently
            selected joint index and a float representing the scaled joint velocity.

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
    Enables teleoperation of a robot using keyboard input. It monitors the robot's
    state, waits for user commands, and sends control commands to the robot based
    on user input, allowing for real-time joint position or torque control.

    Attributes:
        keyboard (Callable[[],Tuple[int|None,float]]): Likely a method that returns
            the index of the joint to be controlled or None if no control is
            desired, along with the velocity goal for that joint.
        torques (npndarray[float32]|None): Initialized as a NumPy array of zeros
            with length fri.LBRState.NUMBER_OF_JOINTS when the instance is created.
            It represents joint torques in Newton meters.

    """
    def __init__(self, keyboard):
        """
        Initializes an instance with a keyboard object and sets up attributes to
        store the keyboard and torques for the LBR robot, initializing the torques
        array with zeros.

        Args:
            keyboard (object): Assigned to the instance variable `self.keyboard`.
                Its purpose is not specified within the code, but it may represent
                an external input device or interface for user interaction.

        """
        super().__init__()
        self.keyboard = keyboard
        self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        """
        Prints state changes and handles transitions to specific states, notably
        when the robot reaches the MONITORING_READY state. It resets certain
        attributes and displays instructions for controlling the robot joints using
        keyboard inputs.

        Args:
            old_state (Union[fri.ESessionState, fri.ESessionState | str]): Apparently
                an enumeration object or a string representing the previous state
                of the session.
            new_state (fri.ESessionState): Used to store the current state of the
                session. In this specific case, it can be one of the enumeration
                values defined under fri.ESessionState.

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
        Retrieves the current joint position from the robot and sets it as the new
        target position for the joints, updating the torque command if the client
        is in torque mode.

        """
        self.q = self.robotState().getIpoJointPosition()
        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))

    def command(self):
        """
        Receives user input from the keyboard and updates the robot's joint positions
        based on velocity goals, before sending updated position commands to the
        robot's control interface. It also sets torque values when operating in
        torque mode.

        """
        joint_index, vgoal = self.keyboard()

        if isinstance(joint_index, int):
            self.q[joint_index] += self.robotState().getSampleTime() * vgoal

        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))


def get_arguments():
    """
    Parses command-line arguments using `argparse`. It defines two optional
    arguments: `hostname` and `port`, which are used to specify communication
    settings with a KUKA Sunrise Controller. The parsed arguments are then returned
    as an object.

    Returns:
        argparseNamespace: An object containing the parsed command line arguments.
        This includes the values for the specified options such as hostname and port.

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
    Initializes and connects to a KUKA Sunrise controller, establishing a teleoperation
    session using keyboard input and prints messages indicating success or failure
    at each stage. It then enters a loop where it repeatedly checks for the robot's
    state until idle or interrupted.

    Returns:
        int: 1 if connection to KUKA Sunrise controller fails and 0 otherwise.
        This indicates whether the program executed successfully or not.

    """
    print("Running FRI Version:", fri.FRI_CLIENT_VERSION)

    args = get_arguments()
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
