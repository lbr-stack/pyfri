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
    Sets up a pygame display and manages user input to control joint movement. It
    allows users to turn joints on/off using number keys (1-7) and adjust their
    rotation speed using arrow keys or designated keybindings.

    Attributes:
        max_joint_velocity (float): Set to a value that corresponds to a
            degree-to-radian conversion of 5 degrees, indicating the maximum allowed
            angular velocity for the joint.
        joint_index (Optional[int]|None): Used to store the index of a specific
            joint selected by the user, initially set to None but assigned when a
            joint key is pressed.
        joint_velocity (float): 0 by default. It tracks the current velocity of a
            joint, which can be increased or decreased based on user input from
            the arrow keys. Its value is capped at `max_joint_velocity`.
        key_to_dir_map (Dict[int,float]): Initialized with a dictionary mapping
            Pygame keyboard keys (integers) to their corresponding direction values
            as floats. The keys are for movement control, while the values determine
            the direction.
        key_joint_map (List[int]): Mapped to keys on the keyboard corresponding
            to joints. It contains numeric key values from K_1 to K_7 that are
            used to turn joint actions on or off by pressing the associated key.

    """
    def __init__(self):
        """
        Initializes various attributes related to keyboard input and joint control.
        It sets up a Pygame display, defines maximum joint velocity, key-to-direction
        mapping, and key-joint mappings for controlling multiple joints with
        specific keys.

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
        Handles keyboard events to control a joint's state and velocity, allowing
        the user to turn it on/off and adjust its direction using specific key combinations.

        Returns:
            Tuple[int,float]: A tuple containing two values: an integer and a float.
            The first value is either None or a joint index (an integer).
            The second value is the product of max_joint_velocity (a float) and
            joint_velocity (another float).

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
    Initializes a teleoperation client with keyboard input, monitors robot states,
    and enables users to control robotic joints using left/right arrow keys or
    specific joint numbers on the keyboard. It sends joint position commands to
    the robot based on user input.

    Attributes:
        keyboard (object): Initialized in the `__init__` method. It does not have
            any direct description within the code snippet provided, but its usage
            suggests it is related to capturing keyboard input for controlling
            robot joints.
        torques (npndarray[float32]): Initialized with zeros. It stores torque
            values for each joint, representing a set of torques that can be applied
            to control robot movements when operating in torque mode.

    """
    def __init__(self, keyboard):
        """
        Initializes an instance with a keyboard object and sets its attributes,
        including the torques of its joints to zeros. The NUMBER_OF_JOINTS constant
        is used from fri.LBRState to determine the size of the torques array.

        Args:
            keyboard (fri.Keyboard): Assumed to be an instance of the Keyboard
                class from the fri package. Its actual behavior depends on its
                implementation in that package.

        """
        super().__init__()
        self.keyboard = keyboard
        self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        """
        Prints state change notifications and sets up a control interface when the
        robot reaches the MONITORING_READY state, prompting the user to control
        robot joints using keyboard input.

        Args:
            old_state (Union[fri.ESessionState, fri.LBRState]): Used to represent
                the state of the robot before it changed to the new_state. It
                indicates the previous operational mode or configuration of the robot.
            new_state (fri.ESessionState): Checked for equality with
                fri.ESessionState.MONITORING_READY. It represents the new state
                of the robot session after a change has occurred.

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
        Waits for a new command from the robot and applies it by setting the joint
        positions and torques accordingly based on the client's current mode,
        allowing torque control when enabled.

        """
        self.q = self.robotState().getIpoJointPosition()
        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))

    def command(self):
        """
        Updates the robot's joint positions and torques based on user input from
        a keyboard, and sends the updated values to the robot for execution. It
        handles different client command modes, including torque control.

        """
        joint_index, vgoal = self.keyboard()

        if isinstance(joint_index, int):
            self.q[joint_index] += self.robotState().getSampleTime() * vgoal

        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))


def args_factory():
    """
    Parses command line arguments using argparse. It defines two arguments,
    `hostname` and `port`, to connect with a KUKA Sunrise Controller. The default
    port is set to 30200. Parsed arguments are returned as an object, allowing
    access to their values.

    Returns:
        argparseNamespace: An object holding the parsed arguments from the command
        line, including hostname and port.

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
    Initializes and connects to a KUKA Sunrise controller using the FRI (Fieldbus
    Remote Interface) protocol, then enters an infinite loop where it continuously
    checks for new commands until the session state changes or an interrupt is received.

    Returns:
        int: 0 when a connection to KUKA Sunrise controller is established, and 1
        otherwise, indicating failure. This allows the operating system or script
        that calls this function to determine whether it was successful.

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
