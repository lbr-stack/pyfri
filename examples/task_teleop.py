import argparse
import sys
from collections import OrderedDict

# PyGame: https://www.pygame.org/news
import pygame

# FRI Client: https://github.com/cmower/FRI-Client-SDK_Python
import pyFRI as fri

pygame.init()

# NumPy: https://numpy.org/
import numpy as np

np.set_printoptions(precision=5, suppress=True, linewidth=1000)

# Local scripts
from ik import IK


def print_instructions():
    """
    Prints a set of instructions to control a robot using direction keys and task
    axes on the PyGame window.

    """
    print("\n")
    print("-" * 65)
    print("-- Control robot joints using LEFT/RIGHT direction keys.       --")
    print("-- Press keys x, y, z, r, p, a to enable a specific task axis. --")
    print("-- The PyGame window must be in focus.                         --")
    print("-" * 65, end="\n\n\n")


class Keyboard:
    """
    Handles keyboard events to control tasks with velocities. It maps keys to task
    indices and velocities, enabling task activation/deactivation and velocity
    adjustments using arrow keys or dedicated key shortcuts. It also supports quit
    event handling.

    Attributes:
        max_task_velocity (float): 0.04 by default. It represents the maximum
            velocity for a task, which can be scaled down using the `self.task_velocity`
            attribute when keys are held to control task movement speed.
        task_index (None|int): Set to a specific key when it is pressed and its
            corresponding task label is turned ON or OFF, otherwise, its value
            remains unchanged until the corresponding key is pressed again.
        task_velocity (float): 0.0 by default. It accumulates changes based on
            keyboard input, indicating how fast a task should be executed with
            positive values representing increasing speed and negative values
            representing decreasing speed.
        key_to_dir_map (Dict[pygameK_LEFT,float]|Dict[pygameK_RIGHT,float]):
            Initialized with mappings from pygame keyboard keys to floating point
            numbers representing direction values. The map assigns a value of 1.0
            to the LEFT key and -1.0 to the RIGHT key.
        key_task_map (OrderedDict[pygameK_x|pygameK_y||pygameK_a,str]): Initialized
            to store key-value pairs where keys are specific keyboard keys (e.g.,
            pygame.K_x) and values are corresponding task labels (e.g., "x").

    """
    def __init__(self):
        """
        Initializes game window settings, defines task velocity and mapping, sets
        initial key-task mappings, and establishes relationships between keyboard
        keys and their corresponding tasks or directions.

        """
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
        """
        Handles Pygame events to control tasks and task velocity by keyboard input,
        allowing users to turn tasks on/off with specific keys and adjust
        direction/velocity with others. It also exits the program upon QUIT or
        ESCAPE key press.

        Returns:
            Tuple[int,float]: A tuple containing two values: the current task index
            and the scaled velocity of the task. The task index and velocity are
            computed based on user input events.

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
    """
    Handles robot teleoperation, monitoring its state and sending commands to the
    robot based on user input from a keyboard or other interface. It implements a
    kinematic inverse solution for joint position updates.

    Attributes:
        ik (Callable[[npndarray,npndarray,float],npndarray]): Used to perform
            inverse kinematics calculations, mapping desired joint velocities to
            joint positions. Its implementation is not provided in this code snippet.
        keyboard (Keyboard|friLBRClient): Used to get tasks from a keyboard input,
            where it returns a task index and velocity goal for each key pressed.
            It appears to be responsible for translating user input into robot commands.
        torques (npndarray[float32]): Initialized with zeros, representing the
            torques applied to each joint of a robot arm. It is used to set torque
            commands when the client command mode is set to TORQUE.

    """
    def __init__(self, ik, keyboard):
        """
        Initializes an instance with input kinematics (ik) and keyboard controls.
        It sets up the LBR state, storing zeros for torques, indicating no initial
        torque.

        Args:
            ik (fri.IK): Initialized as an attribute `self.ik`. The exact nature
                of this IK object is not provided, but it is assumed to be related
                to inverse kinematics calculations.
            keyboard (object): Assigned to an instance variable named `self.keyboard`.

        """
        super().__init__()
        self.ik = ik
        self.keyboard = keyboard
        self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        """
        Prints state change notifications and updates internal variables when the
        client enters a monitoring ready state, including resetting torque values
        and queue.

        Args:
            old_state (ESessionState | str): Used to store the previous state of
                the session before it changes.
            new_state (fri.ESessionState | Enum): Used to determine whether an
                instruction should be printed, specifically when it reaches the
                'MONITORING_READY' state.

        """
        print(f"State changed from {old_state} to {new_state}")

        if new_state == fri.ESessionState.MONITORING_READY:
            self.q = None
            self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

            print_instructions()

    def waitForCommand(self):
        """
        Synchronizes joint positions and torque values with the robot's state,
        ensuring that any commands sent by the client are applied to the robot's
        current configuration. It sets joint positions and torque values according
        to the provided data types.

        """
        self.q = self.robotState().getIpoJointPosition()
        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))

    def command(self):
        """
        Updates the robot's joint positions and torques based on user input from
        the keyboard, utilizing inverse kinematics to compute optimal joint values
        for a specific task index and corresponding velocity goal.

        """
        task_index, vgoal = self.keyboard()

        if isinstance(task_index, int):
            vg = np.zeros(len(self.keyboard.key_task_map))
            vg[task_index] = vgoal

            self.q = self.ik(self.q, vg, self.robotState().getSampleTime())

        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))


def get_arguments():
    """
    Parses command line arguments using the argparse module to provide a structured
    way to retrieve input parameters for a KUKA Sunrise Controller communication
    script, such as hostname, port, and LBR Med version number.

    Returns:
        argparseNamespace: An object containing attributes for each argument passed
        to it.

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
    parser.add_argument(
        "--lbr-ver",
        dest="lbr_ver",
        type=int,
        choices=[7, 14],
        required=True,
        help="The KUKA LBR Med version number.",
    )

    return parser.parse_args()


def main():
    """
    Establishes a connection to a KUKA Sunrise controller, initializes an application
    using the client and keyboard, and enters a loop where it continuously steps
    through the application until a disconnect or idle session state is encountered.

    Returns:
        int|None: 0 on successful execution and 1 if connection to KUKA Sunrise
        controller fails.

    """
    print("Running FRI Version:", fri.FRI_CLIENT_VERSION)

    args = get_arguments()
    ik = IK(args.lbr_ver)
    keyboard = Keyboard()
    client = TeleopClient(ik, keyboard)
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
