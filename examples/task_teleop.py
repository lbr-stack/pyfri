import argparse
import sys
from collections import OrderedDict

# PyGame: https://www.pygame.org/news
import pygame

# FRI Client: https://github.com/cmower/FRI-Client-SDK_Python
import pyfri as fri

pygame.init()

# NumPy: https://numpy.org/
import numpy as np

np.set_printoptions(precision=5, suppress=True, linewidth=1000)

# Local scripts
from ik import IK


def print_instructions():
    """
    Displays a block of text explaining how to control robot joints using keyboard
    keys and PyGame window focus. The output is formatted with line separators and
    dashes for visual clarity.

    """
    print("\n")
    print("-" * 65)
    print("-- Control robot joints using LEFT/RIGHT direction keys.       --")
    print("-- Press keys x, y, z, r, p, a to enable a specific task axis. --")
    print("-- The PyGame window must be in focus.                         --")
    print("-" * 65, end="\n\n\n")


class Keyboard:
    """
    Handles user input from a Pygame application. It monitors for key presses and
    releases to control tasks and their velocities. The class maps keyboard keys
    to task labels or direction values, enabling users to interact with tasks using
    specific keys.

    Attributes:
        max_task_velocity (float): 0.04. This value represents the maximum velocity
            with which tasks can be executed based on user input via keyboard. It
            scales user input to a specific range of values for task execution.
        task_index (NoneType|int): Used to keep track of the current task being
            controlled by the user, with a value of None indicating that no task
            is currently selected.
        task_velocity (float): 0.0 by default. It accumulates change based on key
            presses or releases from keys assigned to direction changes, with a
            maximum limit set by `self.max_task_velocity`.
        key_to_dir_map (Dict[int,float]): Initialized with two key-value pairs,
            mapping Pygame keyboard keys to direction values. It maps LEFT arrow
            key to a value of -1.0 and RIGHT arrow key to a value of 1.0.
        key_task_map (OrderedDict[pygameK_x|pygameK_y||pygameK_a,str]): Used to
            map keys on the keyboard to corresponding tasks, represented by string
            labels such as "x", "y", etc.

    """
    def __init__(self):
        """
        Initializes pygame display, sets task velocity and index to default values,
        defines key-to-direction mapping, creates an ordered dictionary mapping
        keys to tasks, and assigns various keys to corresponding task mappings.

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
        Handles Pygame events to manage task execution and velocity control based
        on keyboard input. It monitors key presses, turns tasks on or off, and
        adjusts task velocity by adding or subtracting direction values when keys
        are pressed or released.

        Returns:
            Tuple[Optional[int],float]: 2-element tuple containing:
            
            * The index of currently active task (or None if no task is active)
            * A float representing current task velocity scaled by max allowed velocity.

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
    Implements a teleoperation interface for controlling a robotic arm, using
    inverse kinematics to calculate joint positions and torques based on user input
    from a keyboard or other external source.

    Attributes:
        ik (Callable[[npndarray,npndarray,float],npndarray]): Used to compute the
            inverse kinematics (IK) of a robot arm. It takes as input the current
            joint positions, desired joint velocities, and sample time, and returns
            the new joint positions.
        keyboard (object): Assumed to be an instance of a class that manages
            keyboard input.
        torques (npndarray[float32]): Initialized with zeros to represent joint
            torques. It stores the torques for each of the LBR robot's joints,
            which are set based on the client command mode.

    """
    def __init__(self, ik, keyboard):
        """
        Initializes an instance by setting its attributes: ik (inverse kinematics),
        keyboard, and torques as zero vectors with a specific number of joints.
        It also calls the parent class's constructor using super().__init__.

        Args:
            ik (object): Assigned to an instance variable of the same name, self.ik.
                Its purpose or contents are not specified by this code snippet.
            keyboard (object): Referenced as an attribute of instances from this
                class.

        """
        super().__init__()
        self.ik = ik
        self.keyboard = keyboard
        self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        """
        Resets its internal state when the robot reaches a specific monitoring
        ready state, including clearing an active task queue and resetting joint
        torques to zero.

        Args:
            old_state (fri.ESessionState | None): Specified by the function's
                signature, indicating that it can take on any value from the enum
                fri.ESessionState or be None. Its purpose is to represent the
                previous state of the system before the change.
            new_state (enum.Enum): Used to track the state of a process, specifically
                the ESessionState.MONITORING_READY state, which represents when
                monitoring is ready. It's expected to be one of the values from
                an enumeration called fri.ESessionState.

        """
        print(f"State changed from {old_state} to {new_state}")

        if new_state == fri.ESessionState.MONITORING_READY:
            self.q = None
            self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

            print_instructions()

    def waitForCommand(self):
        """
        Updates the robot's joint positions based on the current IPO position and
        applies torque if the client command mode is set to TORQUE. It sends these
        commands to the robot.

        """
        self.q = self.robotState().getIpoJointPosition()
        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))

    def command(self):
        """
        Generates joint positions and/or torques for an LBR robot, based on user
        input from the keyboard. It then sends these commands to the robot, using
        either position or torque control mode depending on its current configuration.

        """
        task_index, vgoal = self.keyboard()

        if isinstance(task_index, int):
            vg = np.zeros(len(self.keyboard.key_task_map))
            vg[task_index] = vgoal

            self.q = self.ik(self.q, vg, self.robotState().getSampleTime())

        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))


def args_factory():
    """
    Parses command line arguments into a namespace object using the ArgumentParser
    class from the argparse module. It sets up and validates options for a KUKA
    Sunrise Controller connection, including hostname, port, and LBR Med version
    number.

    Returns:
        argparseNamespace: A container object that holds data passed to a script
        by way of its command-line arguments. The actual content varies based on
        the provided arguments.

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
    Initializes a connection to a KUKA Sunrise controller, establishes a teleoperation
    client application, and enters a loop where it continuously receives and
    processes robot commands until idle or interrupted.

    Returns:
        int|None: 1 if connection to KUKA Sunrise controller fails or None when
        interrupted by a system exit or keyboard interrupt. Otherwise, it returns
        0 upon successful execution and disconnection from the controller.

    """
    print("Running FRI Version:", fri.FRI_CLIENT_VERSION)

    args = args_factory()
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
