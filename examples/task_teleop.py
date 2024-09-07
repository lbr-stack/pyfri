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
    Displays a set of instructions to the user regarding control of robot joints
    using keyboard keys and enabling specific task axes through key presses. The
    instructions are printed to the console, emphasizing the need for focus on the
    PyGame window.

    """
    print("\n")
    print("-" * 65)
    print("-- Control robot joints using LEFT/RIGHT direction keys.       --")
    print("-- Press keys x, y, z, r, p, a to enable a specific task axis. --")
    print("-- The PyGame window must be in focus.                         --")
    print("-" * 65, end="\n\n\n")


class Keyboard:
    """
    Manages keyboard input to control a virtual task. It handles quit events, key
    presses for task selection and direction modification, and updates task velocity
    accordingly. The task index is returned along with its velocity scaled by a
    maximum velocity constant.

    Attributes:
        max_task_velocity (float): 0.04 by default. This attribute limits the
            maximum velocity at which tasks are performed. It serves as a bound
            for the task's velocity.
        task_index (NoneType|int): Used to keep track of the currently active task.
            It defaults to None, indicating that no task is currently active. When
            a key corresponding to a specific task is pressed or released, its
            value updates accordingly.
        task_velocity (float): Initialized to zero. It represents the velocity of
            a task being controlled by the keyboard input. Its value changes when
            specific keys are pressed or released.
        key_to_dir_map (Dict[int,float]): Initialized with two key-value pairs
            that map keyboard keys to direction values used for task velocity adjustments.
        key_task_map (OrderedDict[int,str]): Initialized with key-value pairs that
            map Pygame keys to task labels: x, y, z, rx, ry, rz. These tasks are
            presumably controlled by the keyboard.

    """
    def __init__(self):
        """
        Initializes several instance variables and sets up key mappings for Pygame
        events. These include task velocity, keyboard keys for directional movement
        and task assignments, and an ordered dictionary to store key-task mappings.

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
        Processes Pygame events to handle keyboard inputs and updates task velocity
        accordingly. When specific keys are pressed, tasks are toggled on or off;
        pressing movement keys modifies task direction and speed. Quitting the
        program occurs upon QUIT event.

        Returns:
            Tuple[int,float]: A pair of an integer and a float number. The first
            element of the tuple represents the current task index, while the
            second element represents the magnitude of the task's velocity scaled
            to its maximum possible velocity.

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
    Handles user input through a keyboard interface and controls a robotic arm
    using inverse kinematics and torque control, transitioning between monitoring
    ready state and issuing commands based on keyboard inputs and joint positions.

    Attributes:
        ik (callable): Associated with an instance of Inverse Kinematics solver,
            which solves the forward kinematics problem to compute joint positions
            given a target position or orientation.
        keyboard (object): Used to obtain task index and velocity goal values
            through its method call, allowing user input and control over robot actions.
        torques (npndarray[float32]): Initialized as a zero-filled array with shape
            NUMBER_OF_JOINTS, where NUMBER_OF_JOINTS is defined by fri.LBRState.NUMBER_OF_JOINTS.

    """
    def __init__(self, ik, keyboard):
        """
        Initializes object attributes: ik (inverse kinematics), keyboard, and
        torques. It sets up an array of zeros to store torques for each joint based
        on a predefined number from fri.LBRState.NUMBER_OF_JOINTS.

        Args:
            ik (InvertedKinematics): Assigned to an instance variable called
                `self.ik`. It represents an object that performs inverse kinematics
                calculations for the robot's joints.
            keyboard (object | Dict[str, Any]): Expected to be passed when creating
                an instance of this class. Its actual type or behavior is not
                specified by the code snippet provided.

        """
        super().__init__()
        self.ik = ik
        self.keyboard = keyboard
        self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        """
        Monitors state changes and triggers specific actions when entering the
        MONITORING_READY state. It resets internal variables, clears a queue, and
        initializes torques to zero before printing instructions.

        Args:
            old_state (fri.ESessionState): Passed to the function when its state
                changes, representing the previous state before the change occurred.
                Its value is used for logging purposes within the function body.
            new_state (str | Enum): Used to store the current state of an object,
                specifically the monitoring ready state of a session. It represents
                the new status after a state change event occurred.

        """
        print(f"State changed from {old_state} to {new_state}")

        if new_state == fri.ESessionState.MONITORING_READY:
            self.q = None
            self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS)

            print_instructions()

    def waitForCommand(self):
        """
        Synchronizes the robot's actual and commanded states by setting joint
        positions and torques according to the current command mode.

        """
        self.q = self.robotState().getIpoJointPosition()
        self.robotCommand().setJointPosition(self.q.astype(np.float32))

        if self.robotState().getClientCommandMode() == fri.EClientCommandMode.TORQUE:
            self.robotCommand().setTorque(self.torques.astype(np.float32))

    def command(self):
        """
        Updates the robot's joint positions and torques based on user input from
        the keyboard, converting user-defined goals into control commands for the
        robot. It ensures that the command mode is set accordingly to either
        position or torque control.

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
    Parses command line arguments using the `argparse` library and returns them
    as a parsed argument object. It defines three positional or optional arguments:
    hostname, port number, and LBR Med version number. The function is designed
    to configure communication settings with a KUKA Sunrise Controller.

    Returns:
        argparseNamespace|argparseArgumentParser: A parsed object containing
        arguments that are used to initialize an application or module.

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
    Establishes a connection to a KUKA Sunrise controller, runs a teleoperation
    client application, and continuously steps through the application until it
    reaches an idle state or encounters an error.

    Returns:
        int: 0 if the execution completes successfully and 1 if it fails, indicating
        a non-zero status code.

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
