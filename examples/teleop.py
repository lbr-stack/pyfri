import sys

# FRI Client: https://github.com/cmower/FRI-Client-SDK_Python
import pyFRIClient as fri

# OpTaS: https://github.com/cmower/optas
import optas

# PyGame: https://www.pygame.org/news
import pygame

pygame.init()

# NumPy: https://numpy.org/
import numpy as np


class Keyboard:
    def __init__(self):
        pygame.display.set_mode((300, 300))
        self.vgoal = np.zeros(3)  # position x, y, z

        self.max_vel_magnitude = 0.02

        self.keys = {
            pygame.K_LEFT,
            pygame.K_RIGHT,
            pygame.K_UP,
            pygame.K_DOWN,
            pygame.K_w,
            pygame.K_s,
        }

        self.key_to_dim_map = {
            pygame.K_LEFT: 0,
            pygame.K_RIGHT: 0,
            pygame.K_UP: 1,
            pygame.K_DOWN: 1,
            pygame.K_w: 2,
            pygame.K_s: 2,
        }

        self.key_to_dir_map = {
            pygame.K_LEFT: 1.0,
            pygame.K_RIGHT: -1.0,
            pygame.K_UP: 1.0,
            pygame.K_DOWN: -1.0,
            pygame.K_w: 1.0,
            pygame.K_s: -1.0,
        }

    def __call__(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                # User closed pygame window -> shutdown
                pygame.quit()
                raise SystemExit

            if event.type == pygame.KEYDOWN:
                # Keydown event

                if event.key in self.keys:
                    idx = self.key_to_dim_map[event.key]
                    self.vgoal[idx] += self.key_to_dir_map[event.key]

            elif event.type == pygame.KEYUP:
                # Keyup event

                if event.key in self.keys:
                    idx = self.key_to_dim_map[event.key]
                    self.vgoal[idx] -= self.key_to_dir_map[event.key]

        return self.max_vel_magnitude * self.vgoal.copy()


class IK:

    """
    Inverse Kinematics
    ==================

    This class solves the following optimization problem.


    q0*, qf*, dq*   =     arg  min    || p(qf) - pg ||^2  +  w * || dq ||^2
                         q0, qf, dq

    s.t.

           Goal velocity
           pg = p(qc) + dt * vg

           Initial configuration:
           q0 == qc

           Joint limits:
           q- <= q0 <= q+
           q- <= qf <= q+

           Dynamics:
           qf = q0 + dt * dq


    Decision variables: q0, qf, dq
    Parameters:
      qc : current joint configuration
      dt : time step
      vg : linear goal end-effector velocity
      w  : cost term weight

    Functions:
      p( .. ) : Forward kinematics (global link position)

    Solver:
      SLSQP from scipy.minimize. This is an SQP solver.

    """

    def __init__(self, robot_model, eelink):
        # Setup robot model
        self.robot = robot_model
        self.name = self.robot.get_name()

        # Setup optimization builder
        T = 2
        builder = optas.OptimizationBuilder(T, robots=self.robot)

        # Setup parameters
        qcurr = builder.add_parameter("qcurr", self.robot.ndof)
        vgoal = builder.add_parameter("vgoal", 3)
        dt = builder.add_parameter("dt")

        # Current end-effector position
        pcurr = self.robot.get_global_link_position(eelink, qcurr)

        # Goal end-effector position
        pgoal = pcurr + dt * vgoal

        # Get final end-effector state
        qfinal = builder.get_model_state(self.name, 1)
        pfinal = self.robot.get_global_link_position(eelink, qfinal)

        # Cost: goal end-effector position
        builder.add_cost_term("eegoal", optas.sumsqr(pfinal - pgoal))

        # Get joint velocity
        dq = builder.get_model_state(self.name, 0, time_deriv=1)

        # Cost: minimize joint velocity
        w = 0.01
        builder.add_cost_term("dqmin", w * optas.sumsqr(dq))

        # Constraint: model limits
        builder.enforce_model_limits(self.name)  # joint position limits from URDF

        # Constraint: initial configuration
        builder.fix_configuration(self.name, config=qcurr)
        builder.integrate_model_states(
            self.name, 1, dt
        )  # integrate velocity -> position

        # Setup solver
        optimization_problem = builder.build()
        self.solver = optas.ScipyMinimizeSolver(optimization_problem).setup("SLSQP")
        self.solution = None

    def __call__(self, qcurr, vgoal, dt):
        # Setup solver
        if self.solution is not None:
            self.solver.reset_initial_seed(self.solution)
        else:
            self.solver.reset_initial_seed(
                {f"{self.name}/q": optas.horzcat(qcurr, qcurr)}
            )

        self.solver.reset_parameters({"qcurr": qcurr, "vgoal": vgoal, "dt": dt})

        # Solve problem
        self.solution = self.solver.solve()

        return self.solution[f"{self.name}/q"][:, -1].toarray().flatten()


class TeleopClient(fri.LBRClient):
    def __init__(self, ik, keyboard):
        super().__init__()
        self.ik = ik
        self.keyboard = keyboard
        self.torques = np.zeros(fri.LBRState.NUMBER_OF_JOINTS, dtype=np.float32)

    def monitor(self):
        pass

    def onStateChange(self, old_state, new_state):
        print(f"State changed from {old_state} to {new_state}")

    def set_position(self, q):
        self.robotCommand().setJointPosition(q.astype(np.float32))
        self.robotCommand().setTorque(self.torques)

    def waitForCommand(self):
        self.set_position(self.robotState().getIpoJointPosition())

    def command(self):
        dt = self.robotState().getSampleTime()
        qcurr = self.robotState().getIpoJointPosition()
        vgoal = self.keyboard()
        qgoal = self.ik(qcurr, vgoal, dt)
        self.set_position(qgoal)


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

    xacro_file_name = f"robots/med{lbr_med_num}.urdf.xacro"
    eelink = "lbr_link_ee"
    robot_model = optas.RobotModel(xacro_filename=xacro_file_name, time_derivs=[0, 1])

    ik = IK(robot_model, eelink)
    client = TeleopClient(ik)
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
