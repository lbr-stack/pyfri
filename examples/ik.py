import optas


class IK:
    def __init__(self, lbr_med_num):
        # Setup robot
        xacro_file_name = f"robots/med{lbr_med_num}.urdf.xacro"
        ee_link = "lbr_link_ee"
        self.robot = optas.RobotModel(
            xacro_filename=xacro_file_name, time_derivs=[0, 1]
        )
        self.name = self.robot.get_name()

        # Setup builder
        T = 2
        builder = optas.OptimizationBuilder(T, robots=self.robot)

        # Set parameters
        qc = builder.add_parameter("qc", self.robot.ndof)  # current joint position
        vg = builder.add_parameter("vg", 3)  # task space velocity goal: [vx, vy, vz]
        dt = builder.add_parameter("dt")  # time step

        # Get model states
        qf = builder.get_model_state(self.name, t=1, time_deriv=0)
        dq = builder.get_model_state(self.name, t=0, time_deriv=1)

        # Cost: end-effector goal velocity
        Jl = self.robot.get_global_link_linear_jacobian(ee_link, qc)
        builder.add_cost_term("ee_vel_goal", 50.0 * optas.sumsqr(Jl @ dq - vg))

        # Constraint: initial configuration
        builder.initial_configuration(self.name, qc)

        # Constraint: dynamics
        builder.integrate_model_states(self.name, 1, dt)

        # Constraint: joint limits
        builder.enforce_model_limits(self.name)

        # Cost: minimize joint velocity
        builder.add_cost_term("min_joint_vel", 0.01 * optas.sumsqr(dq))

        # Setup solver
        opt = builder.build()
        self.solver = optas.ScipyMinimizeSolver(opt).setup("SLSQP")
        # self.solver = optas.OSQPSolver(opt).setup(True)
        self.solution = None

    def __call__(self, qc, vg, dt):
        # Reset initial seed with previous solution
        if self.solution is not None:
            self.solver.reset_initial_seed(self.solution)
        else:
            self.solver.reset_initial_seed({f"{self.name}/q": optas.horzcat(qc, qc)})

        # Reset parameters
        self.solver.reset_parameters({"qc": qc, "vg": vg, "dt": dt})

        # Solve problem
        self.solution = self.solver.solve()

        # Return solution or current position if solver failed
        if self.solver.did_solve():
            return self.solution[f"{self.name}/q"][:, 1].toarray().flatten()
        else:
            print("[WARN] solver failed!")
            return qc
