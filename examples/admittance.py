import optas
from robot import load_robot

import numpy as np


class AdmittanceController:
    def __init__(self, lbr_med_num, con_ee_z):
        # Setup robot
        ee_link = "lbr_link_ee"
        self.robot = load_robot(lbr_med_num, [1])
        self.name = self.robot.get_name()

        # Setup builder
        T = 1
        builder = optas.OptimizationBuilder(T, robots=self.robot, derivs_align=True)

        # Set parameters
        qc = builder.add_parameter("qc", self.robot.ndof)  # current joint position
        vg = builder.add_parameter(
            "vg", 6
        )  # task space velocity goal: [vx, vy, vz, wx, wy, wz]
        dt = builder.add_parameter("dt")  # time step

        # Get model states
        dq = builder.get_model_state(self.name, t=0, time_deriv=1)

        # Cost: end-effector goal velocity
        J = self.robot.get_global_link_geometric_jacobian(ee_link, qc)
        v = J @ dq
        builder.add_cost_term("ee_vel_goal", 50.0 * optas.sumsqr(v - vg))

        # Constraint: only allow eff axis motion
        if con_ee_z:
            Je = self.robot.get_link_geometric_jacobian(ee_link, qc, ee_link)
            ve = Je @ dq
            selection = optas.DM([1, 1, 0, 1, 1, 1])
            builder.add_equality_constraint("con_ee_motion", selection * ve)

        # Constraint: joint limits
        q = qc + dt * dq
        builder.add_bound_inequality_constraint(
            "joint_limits",
            self.robot.lower_actuated_joint_limits,
            q,
            self.robot.upper_actuated_joint_limits,
        )

        # Cost: minimize joint velocity
        builder.add_cost_term("min_joint_vel", optas.sumsqr(dq))

        # Setup solver
        opt = builder.build()
        solver_options = {"printLevel": "none"}
        self.solver = optas.CasADiSolver(opt).setup("qpoases", solver_options)
        self.solution = None

        # Setup for callback
        self.J = self.robot.get_global_link_geometric_jacobian_function(
            ee_link, numpy_output=True
        )
        self.tau_ext = np.zeros(self.robot.ndof)
        self.tau_ext_prev = np.zeros(self.robot.ndof)
        self.smooth_tau_ext = 0.02
        self.threshold = np.array([5.5, 5.5, 5.5, 2.0, 2.0, 2.0])
        self.gain = np.array([0.075, 0.075, 0.075, 350.0, 350.0, 350.0])
        self.vlim = np.concatenate(([0.2, 0.2, 0.2], np.deg2rad([40] * 3)))
        self.vg = np.zeros(6)
        self.smooth_vg = np.array([0.0075, 0.0075, 0.0075, 0.0025, 0.0025, 0.0025])
        self.tau_ext_K = 7.5
        self.tau_ext_D = 0.8

    def __call__(self, qc, tau_ext, dt):
        # Smooth external torque
        tau_ext_prev = self.tau_ext.copy()
        self.tau_ext = tau_ext * self.smooth_tau_ext + self.tau_ext * (
            1.0 - self.smooth_tau_ext
        )
        dtau_ext = (self.tau_ext - tau_ext_prev) / dt
        dtau_ext_use = (
            self.tau_ext_K * (self.tau_ext - tau_ext_prev) - self.tau_ext_D * dtau_ext
        )
        tau_ext_use = tau_ext_prev + dtau_ext_use * dt

        # Compute jacobian and its inverse
        J = self.J(qc)
        Jinv = np.linalg.pinv(J, rcond=0.05)

        # Compute external wrench
        f_ext = Jinv.T @ tau_ext_use

        # Apply threshold filter
        f_ext = np.where(
            abs(f_ext) > self.threshold,
            np.sign(f_ext) * (abs(f_ext) - self.threshold),
            0.0,
        )

        # Admittance control: map force -> velocity
        vg = self.gain * f_ext

        # Clip velocity (safety)
        vg = np.clip(vg, -self.vlim, self.vlim)
        vg_prev = self.vg.copy()
        self.vg = vg * self.smooth_vg + self.vg * (1.0 - self.smooth_vg)

        # Setup solver
        if self.solution is not None:
            self.solver.reset_initial_seed(self.solution)

        self.solver.reset_parameters({"qc": qc, "vg": self.vg, "dt": dt})

        # Solve problem and retrieve solution
        self.solution = self.solver.solve()
        dqg = self.solution[f"{self.name}/dq"].toarray().flatten()
        qg = qc + dt * dqg

        return qg
