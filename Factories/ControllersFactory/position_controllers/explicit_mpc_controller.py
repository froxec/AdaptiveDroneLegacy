import numpy as np
from pyMPC.mpc import MPCController
from Factories.ModelsFactory.model_parameters import *
import scipy.sparse as sparse
import time
import matplotlib.pyplot as plt
from Factories.ModelsFactory.linear_models import LinearizedQuad
from Factories.ToolsFactory.Converters import AngularVelocityToThrust
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import Trajectory
from Factories.ToolsFactory.GeneralTools import euclidean_distance
class ModelPredictiveController():
    def __init__(self, quad_parameters, x0, trajectory, Ts,  angular_velocity_range, linear_model=LinearizedQuad, save_history=False):
        self.Ts = Ts
        self.position_trajectory = trajectory
        if isinstance(trajectory, Trajectory):
            self.current_waypoint_id = 0
            self.xref = np.concatenate([self.position_trajectory.generated_trajectory[self.current_waypoint_id], np.array([0, 0, 0])])
        else:
            self.xref = np.concatenate([self.position_trajectory, np.array([0, 0, 0])])
            self.current_waypoint_id = None
        self.angular_velocity_range = angular_velocity_range
        self.angular_velocity_converter = AngularVelocityToThrust(quad_parameters['Kt'])
        self.linear_model = linear_model(quad_parameters, yaw_ss=0.0, x_ref=self.xref[0], y_ref=self.xref[1], z_ref=self.xref[2])
        self.thrust_ss = quad_parameters['m']*quad_parameters['g']
        self.Ac = sparse.csc_matrix(self.linear_model.A)
        self.Bc = sparse.csc_matrix(self.linear_model.B)
        self.x_num, self.u_num = self.Bc.shape
        self.Ad = sparse.csc_matrix(np.eye(self.x_num) + self.Ac*self.Ts)
        self.Bd = sparse.csc_matrix(self.Bc*Ts)
        #reference values
        self.uminus1 = np.array([0.0, 0.0, 0.0])

        #constraints
        thrust_min = self.angular_velocity_converter(self.angular_velocity_range[0])
        thrust_max = self.angular_velocity_converter(self.angular_velocity_range[1])
        delta_thrust_min = thrust_min - self.thrust_ss
        delta_thrust_max = thrust_max - self.thrust_ss
        self.xmin = np.array([-np.inf, -np.inf, -np.inf, -2.5, -2.5, -2.5])
        self.xmax = np.array([np.inf, np.inf, np.inf, 2.5, 2.5, 2.5])

        self.umin = np.array([delta_thrust_min, -np.pi / 6, -np.pi / 6])
        self.umax = np.array([delta_thrust_max, np.pi / 6, np.pi / 6])

        self.Dumin = np.array([-np.inf, -np.pi*Ts/6 , -np.pi*Ts/6])
        self.Dumax = np.array([np.inf, np.pi*Ts/6, np.pi*Ts/6])

        #cost parameters
        self.Qx = sparse.diags([5, 5, 5, 10, 10, 10])  # Quadratic cost for states x0, x1, ..., x_N-1
        self.QxN = sparse.diags([0, 0, 0, 0, 0, 0])  # Quadratic cost for xN
        self.Qu = sparse.diags([10, 1000, 1000])  # Quadratic cost for u0, u1, ...., u_N-1
        self.QDu = sparse.diags([0, 0, 0])  # Quadratic cost for Du0, Du1, ...., Du_N-1

        self.x0 = x0
        self.x = x0
        self.u_prev = self.uminus1
        self.Np = 20
        self.Nc = 10
        self.MPC = MPCController(self.Ad, self.Bd, Np=self.Np, Nc =self.Nc, x0=self.x0, xref=self.xref, uminus1=self.uminus1,
                                 Qx=self.Qx, QxN=self.QxN, Qu=self.Qu, QDu=self.QDu,
                                 xmin=self.xmin, xmax=self.xmax, umin=self.umin, umax=self.umax, Dumin=self.Dumin,
                                 Dumax=self.Dumax)
        self.MPC.setup()
        self.history = []
    def update_state_control(self, x):
        self.x = x
        wp_reached = self.check_if_reached_waypoint(x)
        if wp_reached and self.current_waypoint_id is not None:
            self.current_waypoint_id = self.current_waypoint_id + 1 if self.current_waypoint_id < self.position_trajectory.generated_trajectory.shape[0] - 1 else None
            if self.current_waypoint_id is not None:
                self.xref = np.concatenate([self.position_trajectory.generated_trajectory[self.current_waypoint_id], np.array([0, 0, 0])])
                self.set_reference(self.xref)
        self.MPC.update(self.x, self.u_prev)
        u = self.MPC.output()
        self.u_prev = u
        self.save_history(u)
        return u
    def set_reference(self, ref):
        self.xref = ref
        self.MPC = MPCController(self.Ad, self.Bd, Np=self.Np, Nc=self.Nc, x0=self.x0, xref=self.xref,
                                 uminus1=self.uminus1,
                                 Qx=self.Qx, QxN=self.QxN, Qu=self.Qu, QDu=self.QDu,
                                 xmin=self.xmin, xmax=self.xmax, umin=self.umin, umax=self.umax, Dumin=self.Dumin,
                                 Dumax=self.Dumax)
        self.MPC.setup()

    def check_if_reached_waypoint(self, x):
        distance = euclidean_distance(self.xref, x)
        if distance < 0.1:
            return True
        else:
            return False
    def save_history(self, u):
        self.history.append(u)
