import numpy as np
from scipy.spatial.transform import Rotation
# from waypoint_traj import WaypointTraj
import matplotlib.pyplot as plt
class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
 
        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2  # K_F
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2  # K_M

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        self.K_D = 0.7* np.array([[6.6, 0, 0],
                       [0, 6.5, 0],
                       [0, 0, 6]])

        self.K_P = 0.7* np.array([[9.5, 0, 0],
                       [0, 9.5, 0],
                       [0, 0, 5]])

        self.K_r  = 0.7* np.array([[1750, 0, 0],
                       [0, 1750, 0],
                       [0, 0, 150]])

        self.K_w = 0.7* np.array([[150, 0, 0],
                       [0, 150, 0],
                       [0, 0, 90]])


        self.K_M = self.k_drag
        self.K_F = self.k_thrust 


    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N 
                cmd_moment, N*m 
                cmd_q, quaternion [i,j,k,w] 
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))
       
        self.xT = state['x']
        self.vT = state['v']
        self.qT = state['q']
        self.wT = state['w']

        self.x = flat_output['x']
        self.x_dot = flat_output['x_dot']
        self.x_ddot = flat_output['x_ddot']
        self.x_dddot = flat_output['x_dddot']
        self.x_ddddot = flat_output['x_ddddot']
        self.yaw = flat_output['yaw']
        self.yaw_dot = flat_output['yaw_dot'] 
        F = np.zeros((4,))

        cmd_acceleration = np.zeros((3,))

        cmd_acceleration = np.subtract(self.x_ddot, ((np.dot(self.K_D, np.subtract(self.vT, self.x_dot))) + np.dot(self.K_P, np.subtract(self.xT, self.x))))

        cmd_acceleration = cmd_acceleration.reshape((3,1))
        F_des = np.dot(self.mass, cmd_acceleration) + np.array([[0],[0],[self.mass*self.g]])
        R = Rotation.from_quat(self.qT)
        R = R.as_matrix()
        b_3 = R @ np.array([[0],[0],[1]])
        b3_des = F_des/np.linalg.norm(F_des)
        U1 = np.transpose(b_3) @ F_des
        a_yaw = np.array([[np.cos(self.yaw)], [np.sin(self.yaw)], [0]])
        b2_des = (np.cross(b3_des.T, a_yaw.T)/(np.linalg.norm(np.cross(b3_des.T, a_yaw.T)))).T
        R_des = np.hstack((np.cross(b2_des.T,b3_des.T).T, b2_des, b3_des))
        
        e_R_33 = np.subtract(np.matmul(R_des.T, R),np.matmul(R.T,R_des))
    
        X1 = e_R_33[2,1]
        X2 = e_R_33[0,2]
        X3 = e_R_33[1,0]
        e_R = 0.5*np.array([[X1],[X2],[X3]])
   
        gamma = self.K_M/self.K_F
        l = self.arm_length
        w_des = 0
        e_w = self.wT - w_des
        e_R = e_R.reshape((3,1))
        e_w = e_w.reshape((3,1))

        U2 = np.matmul(self.inertia, (-self.K_r @ e_R - self.K_w @ e_w))
        U = np.vstack((U1,U2))
    

        F = np.zeros((4,1))
        F = np.linalg.inv(np.array([[1,1,1,1],[0, l, 0, -l],[-l, 0, l, 0],[gamma, -gamma, gamma,-gamma]])) @ U
   
        cmd_motor_speeds = np.sign(F) * np.sqrt(np.absolute(F)/self.k_thrust)
        cmd_motor_speeds = np.clip(cmd_motor_speeds, self.rotor_speed_min, self.rotor_speed_max)

        cmd_thrust = U1.flatten()
        cmd_moment = U2

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}

        
        return control_input
