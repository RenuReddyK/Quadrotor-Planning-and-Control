import numpy as np

class WaypointTraj(object):
    """

    """
    def __init__(self, points):
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission. For a waypoint
        trajectory, the input argument is an array of 3D destination
        coordinates. You are free to choose the times of arrival and the path
        taken between the points in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        """

        # STUDENT CODE HERE
        self.N = points.shape[0]
        self.points = points

        self.I = np.zeros((self.N-1,3))
        self.D = np.zeros((self.N-1,1))
        
        for i in range(self.N-1):
            # Unit vectors describing the direction of travel for each segment
            self.I[i, :] = (self.points[i+1,:]-self.points[i,:])/(np.linalg.norm(self.points[i+1,:]-self.points[i,:])) 
            # Distance of each segment
            self.D[i, :] = np.linalg.norm(self.points[i+1,:]-self.points[i,:]) 
        
         
    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        T_start = np.zeros((self.N,))

        # STUDENT CODE HERE
        v = 1
        traj_start_time = 0
        T_start = np.zeros((self.N,1))

        T = np.divide(self.D,v)  #duration of each segment
        for i in range(1,self.N):
            T_start[i] = T[i-1] + T_start[i-1]
       

        point = np.zeros((1,3))
        I_i = np.zeros((3,))
        for i in range(self.N-1):
            if T_start[i]<= t < T_start[i+1]:
                traj_start_time = T_start[i]
                point = self.points[i,:]
                I_i = self.I[i,:]
                x_dot = v * I_i
                point = point.reshape((3,1))
                I_i = I_i.reshape((3,1))
                x = point + (v*I_i)*(t-traj_start_time)
                x_dot = v* I_i
                break
            elif t> T_start[self.N-1]:
                x = self.points[-1]
                x_dot =[0,0,0]
                break

        if t> T_start[self.N-1] or self.I.shape[0]==0:
            x = self.points[-1]
            x_dot =[0,0,0]
        else:
            i = len(T_start[t>=T_start]) - 1
            traj_start_time = T_start[i][0]
            point = self.points[i,:].reshape((3,1))
            I_i = self.I[i,:].reshape((3,1))
            x = (point + (v*I_i)*(t-traj_start_time)).reshape((3,))
            x_dot = v * I_i
    
        
        # The yaw, yaw rate, acceleration, jerk, and snap will be 0
        # i.e.,  x_ddot = x_dddot = x_ddddot =0 anf yaw, yaw_dot are also 0

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}

        return flat_output
