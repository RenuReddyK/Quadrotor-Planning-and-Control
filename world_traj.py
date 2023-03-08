import numpy as np

from .graph_search import graph_search

class WorldTraj(object):


    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. 

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """
       
        self.resolution = np.array([0.1, 0.1, 0.1])
        self.margin = 0.5
  
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        self.points = np.zeros((1,3)) # shape=(n_pts,3)

    # Remove collinear points 
    def sparse_waypoints(path):
        new_waypoints = []
        for i in range(1,path.shape[0]-1):
                x1,y1,z1 = path[i-1]
                x2,y2,z2 = path[i]
                x3,y3,z3 = path[i+1]
                P1 = (x2-x1,y2-y1,z2-z1)/np.linalg.norm((x2-x1,y2-y1,z2-z1))
                P2 = (x3-x2,y3-y2,z3-z2)/np.linalg.norm((x3-x2,y3-y2,z3-z2))
                directional_vec = np.dot(P1,P2)
                if directional_vec != 1:
                    if path[i].tolist() not in new_waypoints:
                        new_waypoints.append(path[i].tolist())
        new_waypoints = new_waypoints[::2]
        
        # New waypoints
        if new_waypoints[-1] != path[-1].tolist():
            new_waypoints.append(path[-1].tolist())
        mypoints = np.array(new_waypoints)

        return mypoints 
       
        
    
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
       
        self.points = self.path 
        N = self.points.shape[0]

        I = np.zeros((N-1,3))
        D = np.zeros((N-1,1))
        
        v = 3.1
        for i in range(N-1):
            # Unit vectors describing the direction of travel for each segment
            I[i, :] = (self.points[i+1,:]-self.points[i,:])/(np.linalg.norm(self.points[i+1,:]-self.points[i,:])) 
            # Distance of each segment
            D[i, :] = np.linalg.norm(self.points[i+1,:]-self.points[i,:]) 
        
        if np.all(self.points[-1, :].flatten() != self.path[-1, :].flatten()):
            self.points = np.concatenate((self.points,self.path[-1].reshape(1,-1)),axis=0)

        self.points = WorldTraj.sparse_waypoints(self.points)
        
        N = self.points.shape[0]
        I = np.zeros((N-1,3))
        D = np.zeros((N-1,1))
        
        for i in range(N-1):
            # Unit vectors describing the direction of travel for each segment
            I[i, :] = (self.points[i+1,:]-self.points[i,:])/(np.linalg.norm(self.points[i+1,:]-self.points[i,:])) 
            # Distance of each segment
            D[i, :] = np.linalg.norm(self.points[i+1,:]-self.points[i,:]) 
      
        traj_start_time = 0
        T_start = np.zeros((N,1))

        T = np.divide(D,v)  #duration of each segment
        for i in range(1,N):
            T_start[i] = T[i-1] + T_start[i-1]
       
        point = np.zeros((1,3))
        I_i = np.zeros((3,))
        for i in range(N-1):
            if T_start[i]<= t < T_start[i+1]:
                traj_start_time = T_start[i]
                point = self.points[i,:]
                I_i = I[i,:]
                x_dot = v * I_i
                point = point.reshape((3,1))
                I_i = I_i.reshape((3,1))
                x = point + (v*I_i)*(t-traj_start_time)
                x_dot = v* I_i
                break
            elif t> T_start[N-1]:
                x = self.points[-1]
                x_dot =[0,0,0]
                break

        if t> T_start[N-1] or I.shape[0]==0:
            x = self.points[-1]
            x_dot =[0,0,0]
        else:
            i = len(T_start[t>=T_start]) - 1
            traj_start_time = T_start[i][0]
            traj_end_time = T_start[i+1][0]
            point = self.points[i,:].reshape((3,1))
            I_i = I[i,:].reshape((3,1))
            x = (point + (v*I_i)*(t-traj_start_time)).reshape((3,))
            x_dot = v * I_i

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
