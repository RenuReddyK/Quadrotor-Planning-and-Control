import numpy as np

from .graph_search import graph_search

class WorldTraj(object):
    """

    """


    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.1, 0.1, 0.1])
        self.margin = 0.5

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        # You must generate a sparse set of wayself.points to fly between. Your
        # original Dijkstra or AStar path probably has too many self.points that are
        # too close together. Store these wayself.points as a class member; you will
        # need it for debugging and it will be used when plotting results.
        self.points = np.zeros((1,3)) # shape=(n_pts,3)

        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # STUDENT CODE HERE

    # Remove collinear points 
    def sparse_waypoints(path):
        new_waypoints = []
        # new_waypoints.append(path[0].tolist())
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
        # print(len(new_waypoints))
        if new_waypoints[-1] != path[-1].tolist():
            new_waypoints.append(path[-1].tolist())
        mypoints = np.array(new_waypoints)

        return mypoints 
       
        
        # new_points 
    
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

        # STUDENT CODE HERE
       
        self.points = self.path 
        # print(self.points)
        # print(len(self.points))
        N = self.points.shape[0]

        I = np.zeros((N-1,3))
        D = np.zeros((N-1,1))
        v = 1
        for i in range(N-1):
            # Unit vectors describing the direction of travel for each segment
            I[i, :] = (self.points[i+1,:]-self.points[i,:])/(np.linalg.norm(self.points[i+1,:]-self.points[i,:])) 
            # Distance of each segment
            D[i, :] = np.linalg.norm(self.points[i+1,:]-self.points[i,:]) 
        # print(np.sum(D[:,-1]))

        
        # if np.sum(D[:,-1]) >25.5:   
        #     # print("k")  
        #     if 34.4 < np.sum(D[:,-1]) < 34.7:   
        #         # self.points = self.points[::2, :]
        #         # self.points = np.concatenate((self.points,self.path[-1].reshape(1,-1)),axis=0)
        #         v = 3.1
        #     else:
        #         self.points = self.points[::2, :]
        #         # self.points = np.concatenate((self.points,self.path[-1].reshape(1,-1)),axis=0)
        #         v = 3.7

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
                # traj_end_time = T_start[i+1]
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

        ########################## MINIMUM JERK SPLINE IMPLEMENTATION #############################
        #########################################################################################

 
        # t1 = traj_start_time
        # t2 = traj_end_time 
        # print(t2)
        # exit()

        # A = [[0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 0, 0, t2**5, t2**4, t2**3, t2**2, t2, 1],
        #     [0, 0, 0, 0, 0, 0, 5*t2**4, 4*t2**3, 3*t2**2, 2*t2, 1, 0],
        #     [0, 0, 0, 0, 0, 0, 20*t2**3, 12*t2**2, 6*t2, 2, 0, 0],
        #     [t1**5, t1**4, t1**3, t1**2, t1, 1, 0, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        #     [5*t1**4, 4*t1**3, 3*t1**2, 2*t1, 1, 0, 0, 0, 0, 0, -1, 0],
        #     [20*t1**3, 12*t1**2, 6*t1, 2, 0, 0 , 0, 0, 0, -2, 0, 0],
        #     [60*t1**2, 24*t1, 6, 0, 0, 0 , 0, 0, -6, 0, 0, 0],
        #     [120*t1, 24, 0, 0, 0, 0 , 0, -24, 0, 0, 0, 0]]

        # #X = [[C15],[C14],[C13],[C12],[C11],[C10],[C25],[C24],[C23],[C22],[C21],[C20]]
        # B = [[p1_0],[p1_dot_0],[p1_ddot_0],[p2_t2],[p2_dot_t2],[p2_ddot_t2],[p1_t1],[p2_0],[0],[0],[0],[0]]
        
        # X = np.solve(A,B)

        #########################################################################################
        
        
        
        
        
        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output