import pybullet as pbl
import numpy as np
from PID import PID_Controller
from MotionPlanners.MultiPointNav import MultiPointAPF
from Obstacle import Obstacle
from time import time
from lidar import lidar
from DVZ import DVZ
import torch
    
class Drone():

    def __init__(self, Obstacles, net = None, init_pos = np.array([0, 0, 0.5]), sensorNoise = 0, goal = np.array([0, 5, 0.5])):
        self.droneID = pbl.loadURDF("qimmiq.urdf", init_pos)

        self.sensorNoise = sensorNoise #Used only in the noise tests


        # Initialise robot state
        self.pos, self.quat = pbl.getBasePositionAndOrientation(self.droneID)
        self.theta = pbl.getEulerFromQuaternion(self.quat)
        self.xdot = np.zeros(3)
        self.all_xdot = []
        self.thetadot = np.zeros(3)

        self.networkInputs = [] # Only used for plotting later
        self.networkOutputs = []

        self.bbox = np.subtract(pbl.getAABB(self.droneID)[1], pbl.getAABB(self.droneID)[0]) #Used to determine collisions and position

        self.pathLength = 0
        self.e = 0
        self.obstacles = [Obstacle(obstacle, sensorNoise = 0) for obstacle in Obstacles]
        self.obstacles[-1].plane = True # Ensure that the ground is always the last obstacle in ObstacleIDs when initialising
        
        self.setpoint = goal
        
        self.controller = PID_Controller()
        if net == None:
            self.planner = MultiPointAPF()
        else:
            self.DVZ = DVZ()
            self.DVZ.setParameters( self.xdot )
            self.DVZ.scan( self )
            self.planner = net
            self.Intrusion = self.DVZ.I

        # The following are used for plotting only
        self.planner.all_distances = np.array([obstacle.getDistance(self) for obstacle in self.obstacles])
        self.obs_speed = []

        self.targetReached = False
        self.approachingTarget = False
        self.completedCourse = False

        self.I = np.array([0.054, 0.054, 0.1024]) #Inertia matrix
        self.dims = [0.5842, 0.5842, 0.127]
        self.body = [np.sqrt((self.dims[0]/2)**2 + (self.dims[1]/2)**2), np.sqrt((self.dims[0]/2)**2 + (self.dims[1]/2)**2), 0]

        self.mass = .42

        self.max_angle = np.pi/4 #Place a cap on the largest roll/pitch angle the drone can take to protect from instability

        # Constants required for quadcopter dynamics and control. Values chosen arbitrarily
        self.b = 0.5
        self.kd = 1
        self.k = 1

        self.time = 0
        self.times = []
        self.time_interval = 0.01 #Used to speed up or slow down simulation. Real Time requires 1/240
        [obstacle.setTimeInterval(self.time_interval) for obstacle in self.obstacles]
        self.all_state = np.zeros(6) #Entire state consists of position and angle
        self.state = np.array([i for i in self.pos] + [i for i in self.theta])
        self.all_state = np.vstack((self.all_state, self.state))

        self.getinitDesState()

        self.old_theta = np.zeros(3)

        self.input = np.zeros(4) #Inputs are the angular velocity squared of each rotor
        self.all_input = [self.input]


    def moveDrone(self, time_interval, j):

        self.detect_obstacle()

        self.old_pos, self.old_theta = self.pos, self.theta
        
        # Update self state
        self.pos, self.quat = pbl.getBasePositionAndOrientation(self.droneID)
        self.theta = pbl.getEulerFromQuaternion(self.quat)

        self.state = np.array([i for i in self.pos] + [i for i in self.theta])
        self.all_state = np.vstack((self.all_state, self.state))
        self.planner.all_distances = np.vstack((self.planner.all_distances, np.array([obstacle.getDistance(self) for obstacle in self.obstacles])))
        self.thetadot = np.subtract(self.theta, self.old_theta)/time_interval
        self.xdot = np.subtract(self.pos, self.old_pos)/time_interval
        self.all_xdot.append(self.xdot)

        self.pathLength += np.linalg.norm(np.subtract(self.pos, self.old_pos))
        
        start = time()

        if not self.targetReached:
            if not self.approachingTarget:
                self.setDesState()

            if abs(np.mean(self.thetadot) <= 0.05) and abs(np.mean(self.xdot) <= 0.05) and np.linalg.norm(self.controller.error[0:3]) <= 0.2 and self.approachingTarget:

                    self.controller.des_state = [i for i in self.all_state[-1, 0:2]] + [0., 0., 0., 0.]
                    self.targetReached = True

        self.times.append(time() - start) # At the end we average self.times to get the planner calculation time

        self.getInputs()
        self.all_input.append(np.array(self.input))

        # The following is the actual application of the forces to the drone in simulation to make the drone move
        pbl.applyExternalForce(self.droneID, -1, self.thrust() - self.kd * self.xdot, [0, 0, 0], pbl.LINK_FRAME)
        pbl.applyExternalTorque(self.droneID, -1, self.torque(), pbl.LINK_FRAME)

        self.time += time_interval

        # ITSE formula
        self.e += time_interval * self.time * (np.linalg.norm(np.subtract(self.setpoint, self.pos)) ** 2)
                
    def thrust(self):
        return [0, 0, self.k * np.sum(self.input)]

    def torque(self):
        return np.array([[self.body[0] * self.k * (self.input[0] + self.input[3] - self.input[1] - self.input[2])],
                         [self.body[1] * self.k * (self.input[2] + self.input[3] - self.input[0] - self.input[1])],
                         [self.b * (self.input[0] - self.input[1] + self.input[2] - self.input[3])]])

    # Acceleration and Angular Acceleration aren't needed since PyBullet takes care of this for us but can be useful for evaluation and debugging
    def angular_acceleration(self):
         return np.linalg.inv(self.I) @ (self.torque().squeeze() - np.cross(self.omega, self.I @ self.omega))

    def acceleration(self):
        gravity = np.array([0, 0, -9.8])
        Fd = -self.kd * self.xdot
        return gravity + (1/self.mass) * (self.thrust() + Fd)

    def detect_obstacle(self):
        """
        Determine the location and orientation of each obstacle in the environment. Use this to update the velocity information for the obstacles
        """
        locs = []
        for obstacle in self.obstacles:
            loc, orient  = pbl.getBasePositionAndOrientation(obstacle.ID)
            obstacle.update_state(np.array(loc))
            self.obs_speed.append(obstacle.xdot)

            obstacle.distance = obstacle.getDistance(self)
            obstacle.angle = np.arctan2(loc[1], loc[0])

    def getNNinputs(self):
        """
        Wrap the past four frames from the DVZ into a PyTorch Tensor to make it compatible with the Net Class
        """
        return torch.tensor(self.DVZ.distHistory)

    def getinitDesState(self):
        """
        Initialise the setpoint of the PID controller based on the output of the avoidance planner. 
        """
        if type(self.planner) == MultiPointAPF:
            self.planner.get_arcpoints(self)
            self.controller.des_state = [i for i in self.planner.getMinAPF(self)] + [0., 0., 0.]

            self.controller.error = self.controller.des_state - self.all_state[-1]
        else:
            self.controller.des_angles = np.zeros(3)
            roll, pitch, vSpeed = self.planner.forward(self.getNNinputs())
            self.controller.des_angles[0], self.controller.des_angles[1], self.controller.des_state[2] = roll.item()*np.pi/6, pitch.item()*np.pi/6, self.pos[2] + vSpeed.item()*2 * (self.time_interval)

            self.controller.error = np.array([self.controller.des_angles[0], self.controller.des_angles[1], self.controller.des_state[2]]) - np.array([self.theta[0], self.theta[1], self.pos[2]])

    def setDesState(self):
        """
        Set the setpoint of the PID Controller based on the output of the avoidance planner. When using the MultipointAPF planner, 
        the setpoint consists of the x-y-z position and zero for all r-p-y angles. We don't need this step for the GNN Method since the network takes care of the control logic
        """
        if type(self.planner) == MultiPointAPF:
            self.planner.get_arcpoints(self)
            self.controller.des_state = [i + self.sensorNoise * np.random.normal(0, 1) for i in self.planner.getMinAPF(self)] + [0., 0., 0.] #[X, Y, Z, R, P, Y]. Gaussian noise of predetermined standard deviation is used to introduce communication noise

            if (np.linalg.norm(self.controller.des_state[0:3] - self.setpoint[0:3])) <= 0.2: # We know that the planner has reached its final point when the setpoint determined by the planner matches (within 20cm) to the drone's goal location (self.setpoint)
                self.controller.des_state = [i for i in self.setpoint] + [0., 0., 0.]
                self.approachingTarget = True
        else:
            if (np.linalg.norm(self.pos[0:3] - self.setpoint[0:3])) <= 0.2:
                self.controller.des_state = [i for i in self.setpoint] + [0., 0., 0.]
                self.approachingTarget = True


    def getInputs(self):
        """
        Determine the inputs to the rotors (their angular velocity squared). In the case of the Multipoint planner this is done using a PID controller.
        With the GNN method, the control logic is bypassed and the inputs to the network are determined by considering the required roll-pitch angular velocities and vertical velocity
        """
        if type(self.planner) == MultiPointAPF:
            self.input = self.controller.posControl(self)
            self.planner.all_arcpositions.append(np.subtract(self.controller.des_state[0:3], self.pos[0:3]))
        else:
            if self.approachingTarget:
                self.input = self.controller.posControl(self)
            else:
                self.DVZ.setParameters( self.xdot )
                self.DVZ.scan(self)
                if self.DVZ.I >= self.Intrusion: # Determine the max intrusion up till current time
                    self.Intrusion = self.DVZ.I
                self.controller.des_angles = np.zeros(3)
                self.networkInputs.append(np.median(self.DVZ.distances))

                vSpeed, pitchVel, rollVel = self.planner.forward(self.getNNinputs()) # Run the DVZ input through the network to determine the outputs

                self.networkOutputs.append([rollVel.item(), pitchVel.item(), vSpeed.item()])
            
                self.controller.des_angles[0] = self.theta[0] + self.max_angle * rollVel.item() * self.time_interval # Numerical integration to determine the required angles and vertical position
                self.controller.des_angles[1] = self.theta[1] + self.max_angle * pitchVel.item() * self.time_interval
                self.controller.des_state[2] = self.pos[2] + 5 * vSpeed.item() * self.time_interval

                self.input = self.controller.nnControl(self)

