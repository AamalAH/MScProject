import numpy as np
import pybullet as pbl

class Obstacle():
    def __init__(self, ID, plane = False, sensorNoise = 0):
        self.ID = ID
        self.plane = plane

        self.sensorNoise = sensorNoise

        self.bbox = np.subtract(pbl.getAABB(self.ID)[1], pbl.getAABB(self.ID)[0])
        self.bbox2 = pbl.getAABB(self.ID)
        self.pos, orient = pbl.getBasePositionAndOrientation(ID)

        self.distance = np.inf
        self.angle = 0

        self.pos = np.array(self.pos)
        self.xdot = np.zeros(3)

    def setTimeInterval(self, time_interval):
        self.time_interval = time_interval

    def update_state(self, pos):
        self.old_pos = self.pos
        self.pos = np.add(pos, self.sensorNoise * np.random.normal(0, 1, size = 3))
        self.bbox2 = pbl.getAABB(self.ID)
        self.xdot = (self.pos - self.old_pos)/self.time_interval
        
    def getClosestPoint(self, Drone):
        if len(pbl.getClosestPoints(Drone.droneID, self.ID, 40, -1, -1)) > 0:
            if self.plane: return np.array(pbl.getClosestPoints(Drone.droneID, self.ID, 50, -1, -1)[0][5]) + self.sensorNoise * np.random.normal(0, 1, size = 3)
            else: return np.array(pbl.getClosestPoints(Drone.droneID, self.ID, 50, -1, -1)[0][6]) + self.sensorNoise * np.random.normal(0, 1, size = 3)
        else:
            return np.array([np.inf, np.inf, np.inf])

    def getDistance(self, Drone):
        if len(pbl.getClosestPoints(Drone.droneID, self.ID, 40, -1, -1)) > 0:
            return pbl.getClosestPoints(Drone.droneID, self.ID, 40, -1, -1)[0][8] + self.sensorNoise * np.random.normal(0, 1)
        else:
            return np.inf