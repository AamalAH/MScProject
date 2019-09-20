import numpy as np

class MultiPointAPF():

    def __init__(self):
        self.rad = .2

        self.arcpoints = np.array([[[self.rad * np.sin(theta) * np.cos(phi), self.rad * np.sin(theta) * np.sin(phi), self.rad * np.cos(theta)] for theta in np.arange(-np.pi, np.pi, np.pi/4)] for phi in np.arange(-np.pi, np.pi, np.pi/4)]).reshape((64, 3)).T
        self.last_arcpoint = 0
        self.all_arcpositions = []
        self.all_distances = []

        self.A = 1e2
        self.eta = 1
        self.mu = self.eta * 20

        self.k = self.A * self.eta

        self.d_rep = 40
        self.Gdiscount = 1.5

    def get_arcpoints(self, Drone):
        point = self.arcpoints[:, self.last_arcpoint]
        T = np.arccos(point[2]/self.rad)
        P = np.arctan2(point[1], point[0])

        self.arc_positions = np.array([Drone.all_state[-1, 0:3] + self.arcpoints[:, i] for i in range(64)]).T
        self.arc_positions = np.hstack((self.arc_positions, np.array([[Drone.all_state[-1, 0] + self.rad * np.sin(theta) * np.cos(P), Drone.all_state[-1, 1] + self.rad * np.sin(theta) * np.sin(P), Drone.all_state[-1, 2] + self.rad * np.cos(theta)] for theta in [T - (np.pi / 8), T + (np.pi / 8)]]).T))
        self.arc_positions = np.hstack((self.arc_positions, np.array([[Drone.all_state[-1, 0] + self.rad * np.sin(T) * np.cos(phi), Drone.all_state[-1, 1] + self.rad * np.sin(T) * np.sin(phi), Drone.all_state[-1, 2] + self.rad * np.cos(T)] for phi in [P - (np.pi / 8), P + (np.pi / 8)]]).T))
        self.length = np.shape(self.arc_positions)[1]

    def U_cc(self, Drone):
        U_cc = np.zeros(self.length)

        relativeVelocities_H = [np.array(((self.arc_positions.T[:, 0:2] - Drone.pos[0:2])/self.rad * np.linalg.norm(Drone.xdot[0:2])) - obstacle.xdot[0:2]) for obstacle in Drone.obstacles[:-1]]
        R_NH = [np.linalg.norm(Drone.bbox[0:2]/2) + np.linalg.norm(obstacle.bbox[0:2]/2) for obstacle in Drone.obstacles[:-1]]
        d_NH = [np.linalg.norm(np.subtract(obstacle.pos[0:2], Drone.pos[0:2])) for obstacle in Drone.obstacles[:-1]]
        L_NH = np.sqrt(np.power(d_NH, 2) - np.power(R_NH, 2))
        phi_NH = [np.arctan2(obstacle.pos[1] - Drone.pos[1], obstacle.pos[0] - Drone.pos[0]) for obstacle in Drone.obstacles[:-1]]
        phi_L_NH = np.arctan2(R_NH, L_NH) + phi_NH
        phi_R_NH = - np.arctan2(R_NH, L_NH) + phi_NH

        horizontalCone = [np.where(np.array(np.arctan2(relativeVelocities_H[i][:, 1], relativeVelocities_H[i][:, 0]) <= phi_L_NH[i]) * np.array(np.arctan2(relativeVelocities_H[i][:, 1], relativeVelocities_H[i][:, 0]) >= phi_R_NH[i]))[0] for i in range(len(phi_NH))]

        relativeVelocities_V = [np.array(((self.arc_positions.T[:, 0::2] - Drone.pos[0::2])/self.rad * np.linalg.norm(Drone.xdot[0::2])) - obstacle.xdot[0::2]) for obstacle in Drone.obstacles[:-1]]

        getVerticalRAndD = lambda Drone, obstacle, Direction: (np.linalg.norm(Drone.bbox[0::2] / 2) + np.linalg.norm(obstacle.bbox[0::2] / 2), np.linalg.norm(np.subtract(obstacle.pos[0::2], Drone.pos[0::2]))) if Direction else (np.linalg.norm(Drone.bbox[1:2] / 2) + np.linalg.norm(obstacle.bbox[1:2] / 2), np.linalg.norm(np.subtract(obstacle.pos[1:2], Drone.pos[1:2])))

        Direction = [(Drone.xdot[0] - obstacle.xdot[0] > Drone.xdot[1] - obstacle.xdot[1]) for obstacle in Drone.obstacles[:-1]]
        Response = [getVerticalRAndD(Drone, Drone.obstacles[i], Direction[i]) for i in range((len(Direction)))]
        R_NV, d_NV = [response[0] for response in Response], [response[1] for response in Response]
        L_NV = np.sqrt(np.power(d_NV, 2) - np.power(R_NV, 2))
        theta_NV = [np.arccos((obstacle.pos[2] - Drone.pos[2]) / np.linalg.norm(obstacle.pos - Drone.pos)) for obstacle in Drone.obstacles[:-1]]
        theta_U_NV = - np.arctan2(R_NV, L_NV) + theta_NV
        theta_L_NV = np.arctan2(R_NV, L_NV) + theta_NV

        verticalCone = [np.where(np.array(np.arctan2(relativeVelocities_V[i][:, 1], relativeVelocities_V[i][:, 0]) <= theta_U_NV[i]) * np.array(np.arctan2(relativeVelocities_V[i][:, 1], relativeVelocities_V[i][:, 0]) >= theta_L_NV[i]))[0] for i in range(len(theta_NV))]

        for i in (verticalCone + horizontalCone): U_cc[i] += self.mu

        return U_cc

    def U_rep(self, Drone):
        distanceToObstacle = np.array([Drone.pos - obstacle.getClosestPoint(Drone) for obstacle in Drone.obstacles]) if not Drone.approachingTarget else np.array([Drone.pos - obstacle.getClosestPoint(Drone) for obstacle in Drone.obstacles])
        detectedObstacles = np.where(np.linalg.norm(distanceToObstacle, axis=1) < self.d_rep)[0]
        arcToObstacle = np.array([np.linalg.norm(self.arc_positions.T - Drone.obstacles[i].getClosestPoint(Drone), axis=1) for i in detectedObstacles])
        arcToObstacle[np.where((distanceToObstacle[detectedObstacles, 2] >= Drone.bbox[2]/2))] *= self.Gdiscount
        potentialContributions = self.eta * ((1/arcToObstacle - 1/self.d_rep) ** 2) * ((self.U_att(Drone)/self.k)**2)
        potential = np.sum(potentialContributions, axis = 0)

        return potential

    def U_att(self, Drone):
        return self.k * np.linalg.norm(np.array([(self.arc_positions[:, i] - Drone.setpoint)**2 for i in range(self.length)]).T, axis = 0)

    def evaluate_APF(self, Drone):
        return self.U_att(Drone) + self.U_rep(Drone) + self.U_cc(Drone)

    def getMinAPF(self, Drone):
        self.last_arcpoint = np.argmin(self.evaluate_APF(Drone))
        self.last_arcpoint = self.last_arcpoint if self.last_arcpoint < 64 else np.argmin([np.linalg.norm(self.arc_positions[:, i] - self.arc_positions[:, self.last_arcpoint]) for i in range(64)])
        return self.arc_positions[:, np.argmin(self.evaluate_APF(Drone))]

    def evaluate_deviation(self):
        if len(self.all_distances) > 0:
            return np.min(self.all_distances)
        else:
            return np.inf

    def evaluate_curvature(self, Drone):
        if Drone.all_state.shape[0] > 0:
            return np.mean([0.5 * np.linalg.norm(0.5 * Drone.all_state[i - 1, 0:3] - Drone.all_state[i, 0:3] + 0.5 * Drone.all_state[i + 1, 0:3]) for i in range(1, len(Drone.all_state) - 1)])
        else:
            return np.inf
    def evaluate_input_deviation(self, Drone):
        if len(Drone.all_input) > 2:
            Drone.all_input = np.array(Drone.all_input)
            print(Drone.all_input.shape)
            return np.max(np.array([0.5 * (0.5 * Drone.all_input[i - 1] - Drone.all_input[i] + 0.5 * Drone.all_input[i + 1]) for i in range(1, len(Drone.all_input) - 1)]), axis=0)
        else:
            return np.ones(4) * np.inf
