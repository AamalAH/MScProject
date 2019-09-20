import numpy as np

class PID_Controller():

    def __init__(self):

        # State information contains position and angle
        self.Kp = [25 * np.pi/180, -25 * np.pi/180, .3686864, 25, 25, 25]
        self.Ki = [ 5* np.pi/180, -5 * np.pi/180, 0.5, 3, 3, .0]
        self.Kd = [20 * np.pi/180, -20 * np.pi/180, .84622276, 5, 5, 5]

        self.des_state = np.zeros(6)

        self.der_ang_error = np.zeros(3)
        self.der_error = np.zeros(6)

        self.cum_error_pos = np.zeros(3)
        self.cum_error_angle = np.zeros(3)

    def theta_control(self, Drone):

        if Drone.time != 0:
            self.old_ang_error = self.ang_error
            self.ang_error = self.des_angles - np.array(Drone.theta)
            self.der_ang_error = -1 * Drone.thetadot

        else:
            self.ang_error = self.des_angles - np.array(Drone.theta)

        self.ang_output = self.Kp[3:] * self.ang_error + self.Ki[3:] * self.cum_error_angle + self.Kd[3:] * self.der_ang_error

        upthrust = (Drone.mass * 9.8) / (Drone.k * np.cos(Drone.theta[0]) * np.cos(Drone.theta[1]))

        input = self.error2input(Drone, upthrust, self.ang_output)
        self.cum_error_angle += self.ang_error * self.time_interval

        return input

    def posControl(self, Drone):

        self.time_interval = Drone.time_interval

        if Drone.time != 0:
            self.old_error = self.error
            self.error = self.des_state - Drone.state
            self.der_error = -1 * np.hstack((Drone.xdot, Drone.thetadot))#(self.error - self.old_error) / (1 / 240)
        else:
            self.error = self.des_state - Drone.state

        if abs(self.error[2]) < 0.05:

            self.zOutput = self.Kp[2] * self.error[2] + self.Ki[2] * self.cum_error_pos[2] + self.Kd[2] * self.error[2]
            self.cum_error_pos[2] += self.error[2] * (1/240)
        else:
            self.zOutput = self.Kp[2] * self.error[2] + self.Kd[2] * self.error[2]

        self.set_des_angles()
        input = self.theta_control(Drone)
        input += self.error2input(Drone, self.zOutput, np.zeros(3))

        input[input < 0] = 0

        return input

    def set_des_angles(self):

        self.des_angles = np.zeros(3)
        if abs(self.error[1]) < 0.05:
            self.des_angles[0] = self.Kp[1] * self.error[1] + self.Ki[1] * self.cum_error_pos[1] + self.Kd[1] * self.der_error[1]
            self.cum_error_pos[1] += self.error[1]*(1/240)
        else:
            self.des_angles[0] = self.Kp[1] * self.error[1] + self.Kd[1] * self.der_error[1]

        if self.des_angles[0] > np.pi / 6: self.des_angles[0] = np.pi / 6
        if self.des_angles[0] < -np.pi / 6: self.des_angles[0] = -np.pi / 6

        if abs(self.error[0]) < 0.05:
            self.des_angles[1] = self.Kp[0] * self.error[0] + self.Ki[0] * self.cum_error_pos[0] + self.Kd[0] * self.der_error[0]
            self.cum_error_pos[1] += self.error[0] * (1 / 240)
        else:
            self.des_angles[1] = self.Kp[0] * self.error[0] + self.Kd[0] * self.der_error[0]

        if self.des_angles[1] > np.pi/6: self.des_angles[1] = np.pi/6
        if self.des_angles[1] < -np.pi/6: self.des_angles[1] = -np.pi/6

    def error2input(self, Drone, upthrust, ang_error):
        L = Drone.dims[0]/2
        k = Drone.k
        b = Drone.b

        Params = np.array([[-L*k, L*k, L*k, -L*k], [L*k, L*k, -L*k, -L*k], [-b, b, -b, b], [1, 1, 1, 1]])
        constraints = np.hstack((-Drone.I * ang_error, np.array([upthrust])))

        return np.linalg.solve(Params, constraints)

    def nnControl(self, Drone):
        self.time_interval = Drone.time_interval

        if Drone.time != 0:
            self.old_error = self.error
            self.error = self.des_state - Drone.state
            self.der_error = -1 * np.hstack((Drone.xdot, Drone.thetadot))#(self.error - self.old_error) / (1 / 240)
        else:
            self.error = np.array([self.des_angles[0], self.des_angles[1], self.des_state[2]]) - np.array([Drone.theta[0], Drone.theta[1], Drone.pos[2]])

        if abs(self.error[2]) < 0.05:

            self.zOutput = self.Kp[2] * self.error[2] + self.Ki[2] * self.cum_error_pos[2] + self.Kd[2] * self.error[2]
            self.cum_error_pos[2] += self.error[2] * (1/240)
        else:
            self.zOutput = self.Kp[2] * self.error[2] + self.Kd[2] * self.error[2]

        input = self.theta_control(Drone)
        input += self.error2input(Drone, self.zOutput, np.zeros(3))
        upthrust = (Drone.mass * 9.8) / (Drone.k * np.cos(Drone.theta[0]) * np.cos(Drone.theta[1]))
        input[input < 0] = 0

        return input