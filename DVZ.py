#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  8 17:51:45 2019

@author: aamalhussain
"""

import numpy as np
import pybullet as pbl

class DVZ():
    def __init__(self):

        # Definitions for all of these constants can be found in the dissertation

        self.c = [0, 0, 0]
        self.a = [0, 0, 0]
        self.L = 0.0096
        self.I = 0 # Intrusion term
        self.c_xmin = 30

        self.angleStep = 20 # Resolution of the lidar
        self.allAngles = np.array([[(theta, phi) for theta in np.arange(0, 2 * np.pi, np.deg2rad(self.angleStep))] for phi in np.arange(0, np.pi, np.deg2rad(self.angleStep))]).reshape((int(360 * 180/self.angleStep**2), 2))
        self.distances = np.zeros((self.allAngles.shape[0] + 94,))
        self.distHistory = [np.ones((self.allAngles.shape[0] + 94, )) * 40] * 4 #This will form the input to the network
        
        
    def setParameters(self, xdot):

        # The manner in which these are set can be found in the paper by Baklouti et al.

        V = np.linalg.norm(xdot)
        if V == 0:
            V = 1

        if xdot[0] > xdot[1] and xdot[0] > xdot[2]:
            self.c[0] = self.L * V**2 + self.c_xmin
            self.c[1] = self.c[2] = (5**0.5)/3 * self.c[0]

            self.a[0] = -(2/3) * self.c[0]
            self.a[1] = self.a[2] = 0

        elif xdot[1] > xdot[0] and xdot[1] > xdot[2]:
            self.c[1] = self.L * V**2 + self.c_xmin
            self.c[0] = self.c[2] = (5**0.5)/3 * self.c[1]

            self.a[1] = -(2/3) * self.c[1]
            self.a[0] = self.a[2] = 0

        else:
            self.c[2] = self.L * V**2 + self.c_xmin
            self.c[0] = self.c[1] = (5**0.5)/3 * self.c[2]

            self.a[2] = -(2/3) * self.c[2]
            self.a[0] = self.a[1] = 0

    def setDh(self, theta, phi, M):

        cx = self.c[0]
        cy = self.c[1]
        cz = self.c[2]

        ax = self.a[0]
        ay = self.a[1]
        az = self.a[2]

        cp = np.cos(phi)
        ct = np.cos(theta)
        st = np.sin(theta)
        sp = np.sin(phi)

        (r11, r12, r13, r21, r22, r23, r31, r32, r33) = M

        # This equation was solved for Dh using a MATLAB solver. The code for this can be found in /Misc/D_hCalc.m

        return (cx * cy * cz * (
                    - ax ** 2 * cp ** 2 * ct ** 2 * cy ** 2 * r31 ** 2 - ax ** 2 * cp ** 2 * ct ** 2 * cz ** 2 * r21 ** 2 - 2 * ax ** 2 * cp ** 2 * ct * cy ** 2 * r31 * r32 * st - 2 * ax ** 2 * cp ** 2 * ct * cz ** 2 * r21 * r22 * st - ax ** 2 * cp ** 2 * cy ** 2 * r32 ** 2 * st ** 2 - ax ** 2 * cp ** 2 * cz ** 2 * r22 ** 2 * st ** 2 - 2 * ax ** 2 * cp * ct * cy ** 2 * r31 * r33 * sp - 2 * ax ** 2 * cp * ct * cz ** 2 * r21 * r23 * sp - 2 * ax ** 2 * cp * cy ** 2 * r32 * r33 * sp * st - 2 * ax ** 2 * cp * cz ** 2 * r22 * r23 * sp * st - ax ** 2 * cy ** 2 * r33 ** 2 * sp ** 2 - ax ** 2 * cz ** 2 * r23 ** 2 * sp ** 2 + 2 * ax * ay * cp ** 2 * ct ** 2 * cz ** 2 * r11 * r21 + 2 * ax * ay * cp ** 2 * ct * cz ** 2 * r11 * r22 * st + 2 * ax * ay * cp ** 2 * ct * cz ** 2 * r12 * r21 * st + 2 * ax * ay * cp ** 2 * cz ** 2 * r12 * r22 * st ** 2 + 2 * ax * ay * cp * ct * cz ** 2 * r11 * r23 * sp + 2 * ax * ay * cp * ct * cz ** 2 * r13 * r21 * sp + 2 * ax * ay * cp * cz ** 2 * r12 * r23 * sp * st + 2 * ax * ay * cp * cz ** 2 * r13 * r22 * sp * st + 2 * ax * ay * cz ** 2 * r13 * r23 * sp ** 2 + 2 * ax * az * cp ** 2 * ct ** 2 * cy ** 2 * r11 * r31 + 2 * ax * az * cp ** 2 * ct * cy ** 2 * r11 * r32 * st + 2 * ax * az * cp ** 2 * ct * cy ** 2 * r12 * r31 * st + 2 * ax * az * cp ** 2 * cy ** 2 * r12 * r32 * st ** 2 + 2 * ax * az * cp * ct * cy ** 2 * r11 * r33 * sp + 2 * ax * az * cp * ct * cy ** 2 * r13 * r31 * sp + 2 * ax * az * cp * cy ** 2 * r12 * r33 * sp * st + 2 * ax * az * cp * cy ** 2 * r13 * r32 * sp * st + 2 * ax * az * cy ** 2 * r13 * r33 * sp ** 2 - ay ** 2 * cp ** 2 * ct ** 2 * cx ** 2 * r31 ** 2 - ay ** 2 * cp ** 2 * ct ** 2 * cz ** 2 * r11 ** 2 - 2 * ay ** 2 * cp ** 2 * ct * cx ** 2 * r31 * r32 * st - 2 * ay ** 2 * cp ** 2 * ct * cz ** 2 * r11 * r12 * st - ay ** 2 * cp ** 2 * cx ** 2 * r32 ** 2 * st ** 2 - ay ** 2 * cp ** 2 * cz ** 2 * r12 ** 2 * st ** 2 - 2 * ay ** 2 * cp * ct * cx ** 2 * r31 * r33 * sp - 2 * ay ** 2 * cp * ct * cz ** 2 * r11 * r13 * sp - 2 * ay ** 2 * cp * cx ** 2 * r32 * r33 * sp * st - 2 * ay ** 2 * cp * cz ** 2 * r12 * r13 * sp * st - ay ** 2 * cx ** 2 * r33 ** 2 * sp ** 2 - ay ** 2 * cz ** 2 * r13 ** 2 * sp ** 2 + 2 * ay * az * cp ** 2 * ct ** 2 * cx ** 2 * r21 * r31 + 2 * ay * az * cp ** 2 * ct * cx ** 2 * r21 * r32 * st + 2 * ay * az * cp ** 2 * ct * cx ** 2 * r22 * r31 * st + 2 * ay * az * cp ** 2 * cx ** 2 * r22 * r32 * st ** 2 + 2 * ay * az * cp * ct * cx ** 2 * r21 * r33 * sp + 2 * ay * az * cp * ct * cx ** 2 * r23 * r31 * sp + 2 * ay * az * cp * cx ** 2 * r22 * r33 * sp * st + 2 * ay * az * cp * cx ** 2 * r23 * r32 * sp * st + 2 * ay * az * cx ** 2 * r23 * r33 * sp ** 2 - az ** 2 * cp ** 2 * ct ** 2 * cx ** 2 * r21 ** 2 - az ** 2 * cp ** 2 * ct ** 2 * cy ** 2 * r11 ** 2 - 2 * az ** 2 * cp ** 2 * ct * cx ** 2 * r21 * r22 * st - 2 * az ** 2 * cp ** 2 * ct * cy ** 2 * r11 * r12 * st - az ** 2 * cp ** 2 * cx ** 2 * r22 ** 2 * st ** 2 - az ** 2 * cp ** 2 * cy ** 2 * r12 ** 2 * st ** 2 - 2 * az ** 2 * cp * ct * cx ** 2 * r21 * r23 * sp - 2 * az ** 2 * cp * ct * cy ** 2 * r11 * r13 * sp - 2 * az ** 2 * cp * cx ** 2 * r22 * r23 * sp * st - 2 * az ** 2 * cp * cy ** 2 * r12 * r13 * sp * st - az ** 2 * cx ** 2 * r23 ** 2 * sp ** 2 - az ** 2 * cy ** 2 * r13 ** 2 * sp ** 2 + cp ** 2 * ct ** 2 * cx ** 2 * cy ** 2 * r31 ** 2 + cp ** 2 * ct ** 2 * cx ** 2 * cz ** 2 * r21 ** 2 + cp ** 2 * ct ** 2 * cy ** 2 * cz ** 2 * r11 ** 2 + 2 * cp ** 2 * ct * cx ** 2 * cy ** 2 * r31 * r32 * st + 2 * cp ** 2 * ct * cx ** 2 * cz ** 2 * r21 * r22 * st + 2 * cp ** 2 * ct * cy ** 2 * cz ** 2 * r11 * r12 * st + cp ** 2 * cx ** 2 * cy ** 2 * r32 ** 2 * st ** 2 + cp ** 2 * cx ** 2 * cz ** 2 * r22 ** 2 * st ** 2 + cp ** 2 * cy ** 2 * cz ** 2 * r12 ** 2 * st ** 2 + 2 * cp * ct * cx ** 2 * cy ** 2 * r31 * r33 * sp + 2 * cp * ct * cx ** 2 * cz ** 2 * r21 * r23 * sp + 2 * cp * ct * cy ** 2 * cz ** 2 * r11 * r13 * sp + 2 * cp * cx ** 2 * cy ** 2 * r32 * r33 * sp * st + 2 * cp * cx ** 2 * cz ** 2 * r22 * r23 * sp * st + 2 * cp * cy ** 2 * cz ** 2 * r12 * r13 * sp * st + cx ** 2 * cy ** 2 * r33 ** 2 * sp ** 2 + cx ** 2 * cz ** 2 * r23 ** 2 * sp ** 2 + cy ** 2 * cz ** 2 * r13 ** 2 * sp ** 2) ** (
                     1 / 2) + ax * cy ** 2 * cz ** 2 * r13 * sp + ay * cx ** 2 * cz ** 2 * r23 * sp + az * cx ** 2 * cy ** 2 * r33 * sp + ax * cp * ct * cy ** 2 * cz ** 2 * r11 + ay * cp * ct * cx ** 2 * cz ** 2 * r21 + az * cp * ct * cx ** 2 * cy ** 2 * r31 + ax * cp * cy ** 2 * cz ** 2 * r12 * st + ay * cp * cx ** 2 * cz ** 2 * r22 * st + az * cp * cx ** 2 * cy ** 2 * r32 * st) / (
                    cp ** 2 * ct ** 2 * cx ** 2 * cy ** 2 * r31 ** 2 + cp ** 2 * ct ** 2 * cx ** 2 * cz ** 2 * r21 ** 2 + cp ** 2 * ct ** 2 * cy ** 2 * cz ** 2 * r11 ** 2 + 2 * cp ** 2 * ct * cx ** 2 * cy ** 2 * r31 * r32 * st + 2 * cp ** 2 * ct * cx ** 2 * cz ** 2 * r21 * r22 * st + 2 * cp ** 2 * ct * cy ** 2 * cz ** 2 * r11 * r12 * st + cp ** 2 * cx ** 2 * cy ** 2 * r32 ** 2 * st ** 2 + cp ** 2 * cx ** 2 * cz ** 2 * r22 ** 2 * st ** 2 + cp ** 2 * cy ** 2 * cz ** 2 * r12 ** 2 * st ** 2 + 2 * cp * ct * cx ** 2 * cy ** 2 * r31 * r33 * sp + 2 * cp * ct * cx ** 2 * cz ** 2 * r21 * r23 * sp + 2 * cp * ct * cy ** 2 * cz ** 2 * r11 * r13 * sp + 2 * cp * cx ** 2 * cy ** 2 * r32 * r33 * sp * st + 2 * cp * cx ** 2 * cz ** 2 * r22 * r23 * sp * st + 2 * cp * cy ** 2 * cz ** 2 * r12 * r13 * sp * st + cx ** 2 * cy ** 2 * r33 ** 2 * sp ** 2 + cx ** 2 * cz ** 2 * r23 ** 2 * sp ** 2 + cy ** 2 * cz ** 2 * r13 ** 2 * sp ** 2)
        (ax * cy ** 2 * cz ** 2 * r13 * sp - cx * cy * cz * (
                    - ax ** 2 * cp ** 2 * ct ** 2 * cy ** 2 * r31 ** 2 - ax ** 2 * cp ** 2 * ct ** 2 * cz ** 2 * r21 ** 2 - 2 * ax ** 2 * cp ** 2 * ct * cy ** 2 * r31 * r32 * st - 2 * ax ** 2 * cp ** 2 * ct * cz ** 2 * r21 * r22 * st - ax ** 2 * cp ** 2 * cy ** 2 * r32 ** 2 * st ** 2 - ax ** 2 * cp ** 2 * cz ** 2 * r22 ** 2 * st ** 2 - 2 * ax ** 2 * cp * ct * cy ** 2 * r31 * r33 * sp - 2 * ax ** 2 * cp * ct * cz ** 2 * r21 * r23 * sp - 2 * ax ** 2 * cp * cy ** 2 * r32 * r33 * sp * st - 2 * ax ** 2 * cp * cz ** 2 * r22 * r23 * sp * st - ax ** 2 * cy ** 2 * r33 ** 2 * sp ** 2 - ax ** 2 * cz ** 2 * r23 ** 2 * sp ** 2 + 2 * ax * ay * cp ** 2 * ct ** 2 * cz ** 2 * r11 * r21 + 2 * ax * ay * cp ** 2 * ct * cz ** 2 * r11 * r22 * st + 2 * ax * ay * cp ** 2 * ct * cz ** 2 * r12 * r21 * st + 2 * ax * ay * cp ** 2 * cz ** 2 * r12 * r22 * st ** 2 + 2 * ax * ay * cp * ct * cz ** 2 * r11 * r23 * sp + 2 * ax * ay * cp * ct * cz ** 2 * r13 * r21 * sp + 2 * ax * ay * cp * cz ** 2 * r12 * r23 * sp * st + 2 * ax * ay * cp * cz ** 2 * r13 * r22 * sp * st + 2 * ax * ay * cz ** 2 * r13 * r23 * sp ** 2 + 2 * ax * az * cp ** 2 * ct ** 2 * cy ** 2 * r11 * r31 + 2 * ax * az * cp ** 2 * ct * cy ** 2 * r11 * r32 * st + 2 * ax * az * cp ** 2 * ct * cy ** 2 * r12 * r31 * st + 2 * ax * az * cp ** 2 * cy ** 2 * r12 * r32 * st ** 2 + 2 * ax * az * cp * ct * cy ** 2 * r11 * r33 * sp + 2 * ax * az * cp * ct * cy ** 2 * r13 * r31 * sp + 2 * ax * az * cp * cy ** 2 * r12 * r33 * sp * st + 2 * ax * az * cp * cy ** 2 * r13 * r32 * sp * st + 2 * ax * az * cy ** 2 * r13 * r33 * sp ** 2 - ay ** 2 * cp ** 2 * ct ** 2 * cx ** 2 * r31 ** 2 - ay ** 2 * cp ** 2 * ct ** 2 * cz ** 2 * r11 ** 2 - 2 * ay ** 2 * cp ** 2 * ct * cx ** 2 * r31 * r32 * st - 2 * ay ** 2 * cp ** 2 * ct * cz ** 2 * r11 * r12 * st - ay ** 2 * cp ** 2 * cx ** 2 * r32 ** 2 * st ** 2 - ay ** 2 * cp ** 2 * cz ** 2 * r12 ** 2 * st ** 2 - 2 * ay ** 2 * cp * ct * cx ** 2 * r31 * r33 * sp - 2 * ay ** 2 * cp * ct * cz ** 2 * r11 * r13 * sp - 2 * ay ** 2 * cp * cx ** 2 * r32 * r33 * sp * st - 2 * ay ** 2 * cp * cz ** 2 * r12 * r13 * sp * st - ay ** 2 * cx ** 2 * r33 ** 2 * sp ** 2 - ay ** 2 * cz ** 2 * r13 ** 2 * sp ** 2 + 2 * ay * az * cp ** 2 * ct ** 2 * cx ** 2 * r21 * r31 + 2 * ay * az * cp ** 2 * ct * cx ** 2 * r21 * r32 * st + 2 * ay * az * cp ** 2 * ct * cx ** 2 * r22 * r31 * st + 2 * ay * az * cp ** 2 * cx ** 2 * r22 * r32 * st ** 2 + 2 * ay * az * cp * ct * cx ** 2 * r21 * r33 * sp + 2 * ay * az * cp * ct * cx ** 2 * r23 * r31 * sp + 2 * ay * az * cp * cx ** 2 * r22 * r33 * sp * st + 2 * ay * az * cp * cx ** 2 * r23 * r32 * sp * st + 2 * ay * az * cx ** 2 * r23 * r33 * sp ** 2 - az ** 2 * cp ** 2 * ct ** 2 * cx ** 2 * r21 ** 2 - az ** 2 * cp ** 2 * ct ** 2 * cy ** 2 * r11 ** 2 - 2 * az ** 2 * cp ** 2 * ct * cx ** 2 * r21 * r22 * st - 2 * az ** 2 * cp ** 2 * ct * cy ** 2 * r11 * r12 * st - az ** 2 * cp ** 2 * cx ** 2 * r22 ** 2 * st ** 2 - az ** 2 * cp ** 2 * cy ** 2 * r12 ** 2 * st ** 2 - 2 * az ** 2 * cp * ct * cx ** 2 * r21 * r23 * sp - 2 * az ** 2 * cp * ct * cy ** 2 * r11 * r13 * sp - 2 * az ** 2 * cp * cx ** 2 * r22 * r23 * sp * st - 2 * az ** 2 * cp * cy ** 2 * r12 * r13 * sp * st - az ** 2 * cx ** 2 * r23 ** 2 * sp ** 2 - az ** 2 * cy ** 2 * r13 ** 2 * sp ** 2 + cp ** 2 * ct ** 2 * cx ** 2 * cy ** 2 * r31 ** 2 + cp ** 2 * ct ** 2 * cx ** 2 * cz ** 2 * r21 ** 2 + cp ** 2 * ct ** 2 * cy ** 2 * cz ** 2 * r11 ** 2 + 2 * cp ** 2 * ct * cx ** 2 * cy ** 2 * r31 * r32 * st + 2 * cp ** 2 * ct * cx ** 2 * cz ** 2 * r21 * r22 * st + 2 * cp ** 2 * ct * cy ** 2 * cz ** 2 * r11 * r12 * st + cp ** 2 * cx ** 2 * cy ** 2 * r32 ** 2 * st ** 2 + cp ** 2 * cx ** 2 * cz ** 2 * r22 ** 2 * st ** 2 + cp ** 2 * cy ** 2 * cz ** 2 * r12 ** 2 * st ** 2 + 2 * cp * ct * cx ** 2 * cy ** 2 * r31 * r33 * sp + 2 * cp * ct * cx ** 2 * cz ** 2 * r21 * r23 * sp + 2 * cp * ct * cy ** 2 * cz ** 2 * r11 * r13 * sp + 2 * cp * cx ** 2 * cy ** 2 * r32 * r33 * sp * st + 2 * cp * cx ** 2 * cz ** 2 * r22 * r23 * sp * st + 2 * cp * cy ** 2 * cz ** 2 * r12 * r13 * sp * st + cx ** 2 * cy ** 2 * r33 ** 2 * sp ** 2 + cx ** 2 * cz ** 2 * r23 ** 2 * sp ** 2 + cy ** 2 * cz ** 2 * r13 ** 2 * sp ** 2) ** (
                     1 / 2) + ay * cx ** 2 * cz ** 2 * r23 * sp + az * cx ** 2 * cy ** 2 * r33 * sp + ax * cp * ct * cy ** 2 * cz ** 2 * r11 + ay * cp * ct * cx ** 2 * cz ** 2 * r21 + az * cp * ct * cx ** 2 * cy ** 2 * r31 + ax * cp * cy ** 2 * cz ** 2 * r12 * st + ay * cp * cx ** 2 * cz ** 2 * r22 * st + az * cp * cx ** 2 * cy ** 2 * r32 * st) / (
                    cp ** 2 * ct ** 2 * cx ** 2 * cy ** 2 * r31 ** 2 + cp ** 2 * ct ** 2 * cx ** 2 * cz ** 2 * r21 ** 2 + cp ** 2 * ct ** 2 * cy ** 2 * cz ** 2 * r11 ** 2 + 2 * cp ** 2 * ct * cx ** 2 * cy ** 2 * r31 * r32 * st + 2 * cp ** 2 * ct * cx ** 2 * cz ** 2 * r21 * r22 * st + 2 * cp ** 2 * ct * cy ** 2 * cz ** 2 * r11 * r12 * st + cp ** 2 * cx ** 2 * cy ** 2 * r32 ** 2 * st ** 2 + cp ** 2 * cx ** 2 * cz ** 2 * r22 ** 2 * st ** 2 + cp ** 2 * cy ** 2 * cz ** 2 * r12 ** 2 * st ** 2 + 2 * cp * ct * cx ** 2 * cy ** 2 * r31 * r33 * sp + 2 * cp * ct * cx ** 2 * cz ** 2 * r21 * r23 * sp + 2 * cp * ct * cy ** 2 * cz ** 2 * r11 * r13 * sp + 2 * cp * cx ** 2 * cy ** 2 * r32 * r33 * sp * st + 2 * cp * cx ** 2 * cz ** 2 * r22 * r23 * sp * st + 2 * cp * cy ** 2 * cz ** 2 * r12 * r13 * sp * st + cx ** 2 * cy ** 2 * r33 ** 2 * sp ** 2 + cx ** 2 * cz ** 2 * r23 ** 2 * sp ** 2 + cy ** 2 * cz ** 2 * r13 ** 2 * sp ** 2)

        
    def scan(self, Drone):

        """
        Scan around the drone to determine the locations of the obstacles. Use this to formulate the DVZ as in the dissertation.
        """

        # Initialise all to 40m (the maximum range of the chosen rangefinder)

        self.distances = np.ones((self.allAngles.shape[0])) * 40

        self.I = 0
        # Determine the Drone's rotation matrix w.r.t. the world frame so that all angles are relative to the Drone
        M = np.array(pbl.getMatrixFromQuaternion(Drone.quat)).reshape((3, 3))
        allObstacletheta = [np.arctan2(obstacle.pos[1] - Drone.pos[1], obstacle.pos[0] - Drone.pos[0]) for obstacle in Drone.obstacles[:-1]]
        allObstaclephi =   [np.arccos((obstacle.pos[2] - Drone.pos[2]) / np.linalg.norm(obstacle.pos - Drone.pos)) for obstacle in Drone.obstacles[:-1]]

        # Don't need these anymore but I've kept it in in case it messes anything else up
        allUpperBound = [obstacle.bbox2[1] for obstacle in Drone.obstacles[:-1]]
        allLowerBound = [obstacle.bbox2[0] for obstacle in Drone.obstacles[:-1]]
        
        # Determine the angles to all of the obstacles using theta and phi as defined in spherical coordinates
        thetas = np.array([allObstacletheta[i] + np.arange(np.deg2rad(0), np.deg2rad(self.angleStep), self.angleStep) for i in range(len(allObstacletheta))]).flatten()
        phis = np.array([allObstaclephi[i] + np.arange(np.deg2rad(0), np.deg2rad(self.angleStep), self.angleStep) for i in range(len(allObstaclephi))]).flatten()

        # loop over all the obstacles
        for i, theta, phi in zip(range(len(Drone.obstacles[:-1])), list(dict.fromkeys(thetas)), list(dict.fromkeys(phis))):
            D_h = self.setDh(theta, phi, pbl.getMatrixFromQuaternion(Drone.quat))
            if D_h > 40:
                D_h = 40
            
            D = abs(Drone.obstacles[i].getDistance(Drone))
            index = np.argmin(np.linalg.norm(self.allAngles - [theta, phi], axis=1))

            # We use the if else statement as a safeguard in case an obstacle which is at the same angle as another obstacle but closer to the drone appears later in the loop. This is highly unlikely so can effectively ignore the if else
            self.distances[index - 1:index + 2] = D**10 if D <= self.distances[index] else self.distances[index]
            self.I += 3 * ((D_h - D)/D) * (self.angleStep**2)
                    
        # Append the distance and angle to goal information to the DVZ list
        distanceToGoal = np.linalg.norm(Drone.setpoint - Drone.pos)
        horAngleToGoal = np.arctan2(Drone.setpoint[1] - Drone.pos[1], Drone.setpoint[0] - Drone.pos[0])
        vertAngleToGoal = np.arccos((Drone.setpoint[2] - Drone.pos[2])/np.linalg.norm(Drone.setpoint - Drone.pos))
        
        goalInputs = [distanceToGoal**10, horAngleToGoal*1e10, vertAngleToGoal*1e10] * 30 + [distanceToGoal**10] * 4
        
        self.distances = np.hstack((self.distances, np.array(goalInputs)))

        # This multiplication makes it so that the Intrustion is similar in magnitude to ITSE. Prevents any favouritism of one metric over the other
        self.I *= 1e-3

        # Recast the four frames to include the current frame
        self.distHistory.append(self.distances)
        if len(self.distHistory) > 4:
            self.distHistory = self.distHistory[1:]
