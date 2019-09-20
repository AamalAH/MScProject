#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 23 20:01:40 2019

@author: aamalh
"""

from GNN import *
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd


fig = plt.figure()
ax = fig.add_subplot(111, xlim = (-7.5, 7.5), ylim = (-5, 10), zlim = (0, 10), projection = '3d')

fig2 = plt.figure()
ax2 = fig2.add_subplot(111)

fig3 = plt.figure()
ax3 = fig3.add_subplot(111)


ax.set_xlabel('X Position (m)'), ax.set_ylabel('Y Position (m)'), ax.set_zlabel('Z Position (m)')

net = Net()
import pandas as pd
"""
We will use:
    GNNSolLifeTime
    GNNSolGoalOnly
    VEGASolObsWithMaxDVZ
    WSGASol
"""
planner = 'WSGASol'
path = 'GNNSolutions/'

b = pd.read_csv(path + planner + '.csv')
b = np.array(b).squeeze()
parameterLengths = [np.product(f.size()) for f in net.parameters()]
index = 0
params = []

for l in range(0, len(parameterLengths), 2):
    params.append(b[index:index + parameterLengths[l]])
    index += parameterLengths[l]
    params.append(np.ones(parameterLengths[l+1]) * b[index])
    index += 1
for j, f in enumerate(net.parameters()):
    f.data = torch.nn.Parameter(torch.tensor(params[j].reshape((f.shape))))
    
def generateObstacles(no_obs, start, ObstacleIDs, ObstacleLocations, wallLocations):
    for obs in range(no_obs):
        loc = np.array([np.random.uniform(-4, 4), np.random.uniform(0, 7), np.random.uniform(0, 4)])
        while np.any(np.linalg.norm(np.subtract(loc, np.array(wallLocations)), axis = 1) <= 1) or np.linalg.norm(loc - start)  <= 1:
            loc = np.array([np.random.uniform(-4, 4), np.random.uniform(0, 7), np.random.uniform(0, 4)])
        ObstacleIDs.append(pbl.loadURDF('block.urdf', loc))
        bbox = pbl.getAABB(ObstacleIDs[-1])
        dims = np.subtract(bbox[1], bbox[0])
        disk = Rectangle(bbox[0][0::2], dims[0], dims[2], color='r', fill = False)
        ax.add_patch(disk)
        art3d.pathpatch_2d_to_3d(disk, z=bbox[1][1], zdir='y')
        ObstacleLocations.append(tuple(loc))

    ObstacleIDs.append(pbl.loadURDF('plane.urdf', [0, 0, 0]))
    
    return ObstacleIDs, ObstacleLocations

def hasCollided(Drone, obstacle):
    if len(pbl.getClosestPoints(Drone.droneID, obstacle.ID, 40, -1, -1)) > 0:
        D = pbl.getClosestPoints(Drone.droneID, obstacle.ID, 40, -1, -1)[0][8]
    else:
        D = np.inf
    if D <= 0.03 or (Drone.pos[2] < 0.1 and np.linalg.norm(Drone.xdot) <= 0.05):
        return True
    else:
        return False


def runSim(net, runType = pbl.DIRECT):
    physicsClient = pbl.connect(runType)
    pbl.setAdditionalSearchPath(pybullet_data.getDataPath())

    pbl.setGravity(0, 0, -9.8)

    goal = [0, 5, 0.5]
    L = 5
    dist = np.array([0, -L, 0.5])

    ObstacleIDs = []
    ObstacleLocations = []
    ObstacleOrientations = []

    ObstacleIDs.append(pbl.loadURDF("/Misc/Enclosure/Wall.urdf", basePosition=[0, -1, 5.5]))
    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][0::2], dims[0], dims[2], color='r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z=bbox[1][1], zdir='y')

    ObstacleIDs.append(pbl.loadURDF("/Misc/Enclosure/Wall.urdf", basePosition=[0, 11, 5.5]) )
    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][0::2], dims[0], dims[2], color='r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z=bbox[1][1], zdir='y')
    
    ObstacleIDs.append(pbl.loadURDF("/Misc/Enclosure/Wall.urdf", basePosition=[-6, 5, 5.5], baseOrientation=pbl.getQuaternionFromEuler([0, 0, -np.pi/2])))
    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][1:], dims[1], dims[2], color='r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z=bbox[1][0], zdir='x')

    ObstacleIDs.append(pbl.loadURDF("/Misc/Enclosure/Wall.urdf", basePosition=[6, 5, 5.5], baseOrientation=pbl.getQuaternionFromEuler([0, 0, -np.pi/2])))
    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][1:], dims[1], dims[2], color='r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z=bbox[1][0], zdir='x')

    ObstacleIDs.append(pbl.loadURDF("/Misc/Enclosure/Wall.urdf", basePosition=[0, 5, 11.5], baseOrientation=pbl.getQuaternionFromEuler([-np.pi/2, 0, 0])))
    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][0:2], dims[0], dims[1], color='r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z=bbox[0][2], zdir='z')

    [(ObstacleLocations.append(pbl.getBasePositionAndOrientation(obs)[0]), ObstacleOrientations.append(pbl.getBasePositionAndOrientation(obs)[1])) for obs in ObstacleIDs]

    start = np.array([np.random.uniform(-4, 4), np.random.uniform(0, 7), np.random.uniform(0, 4)])
    while np.any(np.linalg.norm(np.subtract(start, np.array(ObstacleLocations)), axis = 1) <= 1):
        start = np.array([np.random.uniform(-4, 4), np.random.uniform(0, 7), np.random.uniform(0, 4)])

    ObstacleIDs, ObstacleLocations = generateObstacles(5, start, ObstacleIDs, ObstacleLocations, ObstacleLocations)


    Qimmiq = Drone(ObstacleIDs, net=net, init_pos=start)
    ax.scatter(start[0], start[1], start[2], color='green', marker='o')

    i = 0

    Collided = False

    while (not Qimmiq.targetReached and not Collided):
        
        for obstacle in Qimmiq.obstacles[:-1]:
            if hasCollided(Qimmiq, obstacle):   
                Collided = True
                print('Collision!')
                break

        [pbl.resetBasePositionAndOrientation(ObstacleIDs[i], ObstacleLocations[i], ObstacleOrientations[i]) for i in range(len(ObstacleOrientations))]
        if i % 240 == 0:
            velocities = [np.random.normal(size=(3, )) for k in range(len(ObstacleIDs[:-1]))]

        [pbl.resetBaseVelocity(ObstacleIDs[j], velocities[j], [0, 0, 0]) for j in range(len(ObstacleOrientations), len(ObstacleIDs[:-1]))]
            
        i += 1

        Qimmiq.moveDrone(Qimmiq.time_interval, i)
        pbl.stepSimulation()
    pbl.disconnect()

    dronePath, = ax.plot(Qimmiq.all_state[1:, 0], Qimmiq.all_state[1:, 1], Qimmiq.all_state[1:, 2], color='green', linestyle='dashed')
    ax.scatter(Qimmiq.pos[0], Qimmiq.pos[1], Qimmiq.pos[2], color='green', marker='+')

    print(planner + ': {}'.format(Qimmiq.time))

    error = np.linalg.norm(Qimmiq.all_state[1:, 0:3] - np.array([0, 5, 0.5]), axis=1)
    error = pd.DataFrame(np.hstack((Qimmiq.time, error)))
    errors = pd.concat((pd.read_csv('GNNSolutions/GotoGoal/' + planner + '.csv'), error), axis=0)
    error.to_csv('GNNSolutions/GotoGoal/' + planner + '.csv', index = None, sep = ' ', mode = 'a')
    error.to_csv('GNNSolutions/GotoGoal/' + planner + '.csv', index = None, sep = ' ', mode = 'a')


    Qimmiq.networkOutputs = np.array(Qimmiq.networkOutputs)
    ax2.plot(Qimmiq.networkOutputs[:, 0], 'r--', label = 'Roll')
    ax2.plot(Qimmiq.networkOutputs[:, 1], 'k--', label = 'Pitch')
    ax2.plot(Qimmiq.networkOutputs[:, 2], 'b--', label = 'Vertical')
    plt.legend()

    plt.show()
    
runSim(net)