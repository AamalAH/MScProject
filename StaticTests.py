import time

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d
import numpy as np
import pandas as pd
import pybullet as pbl
import pybullet_data
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D

from Drone import Drone

obstacleURDFs = ["block.urdf"]

def generateObstacles(no_obs, start, ObstacleIDs, ObstacleLocations, wallLocations):
    for obs in range(no_obs):
        loc = np.array([np.random.uniform(-4, 4), np.random.uniform(0, 7), np.random.uniform(0, 4)])
        while np.any(np.linalg.norm(np.subtract(loc, np.array(wallLocations)), axis = 1) <= 1) or np.linalg.norm(loc - start)  <= 1:
            loc = np.array([np.random.uniform(-4, 4), np.random.uniform(0, 7), np.random.uniform(0, 4)])
        ObstacleIDs.append(pbl.loadURDF('block.urdf', loc))
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

def runTest(sensorNoise = 0):

    # Create Physics Client
    physicsClient = pbl.connect(pbl.DIRECT)
    pbl.setAdditionalSearchPath(pybullet_data.getDataPath())
    pbl.setGravity(0, 0, -9.8)


    ObstacleIDs = []
    ObstacleLocations = []
    ObstacleOrientations = []

    ObstacleIDs.append(pbl.loadURDF("/Misc/Enclosure/Wall.urdf", basePosition=[0, -1, 5.5]))
    ObstacleIDs.append(pbl.loadURDF("/Misc/Enclosure/Wall.urdf", basePosition=[0, 11, 5.5]) )
    ObstacleIDs.append(pbl.loadURDF("/Misc/Enclosure/Wall.urdf", basePosition=[-6, 5, 5.5], baseOrientation=pbl.getQuaternionFromEuler([0, 0, -np.pi/2])))
    ObstacleIDs.append(pbl.loadURDF("/Misc/Enclosure/Wall.urdf", basePosition=[6, 5, 5.5], baseOrientation=pbl.getQuaternionFromEuler([0, 0, -np.pi/2])))
    ObstacleIDs.append(pbl.loadURDF("/Misc/Enclosure/Wall.urdf", basePosition=[0, 5, 11.5], baseOrientation=pbl.getQuaternionFromEuler([-np.pi/2, 0, 0])))

    [(ObstacleLocations.append(pbl.getBasePositionAndOrientation(obs)[0]), ObstacleOrientations.append(pbl.getBasePositionAndOrientation(obs)[1])) for obs in ObstacleIDs]

    dir = np.random.normal(0, 1, size=(3, ))
    dir /= np.linalg.norm(dir)
    start = [0, 5, 0.5] + dir * 5

    while np.any(np.linalg.norm(np.subtract(start, np.array(ObstacleLocations)), axis = 1) <= 1) or np.linalg.norm(start - [0, 5, 0.5]) <= 1:
        dir = np.random.normal(0, 1, size=(3, ))
        dir /= np.linalg.norm(dir)
        start = [0, 5, 0.5] + dir * 5
    
    no_obs = 4  

    ObstacleIDs, ObstacleLocations = generateObstacles(no_obs, start, ObstacleIDs, ObstacleLocations, ObstacleLocations)


    # Generate environment
    Qimmiq = Drone(ObstacleIDs, sensorNoise = sensorNoise)

    # init = ax.scatter([0], [0], [0.5], color = 'g', marker = 'o')

    Collided = False 

    # Run the simulation
    while not Qimmiq.targetReached and not Collided and Qimmiq.time < 60:
        # Ensure that obstacles remain static
        [pbl.resetBasePositionAndOrientation(ObstacleIDs[i], ObstacleLocations[i], pbl.getQuaternionFromEuler([0, 0, 0])) for i in range(len(ObstacleLocations))]
        # [pbl.resetBaseVelocity(obs, [0, 0, 0], [0, 0, 0]) for obs in ObstacleIDs]
        Qimmiq.moveDrone(Qimmiq.time_interval, 0)
        # Search for any Collisions
        for obstacle in Qimmiq.obstacles[:-1]:
            if hasCollided(Qimmiq, obstacle):   
                Collided = True
                print('Collision!')
                break

        pbl.stepSimulation()
        # time.sleep(1/240)

    # path, = ax.plot(Qimmiq.all_state[1:, 0], Qimmiq.all_state[1:, 1], Qimmiq.all_state[1:, 2], color = 'g', linestyle = 'dashed', label = 'Drone Path')
    # goal = ax.scatter(Qimmiq.pos[0], Qimmiq.pos[1], Qimmiq.pos[2], color = 'g', marker = 'P')

    # ax.legend([init, path, goal, ax.patches[-1]] , ['Drone Initial Position', 'Drone path', 'Drone Final Position', 'Obstacle Final Position'])

    # plt.show()

    if Qimmiq.time < 120:
        plannerCalcTime = np.mean(Qimmiq.times)
        Deviation = Qimmiq.planner.evaluate_deviation()
        Curvature = Qimmiq.planner.evaluate_curvature(Qimmiq)

        if Collided:
            lifeTime = Qimmiq.time
            pathTime = np.inf
        else:
            lifeTime = np.inf
            pathTime = Qimmiq.time

        pathLength = Qimmiq.pathLength
        inputDeviation = Qimmiq.planner.evaluate_input_deviation(Qimmiq)

        pbl.disconnect()

        return (no_obs, plannerCalcTime, Deviation, Curvature, lifeTime, pathTime, pathLength, inputDeviation, Collided, True)
    else:
        pbl.disconnect()
        return (0, 0, 0, 0, 0, 0, 0, 0, False, False)

def main(no_tests, maxSensorNoise):
    """
    Given number of tests to run, produce as many simulations and return the evaluation metrics

    :return: Final evaluation metrics of method
    """


    for sensorNoise in np.arange(0, maxSensorNoise, 0.5):
        nCollided = 0
        obstacleNumbers = []
        plannerCalcTimes = []
        Deviations = []
        Curvatures = []
        lifeTimes = []
        pathTimes = []
        pathLengths = []
        inputDeviations = []
        completedTests = 0

        print('Sensor Noise: {0}'.format(sensorNoise))

        with open('SensorNoiseTests.txt', 'a') as file:
            file.write('Sensor Noise: ' + str(sensorNoise) + '\n')

        for test in range(no_tests):

            print('Running Test {0} of {1}'.format(test + 1, no_tests))
            obstacleNumber, plannerCalcTime, Deviation, Curvature, lifeTime, pathTime, pathLength, inputDeviation, Collided, madeIt = runTest()
            if madeIt:
                completedTests += 1
                obstacleNumbers.append(obstacleNumber)
                plannerCalcTimes.append(plannerCalcTime)
                Deviations.append(Deviation)
                Curvatures.append(Curvature)
                if not lifeTime == np.inf:
                    lifeTimes.append(lifeTime)
                if not pathTime == np.inf:
                    pathTimes.append(pathTime)
                pathLengths.append(pathLength)
                inputDeviations.append(inputDeviation)
                if Collided:
                    nCollided += 1

            else:
                continue

        data = {'commsNoise': sensorNoise,
                'Completed Tests': completedTests,
                'nCollided': nCollided
                }

        with open('SensorNoiseTests.txt', 'a') as file:
            file.write(str(data) + '\n')

    return (obstacleNumbers, plannerCalcTimes, Deviations, Curvatures, lifeTimes, pathTimes, pathLengths, inputDeviations, nCollided)

if __name__ == "__main__":
    numberOFTests = 40
    maxSensorNoise = 5

    # fig = plt.figure()
    # ax = fig.add_subplot(111, xlim = (-7.5, 7.5), ylim = (-5, 10), zlim = (0, 10), projection = '3d')

    # ax.set_xlabel('X Position (m)'), ax.set_ylabel('Y Position (m)'), ax.set_zlabel('Z Position (m)')

    obstacleNumbers, plannerCalcTimes, Deviations, Curvatures, lifeTimes, pathTimes, pathLengths, inputDeviations, nCollided = main(numberOFTests, maxSensorNoise)
    data = {'obstacleNumbers': obstacleNumbers,
            'plannerCalcTimes': plannerCalcTimes,
            'Deviations': Deviations,
            'Curvatures': Curvatures,
            'lifeTimes': lifeTimes,
            'pathTimes': pathTimes,
            'pathLengths': pathLengths,
            'inputDeviations': inputDeviations,
            'nCollided': nCollided
            }
    with open('StaticData.txt', 'a') as file:
        file.write(str(data) + '\n')
