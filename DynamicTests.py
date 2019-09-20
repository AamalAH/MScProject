import pybullet as pbl
import matplotlib.pyplot as plt
import time
import numpy as np
import pybullet_data
from Drone import Drone

obstacleURDFs = ["block.urdf"]

def generateObstacles(no_obs, start, goal):
    ObstacleIDs = []
    for obs in range(no_obs):
        loc = np.array([np.random.normal(scale=1), np.random.uniform(-3, 8), np.random.uniform(0, 5)])
        while np.linalg.norm(loc - goal)  <= 1 or np.linalg.norm(loc - start)  <= 1:
            loc = np.array([np.random.normal(scale=1), np.random.uniform(-3, 5), np.random.uniform(0, 5)])
        ObstacleIDs.append(pbl.loadURDF(np.random.choice(obstacleURDFs), loc))

    ObstacleIDs.append(pbl.loadURDF("plane.urdf"))

    return ObstacleIDs

def hasCollided(Drone, obstacle):
    loc, orient = pbl.getBasePositionAndOrientation(obstacle.ID)
    if obstacle.getDistance(Drone) <= 0.03 or (Drone.pos[2] < 0.1 and np.linalg.norm(Drone.xdot) <= 0.05):
        return True
    else:
        return False

def runTest():

    # Create Physics Client
    physicsClient = pbl.connect(pbl.DIRECT)
    pbl.setAdditionalSearchPath(pybullet_data.getDataPath())
    pbl.setGravity(0, 0, -9.8)


    # Generate environment
    no_obs = 13      
    ObstacleIDs = generateObstacles(no_obs, [0, 0, 0.5], [0., 5., 0.5])
    Qimmiq = Drone(ObstacleIDs)

    Collided = False

    velocities = [np.random.normal(size=(3, )) for i in range(len(ObstacleIDs[:-1]))]
    # Run the simulation
    meanVel = np.linalg.norm(np.mean(velocities, axis=0))
    while not Qimmiq.targetReached and not Collided and Qimmiq.time < 120:
        # Ensure that obstacles remain static
        [pbl.resetBaseVelocity(ObstacleIDs[i], velocities[i], [0, 0, 0]) for i in range(len(ObstacleIDs[:-1]))]

        Qimmiq.moveDrone(Qimmiq.time_interval, 0)
        # Search for any Collisions
        for obstacle in Qimmiq.obstacles[:-1]:
            if hasCollided(Qimmiq, obstacle):
                Collided = True
                print('Collision!')
                break

        pbl.stepSimulation()
        time.sleep(Qimmiq.time_interval)

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

        return (no_obs, plannerCalcTime, Deviation, Curvature, lifeTime, pathTime, pathLength, inputDeviation, Collided, True, meanVel)
    else:
        pbl.disconnect()
        return (0, 0, 0, 0, 0, 0, 0, 0, False, False, meanVel)

def main(no_tests):
    """
    Given number of tests to run, produce as many simulations and return the evaluation metrics

    :return: Final evaluation metrics of method
    """
    nCollided = 0
    obstacleNumbers = []
    plannerCalcTimes = []
    Deviations = []
    Curvatures = []
    lifeTimes = []
    pathTimes = []
    pathLengths = []
    inputDeviations = []
    meanVels = []

    for test in range(no_tests):

        print('Running Test {0} of {1}'.format(test + 1, no_tests))
        obstacleNumber, plannerCalcTime, Deviation, Curvature, lifeTime, pathTime, pathLength, inputDeviation, Collided, madeIt, meanVel = runTest()
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
        meanVels.append(meanVel)

    return (obstacleNumbers, plannerCalcTimes, Deviations, Curvatures, lifeTimes, pathTimes, pathLengths, inputDeviations, nCollided, meanVels)

if __name__ == "__main__":
    numberOFTests = 15
    
    obstacleNumbers, plannerCalcTimes, Deviations, Curvatures, lifeTimes, pathTimes, pathLengths, inputDeviations, nCollided, meanVels = main(numberOFTests)
    data = {'obstacleNumbers': obstacleNumbers,
            'plannerCalcTimes': plannerCalcTimes,
            'Deviations': Deviations,
            'Curvatures': Curvatures,
            'lifeTimes': lifeTimes,
            'pathTimes': pathTimes,
            'pathLengths': pathLengths,
            'inputDeviations': inputDeviations,
            'meanVels': meanVels,
            'nCollided': nCollided,
            }
    with open('DynamicData.txt', 'a') as file:
        file.write(str(data) + '\n')