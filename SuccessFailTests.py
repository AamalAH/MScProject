import pybullet as pbl
import time
import numpy as np
import pybullet_data
from Drone import Drone
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Rectangle
import pandas as pd


obstacleURDFs = ["block.urdf"]

def generateHorseShoeinPath():
    ObstacleIDs = []

    ObstacleIDs.append(pbl.loadURDF("/Misc/HorseShoeinPath/blockback.urdf", [0, 5, 3]))
    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][0::2], dims[0], dims[2], color = 'r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z = bbox[1][1], zdir = 'y')


    ObstacleIDs.append(pbl.loadURDF("/Misc/HorseShoeinPath/blockSide.urdf", [0, 3, 5]))

    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][0:2], dims[0], dims[1], color = 'r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z = bbox[1][2], zdir = 'z')

    ObstacleIDs.append(pbl.loadURDF("/Misc/HorseShoeinPath/blockSide.urdf", [0, 3, 1]))

    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][0:2], dims[0], dims[1], color = 'r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z = bbox[0][2], zdir = 'z')

    ObstacleIDs.append(pbl.loadURDF("/Misc/HorseShoeinPath/blockSideActual.urdf", [2, 3, 3]))

    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][1:], dims[1], dims[2], color = 'r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z = bbox[1][0], zdir = 'x')

    ObstacleIDs.append(pbl.loadURDF("/Misc/HorseShoeinPath/blockSideActual.urdf", [-2, 3, 3]))

    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][1:], dims[1], dims[2], color = 'r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z = bbox[0][0], zdir = 'x')

    ObstacleIDs.append(pbl.loadURDF('plane.urdf'))

    return ObstacleIDs

def generateHorseShoeAroundGoal(size):
    ObstacleIDs = []

    for i in range(int(-size-0.5), int(size+1.5)):
        ObstacleIDs.append(pbl.loadURDF('block.urdf', [i, 5 - size - 0.5, 0.5]))
        bbox = pbl.getAABB(ObstacleIDs[-1])
        dims = np.subtract(bbox[1], bbox[0])
        disk = Rectangle(bbox[0][0::2], dims[0], dims[2], color = 'r', fill = False)
        ax.add_patch(disk)
        art3d.pathpatch_2d_to_3d(disk, z = bbox[1][1], zdir = 'y')

    for i in range(int(5 - size + 0.5), int(5 - size + 3.5)):
        ObstacleIDs.append(pbl.loadURDF('block.urdf', [int(-size-0.5), i, 0.5]))
        bbox = pbl.getAABB(ObstacleIDs[-1])
        dims = np.subtract(bbox[1], bbox[0])
        disk = Rectangle(bbox[0][1:], dims[0], dims[2], color = 'r', fill = False)
        ax.add_patch(disk)
        art3d.pathpatch_2d_to_3d(disk, z = bbox[1][0], zdir = 'x')

        ObstacleIDs.append(pbl.loadURDF('block.urdf', [int(size + 0.5), i, 0.5]))
        bbox = pbl.getAABB(ObstacleIDs[-1])
        dims = np.subtract(bbox[1], bbox[0])
        disk = Rectangle(bbox[0][1:], dims[0], dims[2], color = 'r', fill = False)
        ax.add_patch(disk)
        art3d.pathpatch_2d_to_3d(disk, z = bbox[0][0], zdir = 'x')

    ObstacleIDs.append(pbl.loadURDF('plane.urdf'))

    return ObstacleIDs

def generateNarrowPassage():

    ObstacleIDs = []
    
    ObstacleIDs.append(pbl.loadURDF("/Misc/NarrowPassage/blockTop.urdf", [0, 2.5, 10]))
    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][0::2], dims[0], dims[2], color = 'r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z = bbox[1][1], zdir = 'y')

    ObstacleIDs.append(pbl.loadURDF("/Misc/NarrowPassage/blockSide.urdf", [3, 2.5, 2.5]))
    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][0::2], dims[0], dims[2], color = 'r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z = bbox[1][1], zdir = 'y')

    ObstacleIDs.append(pbl.loadURDF("/Misc/NarrowPassage/blockSide.urdf", [-3, 2.5, 2.5]))
    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][0::2], dims[0], dims[2], color = 'r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z = bbox[1][1], zdir = 'y')

    ObstacleIDs.append(pbl.loadURDF("/Misc/NarrowPassage/blockBottom.urdf", [0, 2.5, 2.7]))
    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][0::2], dims[0], dims[2], color = 'r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z = bbox[1][1], zdir = 'y')

    ObstacleIDs.append(pbl.loadURDF('plane.urdf'))

    return ObstacleIDs

def generateBetweenObjects(distanceBetween):

    ObstacleIDs = []

    ObstacleIDs.append(pbl.loadURDF("/Misc/HorseShoeinPath/blockback.urdf", [2.5 + distanceBetween/2, 2.5, 2.5]))
    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][0::2], dims[0], dims[2], color = 'r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z = bbox[1][1], zdir = 'y')

    ObstacleIDs.append(pbl.loadURDF("/Misc/HorseShoeinPath/blockback.urdf", [-2.5 - distanceBetween/2, 2.5, 2.5]))
    bbox = pbl.getAABB(ObstacleIDs[-1])
    dims = np.subtract(bbox[1], bbox[0])
    disk = Rectangle(bbox[0][0::2], dims[0], dims[2], color = 'r', fill = False)
    ax.add_patch(disk)
    art3d.pathpatch_2d_to_3d(disk, z = bbox[1][1], zdir = 'y')

    ObstacleIDs.append(pbl.loadURDF('plane.urdf'))
    return ObstacleIDs

def generateDynamicTest(startDistance):
    ObstacleIDs = []
    ObstacleIDs.append(pbl.loadURDF('block.urdf', [0, startDistance, 0.5]))

    ObstacleIDs.append(pbl.loadURDF('plane.urdf'))

    return ObstacleIDs

def generateSensorNoiseTest():

    ObstacleIDs = []
    ObstacleLocations = []
    for obs in range(15):
        loc = np.array([np.random.normal(scale=1), np.random.uniform(-3, 8), np.random.uniform(0, 5)])
        while np.linalg.norm(loc - [0, 5, 0.5])  <= 1 or np.linalg.norm(loc - [0, 0, 0.5])  <= 1:
            loc = np.array([np.random.uniform(-3, 4), np.random.uniform(-3, 5), np.random.uniform(0, 5)])
        
        ObstacleLocations.append(loc)
        ObstacleIDs.append(pbl.loadURDF(np.random.choice(obstacleURDFs), loc))

        bbox = pbl.getAABB(ObstacleIDs[-1])
        dims = np.subtract(bbox[1], bbox[0])
        disk = Rectangle(bbox[0][0::2], dims[0], dims[2], color = 'r', fill = False)
        ax.add_patch(disk)
        art3d.pathpatch_2d_to_3d(disk, z = bbox[1][1], zdir = 'y')

    ObstacleIDs.append(pbl.loadURDF("plane.urdf"))

    return ObstacleIDs, ObstacleLocations

    return ObstacleIDs, obstacleLocations

def generateTest(testType):
    assert testType < 6, 'invalid test type; must be 0, 1, 2, 3, 4 or 5'
    if testType == 0:
        return generateDynamicTest(startDistance=10)
    elif testType == 1:
        return generateBetweenObjects(distanceBetween=0.9)
    elif testType == 2:
        return generateNarrowPassage()
    elif testType == 3:
        return generateHorseShoeAroundGoal(size=1.5)
    elif testType == 4:
        return generateHorseShoeinPath()
    elif testType == 5:
        return generateSensorNoiseTest()

def hasCollided(Drone, obstacle):
    loc, orient = pbl.getBasePositionAndOrientation(obstacle.ID)
    if np.all(abs(np.subtract(loc, Drone.pos)) <= (Drone.bbox) / 2 + obstacle.bbox / 2):
        return True
    else:
        return False

def main(testType): 


    # Create Physics Client
    physicsClient = pbl.connect(pbl.GUI)
    pbl.setAdditionalSearchPath(pybullet_data.getDataPath())
    pbl.setGravity(0, 0, -9.8)


    # Generate environment
    if testType == 5:
        ObstacleIDs, obstacleLocations = generateTest(testType)
    else:
        ObstacleIDs = generateTest(testType)

    if testType != 5:
        Qimmiq = Drone(ObstacleIDs, init_pos = [0, 0, 0.5])

        init = ax.scatter([0], [0], [0.5], color = 'g', marker = 'o')
    else:
        sensorNoise = 5
        Qimmiq = Drone(ObstacleIDs, sensorNoise = sensorNoise)

        init = ax.scatter([0], [0], [0.5], color = 'g', marker = 'o')

    Collided = False

    if testType == 0:
        obsPosition = []
        distances = []

    # Run the simulation

    while not Qimmiq.targetReached and not Collided and Qimmiq.time < 60:
        # Ensure that obstacles remain static
        
        if testType == 0:
            velocity = 1.6
            [pbl.resetBaseVelocity(ObstacleIDs[i], [0, -1 * velocity, 0], [0, 0, 0]) for i in range(len(ObstacleIDs[:-1]))]
            obsPosition.append(Qimmiq.obstacles[0].pos)
            distances.append(Qimmiq.obstacles[0].getDistance(Qimmiq))
        elif testType == 5:
            [pbl.resetBasePositionAndOrientation(ObstacleIDs[i], obstacleLocations[i], pbl.getQuaternionFromEuler([0, 0, 0])) for i in range(len(obstacleLocations))]
        else:
            [pbl.resetBaseVelocity(ObstacleIDs[i], [0, 0, 0], [0, 0, 0]) for i in range(len(ObstacleIDs[:-1]))]

            
        Qimmiq.moveDrone(1/240, 0)
        # Search for any Collisions
        for obstacle in Qimmiq.obstacles[:-1]:
            if hasCollided(Qimmiq, obstacle):
                Collided = True
                print('Collision!')
                break

        pbl.stepSimulation()
        # time.sleep(1/240)

    if testType == 0:
        
        sol = np.array(distances)
        sol = np.hstack((np.linspace(0, Qimmiq.time, Qimmiq.time_interval), sol))
        
        sol = pd.DataFrame(sol)
        sol.to_clipboard(index = False, header = True)
        
        print(Qimmiq.obstacles[0].getDistance(Qimmiq))
        print(Qimmiq.time)
        
        path, = ax.plot(Qimmiq.all_state[1:, 0], Qimmiq.all_state[1:, 1], Qimmiq.all_state[1:, 2], color = 'g', linestyle = 'dashed', label = 'Drone Path')
        goal = ax.scatter(Qimmiq.pos[0], Qimmiq.pos[1], Qimmiq.pos[2], color = 'c', marker = 'P')
        
        obsPosition = np.array(obsPosition)
        obstaclePath, = ax.plot(obsPosition[:, 0], obsPosition[:, 1], obsPosition[:, 2], color = 'r', linestyle = 'dashed', label = 'Obstacle Path')

        bbox = Qimmiq.obstacles[0].bbox2
        dims = Qimmiq.obstacles[0].bbox
        disk = Rectangle(bbox[0][0::2], dims[0], dims[2], color = 'r', fill = False)
        ax.add_patch(disk)
        art3d.pathpatch_2d_to_3d(disk, z = bbox[0][1], zdir = 'y')

        ax.legend([init, path, goal, obstaclePath, ax.patches[-1]] , ['Drone Initial Position', 'Drone path', 'Drone Final Position', 'Obstacle Path', 'Obstacle Final Position'])

    else:
        path, = ax.plot(Qimmiq.all_state[1:, 0], Qimmiq.all_state[1:, 1], Qimmiq.all_state[1:, 2], color = 'g', linestyle = 'dashed', label = 'Drone Path')
        goal = ax.scatter(Qimmiq.pos[0], Qimmiq.pos[1], Qimmiq.pos[2], color = 'g', marker = 'P')

        ax.legend([init, path, goal, ax.patches[-1]] , ['Drone Initial Position', 'Drone path', 'Drone Final Position', 'Obstacle Final Position'])

    plt.show()

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

    return (testType, plannerCalcTime, Deviation, Curvature, lifeTime, pathTime, pathLength, inputDeviation, Collided)

if __name__ == "__main__":
# 0: Dynamic Test
# 1: Gap Between Objects
# 2: Narrow Window
# 3: HorseShoe around Goal
# 4: HorseShoe in Path
# 5: SensorNoise

    testType = 5

# fig = plt.figure()
# ax = fig.add_subplot(111, xlim = (-7.5, 7.5), ylim = (-5, 10), zlim = (0, 10), projection = '3d')

# ax.set_xlabel('X Position (m)'), ax.set_ylabel('Y Position (m)'), ax.set_zlabel('Z Position (m)')

    testType, plannerCalcTime, Deviation, Curvature, lifeTime, pathTime, pathLength, inputDeviation, Collided = main(testType)


