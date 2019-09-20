import time
from copy import deepcopy

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pybullet as pbl
import pybullet_data
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.utils.data as data

from Drone import Drone

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.convDistance = nn.Conv2d(4, 16, 5, stride=5, bias=True)
        self.pool = nn.MaxPool2d(2, 2)
        self.hidden2Output = nn.Linear(16, 3)


    def forward(self, x):
        x = x.view(4, 16, 16).unsqueeze(0)
        x = torch.sigmoid(self.convDistance(x))
        x = self.pool(x)
        x = x.view(16)
        x = torch.tanh(self.hidden2Output(x))
        return x


class VEGAOptimiser():
    def __init__(self, populationSize):

        self.f = lambda x, function: self.runSim(x, function)
        self.fitnessesGoal = []
        self.fitnessesObs = []

        self.populationSize = populationSize
        
        self.L = 1
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)

        self.mutationProbability = 0.1
        self.crossoverProbability = 0.2

        self.NNs = [Net() for i in range(self.populationSize)]
        self.getSizes()
        self.population = self.createPopulation(populationSize)
        self.setNNparams()
        self.bestFitnesses = [-np.inf, -np.inf]
        self.bestPositions = [np.zeros((self.solutionLength,)), np.zeros((self.solutionLength,))]
    
    
    def generateObstacles(self, no_obs, start, ObstacleIDs, ObstacleLocations, wallLocations):
        for obs in range(no_obs):
            loc = np.array([np.random.uniform(-4, 4), np.random.uniform(0, 7), np.random.uniform(0, 4)])
            while np.any(np.linalg.norm(np.subtract(loc, np.array(wallLocations)), axis = 1) <= 1) or np.linalg.norm(loc - start)  <= 1:
                loc = np.array([np.random.uniform(-4, 4), np.random.uniform(0, 7), np.random.uniform(0, 4)])
            ObstacleIDs.append(pbl.loadURDF('block.urdf', loc))
            ObstacleLocations.append(tuple(loc))

        ObstacleIDs.append(pbl.loadURDF('plane.urdf', [0, 0, 0]))
        
        return ObstacleIDs, ObstacleLocations

    def hasCollided(self, Drone, obstacle):
        if obstacle.getDistance(Drone) <= 0.03 or (Drone.pos[2] < 0.1 and np.linalg.norm(Drone.xdot) <= 0.05):
            return True
        else:
            return False

    def runSim(self, net, function, runType = pbl.DIRECT):
        physicsClient = pbl.connect(runType)
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

        start = np.array([np.random.uniform(-4, 4), np.random.uniform(0, 7), np.random.uniform(0, 4)])
        while np.any(np.linalg.norm(np.subtract(start, np.array(ObstacleLocations)), axis = 1) <= 1):
            start = np.array([np.random.uniform(-4, 4), np.random.uniform(0, 7), np.random.uniform(0, 4)])

        if function == 0:
            ObstacleIDs, ObstacleLocations = self.generateObstacles(5, start, ObstacleIDs, ObstacleLocations, ObstacleLocations)

        Qimmiq = Drone(ObstacleIDs, net=net, init_pos=start)

        i = 0
        Collided = False
        
        while (not Qimmiq.targetReached) and ((function == 0 and not Collided) or (function == 1 and Qimmiq.time < 20)):
            
            if function == 0:

                for o in range(len(Qimmiq.obstacles[:-1])):
                    if self.hasCollided(Qimmiq, Qimmiq.obstacles[o]):
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

        Qimmiq.networkOutputs = np.array(Qimmiq.networkOutputs)

        if function == 0:
            print("Assessing Intrusion: {}".format(-1 * Qimmiq.Intrusion))
            return ( -1 * Qimmiq.Intrusion )
        elif function == 1:
            print("Assessing ITSE: {}".format(-1 * Qimmiq.e))
            return ( -1 * Qimmiq.e )
        else:
            assert False, "Invalid Function Type"

    def getSizes(self):
        self.parameterSizes = [f.size() for f in self.NNs[0].parameters()]
        self.parameterLengths = [np.product(i) for i in self.parameterSizes]
        self.solutionLength = int(np.sum(self.parameterLengths[::2]) + len(self.parameterLengths)/2)

    def setNNparams(self):
        for i, net in enumerate(self.NNs):
            index = 0
            params = []
            for l in range(0, len(self.parameterLengths), 2):
                params.append(self.population[index:index + self.parameterLengths[l], i])
                index += self.parameterLengths[l]
                params.append(np.ones(self.parameterLengths[l+1]) * self.population[index, i])
                index += 1
            for j, f in enumerate(net.parameters()):
                f.data = torch.nn.Parameter(torch.tensor(params[j].reshape((f.shape))))

    def createPopulation(self, populationSize):
        population = np.zeros((self.solutionLength, ))
        for pop in range(populationSize):
            params = []
            net =[f.detach().numpy() for f in self.NNs[pop].parameters()]
            for i in range(0, len(net), 2):
                [params.append(a) for a in net[i].flatten()]
                params.append(net[i + 1][0])
            population = np.vstack((population, np.array(params)))
        return population[1:].T

    def findFittest(self, pop, fitness):
        fitness = np.array(fitness)
        population = pop[:, np.where(np.isnan(fitness) == False)].squeeze()
        populationSize = population.shape[1]
        numberOfParents = int(np.ceil(self.crossoverProbability * populationSize))

        fitness = fitness[np.where(np.isnan(fitness) == False)]
        
        numberOfParents = len(fitness) if len(fitness) < numberOfParents else numberOfParents

        return (fitness[np.argmax(fitness)], population[:, np.argmax(fitness)],
                np.array([population[:, i] for i in np.argsort(fitness)[::-1][0:numberOfParents]]), numberOfParents)

    def marry(self, Parents, numberOfParents, populationSize):
        children = np.zeros((1, self.solutionLength))
        if numberOfParents%2 == 0:
            allParents = range(0, numberOfParents, 2)
        else:
            allParents = range(0, numberOfParents-1, 2)

        for p in allParents:
            for i in range(int(populationSize/len(allParents))):
                crossoverSite = int(np.random.normal(self.solutionLength / 2, self.solutionLength / 8))
                if crossoverSite < 0: crossoverSite = 0
                if crossoverSite > self.solutionLength: crossoverSite = self.solutionLength
                child = np.hstack((Parents[p, 0:crossoverSite], Parents[p + 1, crossoverSite:])).reshape(
                    (1, self.solutionLength))
                children = np.vstack((children, child))
        
        if children[1:].shape[0] < populationSize:
            children = np.vstack((children, np.random.normal(0, 1, size = (populationSize - children[1:].shape[0], self.solutionLength))))

        return children[1:]

    def mutate(self, member):
        for i in range(len(member)):
            for j in range(self.solutionLength):
                if np.random.rand() < self.mutationProbability:
                    member[i, j] = member[i, j] + 0.25 * np.random.normal(0, 1)

        return member.T

    def getNewGeneration(self, population, fitness, function):
        populationSize = np.shape(population)[1]
        currentBest, topMember, fittestParents, numberOfParents = self.findFittest(population, fitness)
        if currentBest > self.bestFitnesses[function]:
            self.bestFitnesses[function] = currentBest
            self.bestPositions[function] = topMember
        return fittestParents, numberOfParents

    def update(self):
        functions = [0, 1]
        functions = np.random.choice(functions, size = (self.populationSize))
        fitness = []
        [(print("Training member: {0}".format(x+1)), fitness.append(self.f(self.NNs[x], functions[x]))) for x in range(self.populationSize)]
        
        populationITSE, fitnessITSE = self.population[:, np.where(functions == 1)].squeeze(), np.array(fitness)[np.where(functions == 1)]
        populationIntrusion, fitnessIntrusion = self.population[:, np.where(functions == 0)].squeeze(), np.array(fitness)[np.where(functions == 0)]
        
        fittestParentsITSE, numberOfParentsITSE = self.getNewGeneration(populationITSE, fitnessITSE, 1)
        fittestParentsIntrusion, numberOfParentsIntrusion = self.getNewGeneration(populationIntrusion, fitnessIntrusion, 0)

        fittestParents = np.vstack((fittestParentsITSE, fittestParentsIntrusion))
        np.random.shuffle(fittestParents)

        newGeneration = self.marry(fittestParents, numberOfParentsITSE + numberOfParentsIntrusion, self.populationSize)
        self.population = self.mutate(newGeneration)

        self.fitnessesGoal.append(self.bestFitnesses[1])
        self.fitnessesObs.append(self.bestFitnesses[0])

        return

    def train(self, nIters):
        for cIter in range(nIters):            
            self.update()
            print("At iteration {0}, best fitnesses: \n Goal: {1} \n Obstacle: {2}".format(cIter, self.bestFitnesses[1], self.bestFitnesses[0]))

            Result = {
                        'Fitness': self.bestFitnesses,
                        'Parameters': self.bestPositions
                        
            }
            with open('VEGAResults.txt', 'a') as file:
                file.write(str(Result) + '\n')
            
        print('Training Complete!')
        return self.bestPositions[0], self.bestPositions[1]

if __name__ == "__main__":
    populationSize = 200
    trainingIters = 50

    ga = VEGAOptimiser(populationSize)
    solObs, solGoal = ga.train(trainingIters)

    solObs = pd.DataFrame(solObs)
    solObs.to_csv('VEGASolObs.csv', index = None, sep = ' ', mode = 'a')

    solGoal = pd.DataFrame(solGoal)
    solGoal.to_csv('VEGASolGoal.csv', index = None, sep = ' ', mode = 'a')

    fig = plt.figure()
    ax1 = fig.add_subplot(121)
    ax1.set_xlabel('Iterations'), ax1.set_ylabel('Fitness'), ax1.set_title('Go-To-Goal Behaviour')


    ax2 = fig.add_subplot(122)
    ax2.set_xlabel('Iterations'), ax2.set_ylabel('Fitness'), ax2.set_title('Obstacle Avoidance Behaviour')

    ax1.plot(list(range(trainingIters)), ga.fitnessesGoal, 'r--')
    ax2.plot(list(range(trainingIters)), ga.fitnessesObs, 'm--')
    plt.show()