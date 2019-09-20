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


class GAOptimiser():
    def __init__(self, populationSize):

        self.f = lambda x, weights: self.runSim(x, weights)
        self.fitnesses = []
        self.populationSize = populationSize
        self.numberOfParents = 40

        self.L = 3

        self.mutationProbability = 0.1

        self.NNs = [Net() for i in range(self.populationSize)]
        self.getSizes()
        self.population = self.createPopulation(populationSize)
        self.setNNparams()
        self.bestFitness = -np.inf
        self.bestPosition = np.zeros((self.solutionLength,))
    
    
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

    def runSim(self, net, weights, runType = pbl.DIRECT ):
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

        dir = np.random.normal(0, 1, size=(3, ))
        dir /= np.linalg.norm(dir)

        goal = np.array([np.random.uniform(-4, 4), np.random.uniform(0, 8), np.random.uniform(0, 4)])

        start = goal + dir * self.L
            
        ObstacleIDs, ObstacleLocations = self.generateObstacles(5, start, ObstacleIDs, ObstacleLocations, ObstacleLocations)

        Qimmiq = Drone(ObstacleIDs, net=net, init_pos=start, goal = goal)

        i = 0
        Collided = False
        
        while (not Qimmiq.targetReached) and not Collided and Qimmiq.time < 20:
            
            for o in range(len(Qimmiq.obstacles[:-1])):
                if self.hasCollided(Qimmiq, Qimmiq.obstacles[o]):
                    Collided = True
                    print('Collision!')
                    break

            [pbl.resetBasePositionAndOrientation(ObstacleIDs[i], ObstacleLocations[i], ObstacleOrientations[i]) for i in range(len(ObstacleOrientations))]
            if i % 240 == 0:
                velocities = [np.random.normal(size=(3, )) for k in range(len(ObstacleIDs[:-1]))]

            [pbl.resetBaseVelocity(ObstacleIDs[j], velocities[j], [0, 0, 0]) for j in range(len(ObstacleIDs[len(ObstacleOrientations):-1]))]
            
            i += 1
            
            Qimmiq.moveDrone(Qimmiq.time_interval, i)
            pbl.stepSimulation()
        pbl.disconnect()

        print("lifeTime: {}".format(Qimmiq.time))
        print("Intrusion: {}".format(Qimmiq.Intrusion))
        print("ITSE: {}".format(10 * Qimmiq.e))
        # return Qimmiq.time

        print("Fitness: {0}".format(-1 * (weights[0] * Qimmiq.e + weights[1] * Qimmiq.Intrusion)))

        return -1 * (weights[0] * Qimmiq.e + weights[1] * Qimmiq.Intrusion)

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

    def findFittest(self, fitness):
        fitness = np.array(fitness)
        population = self.population[:, np.where(np.isnan(fitness) == False)].squeeze()
        fitness = fitness[np.where(np.isnan(fitness) == False)]
        
        numberOfParents = len(fitness) if len(fitness) < self.numberOfParents else self.numberOfParents

        return (fitness[np.argmax(fitness)], population[:, np.argmax(fitness)],
                np.array([population[:, i] for i in np.argsort(fitness)[::-1][0:numberOfParents]]), numberOfParents)

    def marry(self, Parents, numberOfParents):
        children = np.zeros((1, self.solutionLength))
        for p in range(0, numberOfParents, 2):
            for i in range(int(self.populationSize / (numberOfParents / 2))):
                crossoverSite = int(np.random.normal(self.solutionLength / 2, self.solutionLength / 8))
                if crossoverSite < 0: crossoverSite = 0
                if crossoverSite > self.solutionLength: crossoverSite = self.solutionLength
                child = np.hstack((Parents[p, 0:crossoverSite], Parents[p + 1, crossoverSite:])).reshape(
                    (1, self.solutionLength))
                children = np.vstack((children, child))
        
        if children.shape[0] < self.populationSize:
            children = np.vstack((children, np.random.normal(0, 1, size = (self.populationSize - children.shape[0], self.solutionLength))))

        return children[1:]

    def mutate(self, member):
        for i in range(len(member)):
            for j in range(self.solutionLength):
                if np.random.rand() < self.mutationProbability:
                    member[i, j] = member[i, j] + 0.25 * np.random.normal(0, 1)

        return member.T

    def update(self):
        weights = np.random.normal(0, 1, size = (self.populationSize, 2))
        weights /= np.linalg.norm(weights)
        fitness = []
        [(print("Training member: {0}".format(x+1)), fitness.append(self.f(self.NNs[x], weights[x]))) for x in range(self.populationSize)]
        currentBest, topMember, fittestParents, numberOfParents = self.findFittest(fitness)
        if currentBest > self.bestFitness:
            self.bestFitness = currentBest
            self.bestPosition = topMember
        newGeneration = self.marry(fittestParents, numberOfParents)
        self.population = deepcopy(self.mutate(newGeneration))
        self.setNNparams()
        return

    def train(self, nIters):
        for cIter in range(nIters):
            
            self.update()
            print("At iteration {0}, best fitness: {1}".format(cIter, self.bestFitness))
            self.fitnesses.append(self.bestFitness)

            Result = {
                        'Fitness': self.bestFitness,
                        'Parameters': self.bestPosition
                        
            }
            with open('GNNResults.txt', 'a') as file:
                file.write(str(Result) + '\n')
            
        print('Training Complete!')
        return self.bestPosition

if __name__ == "__main__":
    ga = GAOptimiser(200)
    sol = ga.train(50)
