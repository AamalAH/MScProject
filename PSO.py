import numpy as np
import pybullet as pbl
import pybullet_data
from Drone import Drone
import time


weights = [0., 0., 0., 1]

def run_sim(PID_Params):

    physicsClient = pbl.connect(pbl.DIRECT)
    pbl.setAdditionalSearchPath(pybullet_data.getDataPath())

    pbl.setGravity(0, 0, -9.8)

    Qimmiq = Drone([], PID_Params)

    i = 0

    while not Qimmiq.completedCourse:
        i += 1
        Qimmiq.moveDrone(1/240, i)
        pbl.stepSimulation()
        time.sleep(1/240)

    pbl.disconnect()

    return np.dot(weights, [abs(Qimmiq.SSE), abs(Qimmiq.OS), Qimmiq.TR-Qimmiq.TS, Qimmiq.e])

fitness = lambda x: run_sim(x)

class Solution():
    def __init__(self, position, fitness):
        self.position = position
        self.fitness = fitness

class Particle():
    def __init__(self, fitness, i):
        self.i = i

        self.position = np.array([[25 * np.pi/180, -25 * np.pi/180, .3686864, 25, 25, 25],[ 5* np.pi/180, -5 * np.pi/180, 2e10, 3, 3, .0], [20 * np.pi/180, -20 * np.pi/180, .84622276, 5, 5, 5]]) + 0.1 * np.random.normal(0, 1, size = (3, 6))
        self.velocity = np.zeros((3, 6))
        self.fitFun = fitness
        self.fitness = self.fitFun(self.position)
        print('Creating Particle {0} with position: {1}, fitness: {2:1.2f}'.format(i, self.position, self.fitness))
        self.bestSolution = Solution(self.position, self.fitness)

    def update(self, GlobalSolution, gen):
        self.velocity = 0.729 * self.velocity + 1.494 * np.random.rand() * (self.bestSolution.position - self.position) + 1.494 * np.random.rand() * (GlobalSolution.position - self.position)
        self.position = self.position + self.velocity
        self.fitness = self.fitFun(self.position)
        print('Updating Particle {0} for generation {1} with position: {2}, fitness: {3:1.2f}'.format(self.i, gen, self.position, self.fitness))
        if self.fitness < self.bestSolution.fitness:
            self.bestSolution = Solution(self.position, self.fitness)

Particles = [Particle(fitness, i) for i in range(40)]
bestSolution = Solution(np.zeros((3, 6)), np.inf)

if min([particle.fitness for particle in Particles]) < bestSolution.fitness:
    bestSolution = Solution(Particles[np.argmin([particle.fitness for particle in Particles])].position, min([particle.fitness for particle in Particles]))

for i in range(100):
    print('Current Best Solution: {0} has position {1}'.format(bestSolution.fitness, bestSolution.position))
    [particle.update(bestSolution, i) for particle in Particles]
    if min([particle.fitness for particle in Particles]) < bestSolution.fitness:
        bestSolution = Solution(Particles[np.argmin([particle.fitness for particle in Particles])].position, min([particle.fitness for particle in Particles]))