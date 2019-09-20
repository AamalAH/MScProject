# MScProject

The files contain all the code necessary to run the simulation. It requires:

PyTorch
PyBullet
Pandas (though this can just be commented out)

To run the code use any of the .py files labelled 'Test' (i.e. StaticTests.py, DynamicTests.py). If you wish to see the simulation and the drone etc make sure that the PyBullet connect type is GUI. This can be done by changing the following line (present in any test file)

pbl.connect(pbl.DIRECT)

to 

pbl.connect(pbl.GUI)

This repo will continue to be updated by adding in comments and cleaning up code (but no changes in logic or structure will follow).

Drone.py: Contains all code pertaining to the drone itself. Note that the drone object contains obstacles objects which are used in the planners
DVZ.py: Formulates the DVZ which will be used by the neural network in the GNN method
GNN.py: Contains the code structure for the neural network in PyTorch as well as the GA optimiser.
MOGA.py: Contains the code required for applying the VEGA and WSGA methods for multi objective training (I'll likely change this file so it's easier to switch between the training modes)
Obstacle.py: Contains all the information regarding an obstacle that the planner(s) need to know. This include its state, size and distance from the drone
PID.py: Contains the code required to formulate a PID controller. The gain parameters are chosen through tuning and are subject to the properties of the drone. The code contains both the controller used un the Multipoint method and in the GNN method.
PSO.py: Code for the Particle Swarm Optimisation used to tune the PID controller. 
runGNN.py: Code used to run tests on the GNN method using different optimised planners (contained in the GNNSolutions folder). Use this to actually use the GNN method on a drone.
MotionPlanners/MultiPointNav.py: Code used to define the Multipoint APF planner, including: potential determination and minimum point selection

