from Robot_Simulator_V2 import emptyWorld
from Robot_Simulator_V2 import Robot

import numpy as np

def followLine(p1, p2):
    print("Linienverfolgung P-Regler: \n")


def calcLineLength(x, y):
    print("Linienlänge wird berechnet: \n")
    l = np.sqrt(x**2 + y**2)
    return l


myWorld = emptyWorld.buildWorld()
robot = Robot.Robot
robot.setWorld(myWorld)
