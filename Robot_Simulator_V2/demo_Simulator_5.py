from math import *
from Robot_Simulator_V2 import officeWorld
from Robot_Simulator_V2 import Robot
from Robot_Simulator_V2 import SensorUtilities


# Roboter in Office-World positionieren:
myWorld = officeWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, [2, 5.5, pi/2])

# CursorController definieren:
myCursorController = myWorld.getCursorController()


# Bewege Roboter mit Cursor-Tasten:
while True:
    motion = myCursorController.getSpeed()
    if motion == None:
        break
    myRobot.move(motion)
    dists = myRobot.sense()
    directions = myRobot.getSensorDirections()
    lines_l = SensorUtilities.extractLinesFromSensorData(dists, directions)
    lines_g = SensorUtilities.transformPolylinesL2G(lines_l, myWorld.getTrueRobotPose())
    myWorld.drawPolylines(lines_g)

# Simulation schliessen:
myWorld.close(False)


