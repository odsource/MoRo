from math import *
from Robot_Simulator_V2 import obstacleWorld3
from Robot_Simulator_V2 import Robot


# Roboter in obstacleWorld3 positionieren:
myWorld = obstacleWorld3.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, [1, 6, 0])

# Polygonzug, der abgefahren werden soll, einzeichnen:
polyline = [[1,6],[9.5,6],[10.5,3]]
myWorld.drawPolyline(polyline)


# CursorController definieren:
myCursorController = myWorld.getCursorController()


# Bewege Roboter mit Cursor-Tasten:
while True:
    motion = myCursorController.getSpeed()
    if motion == None:
        break
    print("v = ", motion[0], "omega = ", motion[1]*180/pi)
    myRobot.move(motion)
    distAngles = myRobot.senseBoxes()
    if distAngles is not None:
        print(distAngles)

# Simulation schliessen:
myWorld.close(False)