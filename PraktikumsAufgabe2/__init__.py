from Robot_Simulator_V2 import emptyWorld, Robot


def curveDrive(robot, v, r, Dq):
    print("CurveDrive: \n")


def straightDrive(robot, v, l):
    print("StraightDrive: \n")
    robot.move((v, l))


print("Aufgabe 1: \n")
myRobot = Robot.Robot()
myWorld = emptyWorld.buildWorld()
myRobot.setWorld(myWorld)
dist = myRobot.sense()
