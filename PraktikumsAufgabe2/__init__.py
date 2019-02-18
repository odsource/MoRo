import numpy as np
from Robot_Simulator_V2 import emptyWorld, Robot, World


def curveDrive(robot, v, r, deltaTheta):
    print("CurveDrive: \n")
    omega = v / r  # 3-11 im Skript
    omega = omega * np.sign(deltaTheta)
    deltaTheta = np.deg2rad(deltaTheta)
    tau = deltaTheta / omega  # 3-26 im Skript
    robot.setTimeStep(abs(tau))

    #############################################
    # Kreisfahrt aus demo_Simulator_1.py
    #############################################

    n = 10

    motionCircle = []

    for i in range(n):
        motionCircle.append([v / n, omega / n])

    # Bewege Roboter
    for t in range(n):
        # Bewege Roboter
        motion = motionCircle[t]
        print("v = ", motion[0], "omega = ", motion[1] * 180 / np.pi)
        robot.move(motion)

    #############################################
    # Ende der Kreisfahrt aus demo_Simulator_1.py
    #############################################


def straightDrive(robot, v, l):
    print("StraightDrive: \n")

    robot.setTimeStep(l / v)

    time = v / l

    n = 1000

    motionDrive = []

    for i in range(n):
        motionDrive.append([v / n, 0])

    # Bewege Roboter
    for t in range(n):
        # Bewege Roboter
        motion = motionDrive[t]
        print("v = ", v, "Länge = ", l)
        robot.move(motion)


def kreisFahrt(myRobot):
    print("Kreisfahrt: \n")
    curveDrive(myRobot, 1, 1, -360)


def rechteckFahrt(myRobot):
    print("Rechteckfahrt: \n")
    straightDrive(myRobot, 1, 1)
    curveDrive(myRobot, 1, 1, -90)
    straightDrive(myRobot, 1, 1)
    curveDrive(myRobot, 1, 1, -90)
    straightDrive(myRobot, 1, 1)
    curveDrive(myRobot, 1, 1, -90)
    straightDrive(myRobot, 1, 1)


def spurWechsel(myRobot):
    straightDrive(myRobot, 1, 1)
    curveDrive(myRobot, 1, 1, -45)
    straightDrive(myRobot, 1, 1)
    curveDrive(myRobot, 1, 1, 45)
    straightDrive(myRobot, 1, 1)


#def followLine(myRobot, myWorld, p1, p2):
#    (x1, y1) = p1
#    (x2, y2) = p2
#    l = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
#    kp = -0.5

#    nv = np.array([-(x2 - x1), y2 - y1])
#    (x, y, thetaR) = myWorld.getTrueRobotPose()
#    rv = np.array([x, y])
#    d = nv.dot(rv)
#    theta = d * kp
#    dp = 1000000

#    while(True):
#        curveDrive(myRobot, 1, 2, theta)
#        nv = np.array([-(x2 - x1), y2 - y1])
#        (x, y, thetaR) = myWorld.getTrueRobotPose()
#        rv = np.array([x, y])
#        d = nv.dot(rv)
#        theta = d * kp

print("Aufgabe 1: \n")
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, [2, 5.5, np.pi / 2])

#kreisFahrt(myRobot)
#rechteckFahrt(myRobot)
#spurWechsel(myRobot)
#followLine(myRobot, myWorld, [0, 0], [20, 20])

myWorld.close()

