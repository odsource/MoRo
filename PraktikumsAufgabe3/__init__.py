import random

import numpy as np
from Robot_Simulator_V2 import simpleWorld, Robot


class ParticleFilterPoseEstimator:
    particles = []
    def initialize(self, poseFrom, poseTo, n = 200):
        print("Initializing")
        i = 0
        while i < n:
            #generiere Zufallszahl x aus[poseFrom, poseTo];
            x = random.uniform(poseFrom[0], poseTo[0])
            #generiere Zufallszahl y aus[poseFrom, poseTo];
            y = random.uniform(poseFrom[1], poseTo[1])
            # generiere Zufallswinkel theta aus [poseFrom, poseTo]
            theta = random.uniform(poseFrom[2], poseTo[2])
            i += 1
            self.particles.append([x, y, theta])

    def integrateMovement(self, motion):
        print("")

    def integrateMeasurement(self, dist_list, alpha_list, distantMap):
        print("")

    def getPose(self):
        print("")

    def getCovariance(self):
        print("")


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

    n = 100

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


print("Aufgabe 1: \n")
myWorld = simpleWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, [4, 5.5, np.pi / 2])
curveDrive(myRobot, 100, 5, -180)
myGrid = myWorld.getDistanceGrid()
myGrid.drawGrid()

myWorld.close()
