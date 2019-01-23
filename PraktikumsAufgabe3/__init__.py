import random

import numpy as np

from PoseEstimator.PlotUtilities import plotPoseParticles, plotShow
from Robot_Simulator_V2 import simpleWorld, Robot, SensorUtilities


class ParticleFilterPoseEstimator:
    particles = []
    _maxOmega = np.pi
    _maxSpeed = 1.0
    _T = 0.1
    _SigmaMotion = np.zeros((2, 2))
    _k_d = 0.05 * 0.05  # velocity noise parameter = 0.05m*0.05m / 1m
    _k_theta = (5.0 * 5.0 / 360.0) * (np.pi / 180.0)  # turning rate noise parameter = 5deg*5deg/360deg * (1rad/1deg)
    _k_drift = (2.0 * 2.0) / 1.0 * (np.pi / 180.0) ** 2  # drift noise parameter = 2deg*2deg / 1m

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
        i = 0
        for particle in self.particles:
            v = motion[0]
            omega = motion[1]

            # translational and rotational speed is limited:
            if omega > self._maxOmega:
                omega = self._maxOmega
            if omega < -self._maxOmega:
                omega = -self._maxOmega
            if v > self._maxSpeed:
                v = self._maxSpeed
            if v < -self._maxSpeed:
                v = -self._maxSpeed

            # Add noise to v:
            sigma_v_2 = (self._k_d / self._T) * abs(v)
            v_noisy = v + random.gauss(0.0, np.sqrt(sigma_v_2))

            # Add noise to omega:
            sigma_omega_tr_2 = (self._k_theta / self._T) * abs(omega)  # turning rate noise
            sigma_omega_drift_2 = (self._k_drift / self._T) * abs(v)  # drift noise
            omega_noisy = omega + random.gauss(0.0, np.sqrt(sigma_omega_tr_2))
            omega_noisy += random.gauss(0.0, np.sqrt(sigma_omega_drift_2))

            # Set SigmaMotion:
            self._SigmaMotion[0, 0] = sigma_v_2
            self._SigmaMotion[1, 1] = sigma_omega_tr_2 + sigma_omega_drift_2

            # Move particles in the world (with noise):
            d_noisy = v_noisy * self._T
            dTheta_noisy = omega_noisy * self._T

            self.move(self, i, d_noisy, dTheta_noisy)
            i += 1

    def move(self, i, d, dTheta):
        # Skript 5-23 steht die Formel
        x = self.particles[i][0]
        y = self.particles[i][1]
        theta = self.particles[i][2]
        dx = d * np.cos(theta)
        dy = d * np.sin(theta)
        self.particles[i][0] += dx
        self.particles[i][1] += dy
        self.particles[i][2] = (theta + dTheta)%(2*np.pi)

    def integrateMeasurement(self, dist_list, alpha_list, distantMap):
        # Likelihood-Algorithmus
        weighedParticle = []
        # Obstacle-Koordinaten aus Sicht des KS des Partikels
        for particle in self.particles:
            p = 1
            globalCoObPaDi = []
            for i in range(len(dist_list)):
                d = dist_list[i]
                if d == None:
                    continue
                alpha = alpha_list[i]
                x_local = d * np.cos(alpha)
                y_local = d * np.sin(alpha)
                x_global = particle[0] + x_local * np.cos(particle[2]) - y_local * np.sin(particle[2])
                y_global = particle[1] + x_local * np.sin(particle[2]) + y_local * np.cos(particle[2])
                globalCoObPaDi.append([x_global, y_global])
        # Obstacle-Koordinatenberechnung fertig

        # Gewichtung der Partikel
            for obstacleCoord in globalCoObPaDi:
                xDist = obstacleCoord[0] - distantMap.getValue(obstacleCoord[0])
                yDist = obstacleCoord[1] - distantMap.getValue(obstacleCoord[1])
                p = p * self._ndf(0, [xDist, yDist], 0.5 ** 2)
            weighedParticle.append(particle, p)

        # Resampling Folie 5-54
        # for i = 1 to M do
        # ziehe i zufällig mit Wahrscheinlichkeit wi;
        # ck + 1 = ck + 1 schnitt {xk + 1[i]};
        # endfor



    # 1-dimensional normal distribution N(mu, sigma2)
    def _ndf(self, x, mu, sigma2):
        c = 1/(np.sqrt(2*sigma2*np.pi))
        return c*np.exp(-0.5 * (x-mu)**2 / sigma2)

    def getPose(self):
        return np.mean(self.Particles, axis=0)

    def getCovariance(self):
        return np.cov(self.Particles[:, :3].T)


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

poseEstimator = ParticleFilterPoseEstimator
poseFrom = [3, 4.5, np.pi * (1 / 4)]
poseTo = [5, 6.5, np.pi * (3 / 4)]
poseEstimator.initialize(poseEstimator, poseFrom, poseTo)
plotPoseParticles(poseEstimator.particles, color='g')
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
plotPoseParticles(poseEstimator.particles, color='y')
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
plotPoseParticles(poseEstimator.particles, color='r')
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])
poseEstimator.integrateMovement(poseEstimator, [1, -np.pi / 2])

plotPoseParticles(poseEstimator.particles)
plotShow()

#curveDrive(myRobot, 100, 5, -180)
#myGrid = myWorld.getDistanceGrid()
#myGrid.drawGrid()

myWorld.close()
