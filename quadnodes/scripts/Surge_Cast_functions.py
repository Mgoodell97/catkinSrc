# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, acos, sqrt, exp

def randomNumAtoB(a,b):
    return ((b - a) * np.random.random_sample() + a)

def rotateRobot_WorldToRobot(xRobotCurrentWorld, yRobotCurrentWorld, thetaFunc, xRobotOffsetFunc, yRobotOffsetFunc):
    Xw = np.array([xRobotCurrentWorld, yRobotCurrentWorld, 1])
    # print(Xw)
    R = np.array([[cos(thetaFunc), -sin(thetaFunc)], [sin(thetaFunc), cos(thetaFunc)]])
    P = np.array([[xRobotOffsetFunc, yRobotOffsetFunc]]).T;

    Rinv = np.linalg.inv(R)
    Pinv = np.array([np.matmul(-Rinv, np.squeeze(P))]).T

    bottomArray = np.array([[0, 0 , 1]]);

    TplumeFrame_w = np.concatenate((Rinv, Pinv), axis=1)
    TplumeFrame_w = np.concatenate((TplumeFrame_w, bottomArray), axis=0)
    # print(TplumeFrame_w)
    Xrobot = np.matmul(TplumeFrame_w, Xw)
    # print(Xrobot)
    xRobotRotated = Xrobot[0]
    yRobotRotated = Xrobot[1]

    return xRobotRotated, yRobotRotated

def rotateRobot_RobotToWorld(xRobotCurrentRobot, yRobotCurrentRobot, thetaFunc, xRobotOffsetFunc, yRobotOffsetFunc):
    Xrobot = np.array([xRobotCurrentRobot, yRobotCurrentRobot, 1])
    # print(Xrobot)
    R = np.array([[cos(thetaFunc), -sin(thetaFunc)], [sin(thetaFunc), cos(thetaFunc)]])
    P = np.array([[xRobotOffsetFunc, yRobotOffsetFunc]]).T;

    bottomArray = np.array([[0, 0 , 1]]);
    Trobot_w = np.concatenate((R, P), axis=1)

    # Transfer matrix from robot frame to world frame
    Trobot_w = np.concatenate((Trobot_w, bottomArray), axis=0)
    # print(Trobot_w)
    Xw = np.matmul(Trobot_w, Xrobot)
    # print(Xw)
    xRobotRotated = Xw[0]
    yRobotRotated = Xw[1]

    return xRobotRotated, yRobotRotated


def gaussFunc(xFunc, yFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    con = (QFunc/(4 * pi * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * ((yFunc**2)/DyFunc+ (zFunc**2)/DzFunc))
    return con


def getReading(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    # Rotate frame

    Xw_r = np.array([xRobotDef, yRobotDef, 1])

    R = np.array([[cos(thetaFunc), -sin(thetaFunc)], [sin(thetaFunc), cos(thetaFunc)]])
    P = np.array([[xPlumeFunc, yPlumeFunc]]).T;

    bottomArray = np.array([[0, 0 , 1]]);
    Tw_plumeFrame = np.concatenate((R, P), axis=1)

    # Transfer matrix from plumeframe to w
    Tw_plumeFrame = np.concatenate((Tw_plumeFrame, bottomArray), axis=0)

    Rinv = np.linalg.inv(R)
    Pinv = np.array([np.matmul(-Rinv, np.squeeze(P))]).T

    TplumeFrame_w = np.concatenate((Rinv, Pinv), axis=1)

    # Transfer matrix from w to plumeframe
    TplumeFrame_w = np.concatenate((TplumeFrame_w, bottomArray), axis=0)

    XplumeFrame = np.matmul(TplumeFrame_w, Xw_r)

    xRobotRotated = XplumeFrame[0]
    yRobotRotated = XplumeFrame[1]


    if xRobotRotated <= 0:
        reading = 0
    else:
        reading = gaussFunc(xRobotRotated,yRobotRotated,zFunc,QFunc,vFunc,DyFunc,DzFunc)

    return reading

class SurgeCastSim:
    def __init__(self, simStepsClass, xRobotSpawnClass, yRobotSpawnClass, stepSizeClass, thresholdClass, thetaPlumeClass, xPlumeClass, yPlumeClass, QPlume, vPlume, DyPlume, DzPlume, minLimClass = -25, maxLimClass = 25):
        self.simSteps = simStepsClass
        self.xRobotSpawn = xRobotSpawnClass
        self.yRobotSpawn = yRobotSpawnClass
        self.stepSize = stepSizeClass
        self.threshold = thresholdClass
        self.thetaPlume = thetaPlumeClass * pi/180
        self.xPlume = xPlumeClass
        self.yPlume = yPlumeClass
        self.QPlume = QPlume
        self.vPlume = vPlume
        self.DyPlume = DyPlume
        self.DzPlume = DzPlume
        self.minLim = minLimClass
        self.maxLim = maxLimClass

        self.xMovesList = []
        self.yMovesList = []
        self.readingsList = []
        self.errorList = []


        # surge casting params
        xRobot, yRobot = rotateRobot_WorldToRobot(xRobotSpawnClass, yRobotSpawnClass, self.thetaPlume + pi/2, xRobotSpawnClass, yRobotSpawnClass)
        robotThetaCurrent = 0

        #
        xRobotSurgeOffset = xRobot
        yRobotSurgeOffset = yRobot
        r1 = self.stepSize

        # For tf frames
        xRobotTf = xRobotSpawnClass
        yRobotTf = yRobotSpawnClass
        thetaRotation = self.thetaPlume + pi/2

        # Calculate error
        xError = abs(self.xPlume - xRobot)
        yError = abs(self.yPlume - yRobot)

        # Calculate errorMagnitude
        errorMag = np.linalg.norm(np.array([xError, yError]))

        # Record robots movement
        self.errorList.append(errorMag)

        area = pi * pow((0.015/2),2)

        # Start bias random walk
        for i in range(self.simSteps):

            # plt.clf()

            # rotate to world frame
            xRobotRotatedWorld, yRobotRotatedWorld = rotateRobot_RobotToWorld(xRobot, yRobot, thetaRotation, xRobotTf, yRobotTf)
            # print()
            # print(xRobot, yRobot)
            # print(xRobotRotatedWorld, yRobotRotatedWorld)
            # print(xRobotTf,yRobotTf)
            # print(thetaRotation)
            # print()
            # print("---------------------")
            # get plume reading
            currentReading = getReading(xRobotRotatedWorld, yRobotRotatedWorld, self.thetaPlume, self.xPlume, self.yPlume, 0.0, self.QPlume, self.vPlume, self.DyPlume, self.DzPlume)

            currentReading = (1/self.vPlume) * (1/area) * 1000 * currentReading
            # print(currentReading)
            if currentReading < self.threshold: # below threshold do spinarony
                r1Squared = pow(r1,2)
                stepSizeSquared = pow(self.stepSize,2)
                r2 = sqrt(r1Squared + stepSizeSquared)
                r2Squared = pow(r2,2)

                robotThetaCurrentNew = acos( (r1Squared + r2Squared - stepSizeSquared)/(2*r1*r2) )
                robotThetaCurrent += robotThetaCurrentNew

                xRobot = r2 * cos(robotThetaCurrent) + xRobotSurgeOffset
                yRobot = r2 * sin(robotThetaCurrent) + yRobotSurgeOffset

                r1 = r2

            else: # above threshold go towards plume
                # print("Reset")
                robotThetaCurrent = 0
                xRobotSurgeOffset = xRobot
                yRobotSurgeOffset = yRobot
                r1 = self.stepSize

                yRobot += self.stepSize

            xRobotRotatedWorld, yRobotRotatedWorld = rotateRobot_RobotToWorld(xRobot, yRobot, thetaRotation, xRobotTf, yRobotTf)
            xRobot, yRobot = rotateRobot_WorldToRobot(xRobotRotatedWorld, yRobotRotatedWorld, self.thetaPlume + pi/2, xRobotSpawnClass, yRobotSpawnClass)

            # Calculate error
            xError = abs(self.xPlume - xRobotRotatedWorld)
            yError = abs(self.yPlume - yRobotRotatedWorld)

            # Calculate errorMagnitude
            errorMag = np.linalg.norm(np.array([xError, yError]))

            # Record robots movement
            self.xMovesList.append(xRobotRotatedWorld) # record movements
            self.yMovesList.append(yRobotRotatedWorld)
            self.readingsList.append(currentReading)
            self.errorList.append(errorMag)


    def plotSim(self, frameTime, FixedSize = False):

        # Plotting
        if FixedSize:
            fig = plt.figure()
            fig.set_size_inches(12, 12)
        xPlt = []
        yPlt = []

        xPlumePlot = np.arange(self.minLim, self.maxLim+1, 0.5)
        yPlumePlot = np.arange(self.minLim, self.maxLim+1, 0.5)

        conArray = np.zeros([len(xPlumePlot), len(yPlumePlot)])

        for xCurrentIndex in range(len(xPlumePlot)):
            for yCurrentIndex in range(len(yPlumePlot)):
                conArray[xCurrentIndex,yCurrentIndex] = getReading(yPlumePlot[yCurrentIndex], xPlumePlot[xCurrentIndex], self.thetaPlume, self.xPlume, self.yPlume, 0.0, self.QPlume, self.vPlume, self.DyPlume, self.DzPlume)

        flat=conArray.flatten()
        flat.sort()
        secondHighestValue = flat[-2]
        ind = np.unravel_index(np.argmax(conArray, axis=None), conArray.shape)
        conArray[ind[0]][ind[1]] = secondHighestValue * 1.03

        # Start plotting
        for i in range(self.simSteps):

            # Plotting stuff
            plt.contourf(xPlumePlot,yPlumePlot,conArray,25)
            plt.xlim(self.minLim, self.maxLim)
            plt.ylim(self.minLim, self.maxLim)

            xRobot = self.xMovesList[i]
            yRobot = self.yMovesList[i]

            currentReading = getReading(xRobot, yRobot, self.thetaPlume, self.xPlume, self.yPlume, 0.0, self.QPlume, self.vPlume, self.DyPlume, self.DzPlume)

            xPlt.append(xRobot)
            yPlt.append(yRobot)


            plt.plot(xPlt,yPlt,'r')
            plt.plot(xRobot,yRobot,'go', markersize=6)

            # plt.plot(bestLocation[0],bestLocation[1],'yo', markersize=10)
            plt.show()
            plt.pause(frameTime)
            if i != (self.simSteps-1):
                plt.clf()

    def plotError(self):
        fig = plt.figure()
        plt.plot(list(range(len(self.errorList))),self.errorList)
        plt.grid()
        plt.xlim(0, max(list(range(len(self.errorList))))*1.05)
        plt.ylim(0, max(self.errorList)*1.05)
        plt.xlabel("Number of steps")
        plt.ylabel("Magnitude of location error [m]")


class MultiSurgeCastSims:
    def __init__(self, NumberOfSurgeCasts, simStepsClass, xRobotSpawnClass, yRobotSpawnClass, stepSizeClass, thresholdClass, thetaPlumeClass, xPlumeClass, yPlumeClass, QPlume, vPlume, DyPlume, DzPlume, minLimClass = -25, maxLimClass = 25, randomSpawn = False):
        self.SurgeCastSimxMovesList = []
        self.SurgeCastSimyMovesList = []
        self.SurgeCastSimReadingsList = []
        self.SurgeCastErrorOverTime = []
        self.SurgeCastAverageErrorOverTime = []
        self.SurgeCastSTDErrorOverTime = []

        for i in range(NumberOfSurgeCasts):

            if randomSpawn:
                xRobotSpawnClass = randomNumAtoB(minLimClass, maxLimClass)
                yRobotSpawnClass = randomNumAtoB(minLimClass, maxLimClass)

            SurgeCast = SurgeCastSim(simStepsClass, xRobotSpawnClass, yRobotSpawnClass, stepSizeClass, thresholdClass, thetaPlumeClass, xPlumeClass, yPlumeClass, QPlume, vPlume, DyPlume, DzPlume, minLimClass, maxLimClass)

            # Get robots movements and readings from sim
            self.SurgeCastSimxMovesList.append(SurgeCast.xMovesList)
            self.SurgeCastSimyMovesList.append(SurgeCast.yMovesList)
            self.SurgeCastSimReadingsList.append(SurgeCast.readingsList)
            self.SurgeCastErrorOverTime.append(SurgeCast.errorList)

        self.SurgeCastErrorOverTime = np.array(self.SurgeCastErrorOverTime)


        for i in range(simStepsClass):
            self.SurgeCastAverageErrorOverTime.append(np.mean(self.SurgeCastErrorOverTime[:,i]))
            self.SurgeCastSTDErrorOverTime.append(np.std(self.SurgeCastErrorOverTime[:,i]))

    def plotError(self,plotSTD = True):
        if plotSTD:
            upperSigma = np.array(self.SurgeCastAverageErrorOverTime) + 2*np.array(self.SurgeCastSTDErrorOverTime)
            lowerSigma = np.array(self.SurgeCastAverageErrorOverTime) - 2*np.array(self.SurgeCastSTDErrorOverTime)

        # fig = plt.figure()
        plt.plot(list(range(len(self.SurgeCastAverageErrorOverTime))),self.SurgeCastAverageErrorOverTime)
        if plotSTD:
            plt.fill_between(list(range(len(self.SurgeCastSTDErrorOverTime))), lowerSigma, upperSigma, alpha=0.2)
            plt.legend(('Average', 'std'))

        # plt.plot(list(range(len(self.BRWSTDErrorOverTime))), upperSigma)
        # plt.plot(list(range(len(self.BRWSTDErrorOverTime))), lowerSigma)

        plt.grid()
        plt.xlim(0, max(list(range(len(self.SurgeCastAverageErrorOverTime)))))
        if plotSTD:
            plt.ylim(-10, max(upperSigma)*1.05)
        else:
            plt.ylim(0, max(self.SurgeCastAverageErrorOverTime)*1.05)
        plt.xlabel("Number of steps")
        plt.ylabel("Magnitude of location error [m]")
