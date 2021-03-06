# -*- coding: utf-8 -*-
import random
import matplotlib.pyplot as plt
import time
import numpy as np
from math import pi, sqrt, exp, cos, sin

def wapointBoundsLimiter(x, minLength=-25, maxLength=25):
    if x < minLength:
        x = minLength
    if x > maxLength:
        x = maxLength
    return x


def randomNumAtoB(a,b):
    return ((b - a) * np.random.random_sample() + a)


def moveRobot(xOld, yOld, stepSize, minBound=-25, maxBound=25):

    direction = randomNumAtoB(-2*pi,2*pi)

    xStep = stepSize * np.cos(direction)
    yStep = stepSize * np.sin(direction)

    xOld += xStep
    yOld += yStep

    xNew = wapointBoundsLimiter(xOld,minBound,maxBound)
    yNew = wapointBoundsLimiter(yOld,minBound,maxBound)

    return xNew, yNew


def biasedRandomWalk(xRobotNewFunc, yRobotNewFunc, previousReading, currentReading, biasRange, previousBias, stepSize, minBound=-25, maxBound=25, debug = False, LimitBounds = True):
    slope = currentReading - previousReading

    if slope == 0: # no slope found go in random direction
        xNew, yNew = moveRobot(xRobotNewFunc, yRobotNewFunc, stepSize, minBound, maxBound)
        # print("Moved randomly")
        return xNew, yNew, slope, randomNumAtoB(-2*pi,2*pi)

    else: # if there is a slope bias towards the slope
        if slope > 0:
            biasDirection = previousBias + randomNumAtoB(-biasRange,biasRange) # add random noise
            # print("Moved forward")
            xStep = stepSize * np.cos(biasDirection)
            yStep = stepSize * np.sin(biasDirection)

            xRobotNewFunc += xStep
            yRobotNewFunc += yStep

            if LimitBounds:
                xNew = wapointBoundsLimiter(xRobotNewFunc, minBound, maxBound)
                yNew = wapointBoundsLimiter(yRobotNewFunc, minBound, maxBound)
            else:
                xNew = xRobotNewFunc
                yNew = yRobotNewFunc
            if debug == True:
                print("-----------------------------")
                print()
                print("Slope : ", "{:.3f}".format(slope), " Direction : ", "{:.3f}".format(biasDirection * 180 /pi))
                print("Current Reading : ", "{:.3f}".format(currentReading))
                print()

            return xNew, yNew, slope, biasDirection

        else:
            # biasDirection = biasDirection + pi
            # print("Moved backward")
            biasDirection = previousBias + randomNumAtoB(-biasRange,biasRange) + pi # add random noise

            xStep = stepSize * np.cos(biasDirection)
            yStep = stepSize * np.sin(biasDirection)

            xRobotNewFunc += xStep
            yRobotNewFunc += yStep

            xNew = wapointBoundsLimiter(xRobotNewFunc, minBound, maxBound)
            yNew = wapointBoundsLimiter(yRobotNewFunc, minBound, maxBound)

            if debug == True:
                print("-----------------------------")
                print()
                print("Slope : ", "{:.3e}".format(slope), " Direction : ", "{:.3f}".format(biasDirection * 180 /pi))
                print("Current Reading : ", "{:.3e}".format(currentReading))
                print()

            return xNew, yNew, slope, biasDirection

def gaussFunc(xFunc, yFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    con = (QFunc/(4 * pi * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * ((yFunc**2)/DyFunc + (zFunc**2)/DzFunc))
    return con


def getReading(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    # Rotate frame

    Xw_r = np.array([xRobotDef, yRobotDef, 1])

    R = np.array([[cos(thetaFunc), -sin(thetaFunc)], [sin(thetaFunc), cos(thetaFunc)]])
    P = np.array([[xPlumeFunc, yPlumeFunc]]).T;

    bottomArray = np.array([[0, 0 , 1]]);
    # Tw_plumeFrame = np.concatenate((R, P), axis=1)

    # # Transfer matrix from plumeframe to w
    # Tw_plumeFrame = np.concatenate((Tw_plumeFrame, bottomArray), axis=0)

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


# =============================================================================
# def getReading(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
#
#     reading = xRobotDef + yRobotDef
#
#     return reading
# =============================================================================

class biasRandomWalkSim:
    def __init__(self, simStepsClass, biasRangeClass, thetaPlumeClass, xPlumeClass, yPlumeClass, QPlume, vPlume, DyPlume, DzPlume, xRobotSpawnClass, yRobotSpawnClass, minLimClass = -25, maxLimClass = 25, debug = False, LimitBounds = True):
        self.simSteps = simStepsClass
        self.biasRange = biasRangeClass * pi/180 # converts to radians
        self.thetaPlume = thetaPlumeClass * pi/180 # for getReading
        self.xPlume = xPlumeClass # for getReading
        self.yPlume = yPlumeClass # for getReading
        self.QPlume = QPlume
        self.vPlume = vPlume
        self.DyPlume = DyPlume
        self.DzPlume = DzPlume
        self.xRobotSpawn = xRobotSpawnClass
        self.yRobotSpawn = yRobotSpawnClass
        self.minLim = minLimClass
        self.maxLim = maxLimClass

        bestHit = -100

        self.xMovesList = []
        self.yMovesList = []
        self.slopeList = []
        self.biasList = []
        self.readingsList = []
        self.errorList = []


        xRobotOld = self.xRobotSpawn
        yRobotOld = self.yRobotSpawn

        # Calculate error
        xError = abs(self.xPlume - xRobotOld)
        yError = abs(self.yPlume - yRobotOld)

        # Calculate errorMagnitude
        errorMag = np.linalg.norm(np.array([xError, yError]))

        # Record robots movement
        self.xMovesList.append(xRobotOld)
        self.yMovesList.append(yRobotOld)
        self.errorList.append(errorMag)

        # Get initial reading
        previousReading = getReading(xRobotOld, yRobotOld, self.thetaPlume, self.xPlume, self.yPlume, 0.5, self.QPlume, self.vPlume, self.DyPlume, self.DzPlume)

        # Move robot randomly
        xRobot, yRobot = moveRobot(xRobotOld, yRobotOld, self.minLim, self.maxLim)

        # Calculate error
        xError = abs(self.xPlume - xRobot)
        yError = abs(self.yPlume - yRobot)

        # Calculate errorMagnitude
        errorMag = np.linalg.norm(np.array([xError, yError]))

        previousBias = np.arctan2((yRobot-yRobotOld),(xRobot-xRobotOld))

        # Record robots movement
        self.xMovesList.append(xRobot)
        self.yMovesList.append(yRobot)
        self.slopeList.append(0)
        self.biasList.append(previousBias * 180/pi)
        self.readingsList.append(previousReading)
        self.errorList.append(errorMag)

        # Start bias random walk
        for i in range(self.simSteps-1):

            # Get reading
            currentReading = getReading(xRobot, yRobot, self.thetaPlume, self.xPlume, self.yPlume, 0.5, self.QPlume, self.vPlume, self.DyPlume, self.DzPlume)

            # Move robot to new point
            xRobot, yRobot, slope, bias = biasedRandomWalk(xRobot, yRobot, previousReading, currentReading, self.biasRange, previousBias, self.minLim, self.maxLim, debug)


            xRobotOld = xRobot
            yRobotOld = yRobot
            previousReading = currentReading
            previousBias = bias

            # Record best hit
            if (currentReading > bestHit ):
                bestLocation = [xRobot,yRobot]
                bestHit = currentReading


            # Calculate error
            xError = abs(self.xPlume - xRobot)
            yError = abs(self.yPlume - yRobot)

            # Calculate errorMagnitude
            errorMag = np.linalg.norm(np.array([xError, yError]))

            # Record robots movement
            self.xMovesList.append(xRobot) # record movements
            self.yMovesList.append(yRobot)
            self.slopeList.append(slope)
            self.biasList.append(bias * 180/pi)
            self.readingsList.append(currentReading)
            self.errorList.append(errorMag)


    def plotSim(self, frameTime, debug = False, FixedSize = False):

        # Plotting
        if FixedSize:
            fig = plt.figure()
            fig.set_size_inches(12, 12)
        xPlt = []
        yPlt = []

        xPlumePlot = np.arange(self.minLim, self.maxLim+1, 0.5)
        yPlumePlot = np.arange(self.minLim, self.maxLim+1, 0.5)

        conArray = np.zeros([len(xPlumePlot), len(yPlumePlot)])

        # only for plotting purpose
        currentBestHit = -100

        for xCurrentIndex in range(len(xPlumePlot)):
            for yCurrentIndex in range(len(yPlumePlot)):
                conArray[xCurrentIndex,yCurrentIndex] = getReading(yPlumePlot[yCurrentIndex], xPlumePlot[xCurrentIndex], self.thetaPlume, self.xPlume, self.yPlume, 0.5, self.QPlume, self.vPlume, self.DyPlume, self.DzPlume)

        flat=conArray.flatten()
        flat.sort()
        secondHighestValue = flat[-2]
        ind = np.unravel_index(np.argmax(conArray, axis=None), conArray.shape)
        conArray[ind[0]][ind[1]] = secondHighestValue * 1.03

        # Start plotting
        for i in range(self.simSteps-1):

            # Plotting stuff
            plt.contourf(xPlumePlot,yPlumePlot,conArray,25)
            plt.xlim(self.minLim, self.maxLim)
            plt.ylim(self.minLim, self.maxLim)

            xRobot = self.xMovesList[i]
            yRobot = self.yMovesList[i]

            currentReading = getReading(xRobot, yRobot, self.thetaPlume, self.xPlume, self.yPlume, 0.5, self.QPlume, self.vPlume, self.DyPlume, self.DzPlume)
            if debug == True:
                print("-----------------------------")
                print()
                print("Current Reading : ", "{:.3f}".format(currentReading))
                print("Slope : ", "{:.3e}".format(self.slopeList[i]))
                print("Bias : ", "{:.3f}".format(self.biasList[i]))
                print()

            xPlt.append(xRobot)
            yPlt.append(yRobot)

            # Record best hit
            if (currentReading > currentBestHit ):
                bestLocation = [xRobot,yRobot]
                currentBestHit = currentReading

            plt.plot(xPlt,yPlt,'r')
            plt.plot(xRobot,yRobot,'go', markersize=6)

            # plt.plot(bestLocation[0],bestLocation[1],'yo', markersize=10)
            plt.show()
            plt.pause(frameTime)
            if i != (self.simSteps-2):
                plt.clf()

    def plotError(self):
        fig = plt.figure()
        plt.plot(list(range(len(self.errorList))),self.errorList)
        plt.grid()
        plt.xlim(0, max(list(range(len(self.errorList))))*1.05)
        plt.ylim(0, max(self.errorList)*1.05)
        plt.xlabel("Number of steps")
        plt.ylabel("Magnitude of location error [m]")


class MultiBRWSims:
    def __init__(self, NumberOfBRW, simStepsClass, biasRangeClass, thetaPlumeClass, xPlumeClass, yPlumeClass, QPlume, vPlume, DyPlume, DzPlume, xRobotSpawnClass, yRobotSpawnClass, minLimClass = -25, maxLimClass = 25, randomSpawn = False, LimitBounds = True):
        self.BRWSimxMovesList = []
        self.BRWSimyMovesList = []
        self.BRWSimReadingsList = []
        self.BRWErrorOverTime = []
        self.BRWAverageErrorOverTime = []
        self.BRWSTDErrorOverTime = []

        for i in range(NumberOfBRW):
            if randomSpawn:
                xRobotSpawnClass = randomNumAtoB(minLimClass, maxLimClass)
                yRobotSpawnClass = randomNumAtoB(minLimClass, maxLimClass)

            BRW = biasRandomWalkSim(simStepsClass, biasRangeClass, thetaPlumeClass, xPlumeClass, yPlumeClass, QPlume, vPlume, DyPlume, DzPlume, xRobotSpawnClass, yRobotSpawnClass, minLimClass, maxLimClass, LimitBounds = LimitBounds)

            # Get robots movements and readings from sim
            self.BRWSimxMovesList.append(BRW.xMovesList)
            self.BRWSimyMovesList.append(BRW.yMovesList)
            self.BRWSimReadingsList.append(BRW.readingsList)
            self.BRWErrorOverTime.append(BRW.errorList)

        self.BRWErrorOverTime = np.array(self.BRWErrorOverTime)


        for i in range(simStepsClass):
            self.BRWAverageErrorOverTime.append(np.mean(self.BRWErrorOverTime[:,i]))
            self.BRWSTDErrorOverTime.append(np.std(self.BRWErrorOverTime[:,i]))

    def plotError(self,plotSTD = True):
        if plotSTD:
            upperSigma = np.array(self.BRWAverageErrorOverTime) + 2*np.array(self.BRWSTDErrorOverTime)
            lowerSigma = np.array(self.BRWAverageErrorOverTime) - 2*np.array(self.BRWSTDErrorOverTime)

        # fig = plt.figure()
        plt.plot(list(range(len(self.BRWAverageErrorOverTime))),self.BRWAverageErrorOverTime)
        if plotSTD:
            plt.fill_between(list(range(len(self.BRWSTDErrorOverTime))), lowerSigma, upperSigma, alpha=0.2)
            plt.legend(('Average', 'std'))

        # plt.plot(list(range(len(self.BRWSTDErrorOverTime))), upperSigma)
        # plt.plot(list(range(len(self.BRWSTDErrorOverTime))), lowerSigma)

        plt.grid()
        plt.xlim(0, max(list(range(len(self.BRWAverageErrorOverTime)))))
        if plotSTD:
            plt.ylim(-10, max(upperSigma)*1.05)
        else:
            plt.ylim(0, max(self.BRWAverageErrorOverTime)*1.05)
        plt.xlabel("Number of steps")
        plt.ylabel("Magnitude of location error [m]")
