# -*- coding: utf-8 -*-
"""
Created on Sat Sep  4 13:41:23 2021

@author: asgar
"""

import numpy as np

def rasterScanGen(xRange, xNumSteps, yRange, yNumSteps):
    xArray = np.linspace(xRange[0], xRange[1], xNumSteps, endpoint=True)
    yArray = np.linspace(yRange[0], yRange[1], yNumSteps, endpoint=True)


    xScan = []
    yScan = []

    for i, yi in enumerate(yArray):
        xScan.append(xArray[::(-1)**i]) # reverse when i is odd
        yScan.append(np.ones_like(xArray) * yi)

    # squeeze lists together to vectors
    xScan = np.concatenate(xScan)
    yScan = np.concatenate(yScan)

    desiredWaypoints = np.array([xScan, yScan]).T

    return desiredWaypoints

def rasterScanGenYX(xRange, xNumSteps, yRange, yNumSteps, flipX = False):

    if flipX:
        xRange = np.flip(xRange)

    xArray = np.linspace(xRange[0], xRange[1], xNumSteps, endpoint=True)
    yArray = np.linspace(yRange[0], yRange[1], yNumSteps, endpoint=True)

    xScan = []
    yScan = []

    for i, xi in enumerate(xArray):
        yScan.append(yArray[::(-1)**i]) # reverse when i is odd
        xScan.append(np.ones_like(yArray) * xi)

    # squeeze lists together to vectors
    xScan = np.concatenate(xScan)
    yScan = np.concatenate(yScan)

    desiredWaypoints = np.array([xScan, yScan]).T

    return desiredWaypoints

def rasterScanGenRotate(xRange, yRange, stepSize, theta):
    
    xMinRaster = (xRange[0])
    xMaxRaster = (xRange[1])

    yMinRaster = (xRange[0])
    yMaxRaster = (xRange[1])

    xDiff = (xMaxRaster-xMinRaster)
    xAvg  = (xMaxRaster-xMinRaster)/2

    xLength = xDiff*np.sqrt(2)
    xMinRaster = xAvg - xLength/2
    xMaxRaster = xAvg + xLength/2

    yDiff = (yMaxRaster-yMinRaster)
    yAvg  = (yMaxRaster-yMinRaster)/2

    yLength = yDiff*np.sqrt(2)
    yMinRaster = yAvg - yLength/2
    yMaxRaster = yAvg + yLength/2

    xRangeNew = np.array([xMinRaster,xMaxRaster])
    yRangeNew = np.array([yMinRaster,yMaxRaster])

    xAvg = (xRangeNew[0] + xRangeNew[1])/(2)
    yAvg = (yRangeNew[0] + yRangeNew[1])/(2)


    xArray = np.arange(xRangeNew[0], xRangeNew[1] + stepSize, stepSize)
    yArray = np.arange(yRangeNew[0], yRangeNew[1] + stepSize, stepSize)

    xScan = []
    yScan = []

    for i, yi in enumerate(yArray):
        xScan.append(xArray[::(-1)**i]) # reverse when i is odd
        yScan.append(np.ones_like(xArray) * yi)

    # squeeze lists together to vectors
    xScan = np.concatenate(xScan)
    yScan = np.concatenate(yScan)

    desiredWaypoints = np.array([xScan, yScan]).T

    Ctheta = np.cos(theta)
    Stheta = np.sin(theta)

    newArrayList = []

    for i, currentWaypoint in enumerate(desiredWaypoints):

        xTrans = currentWaypoint[0] - xAvg
        yTrans = currentWaypoint[1] - yAvg

        xRotated = Ctheta * xTrans - Stheta * yTrans + xAvg
        yRotated = Stheta * xTrans + Ctheta * yTrans + yAvg

        desiredWaypoints[i,:] = np.array([xRotated, yRotated])

        if not xRotated < xRange[0] and not xRotated > xRange[1] and not yRotated < yRange[0] and not yRotated > yRange[1]:
            newArrayList.append(np.array([xRotated, yRotated]))

    desiredWaypoints = np.array(newArrayList)

    return desiredWaypoints
