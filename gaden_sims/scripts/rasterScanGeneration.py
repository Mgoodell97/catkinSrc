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