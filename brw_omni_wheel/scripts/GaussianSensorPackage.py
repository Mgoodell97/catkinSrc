# -*- coding: utf-8 -*-

from math import pi, sqrt, exp, cos, sin
from numpy.random import normal

import numpy as np

from scipy.cluster.hierarchy import dendrogram, linkage
from sklearn.cluster import AgglomerativeClustering

# =============================================================================
# Functions
# =============================================================================

def gaussFunc(xFunc, yFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    con = (QFunc/(4 * pi * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * ((yFunc**2)/DyFunc + (zFunc**2)/DzFunc))
    return con

def GaussianSensor(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc, addNoise = False):
    
    # thetaFunc = thetaFunc
    
    # Rotate frame
    Stheta = sin(thetaFunc)
    Ctheta = cos(thetaFunc)

    XplumeFrame = Ctheta  * xRobotDef + Stheta * yRobotDef + -Ctheta * xPlumeFunc - Stheta * yPlumeFunc
    YplumeFrame = -Stheta * xRobotDef + Ctheta * yRobotDef +  Stheta * xPlumeFunc - Ctheta * yPlumeFunc

    if XplumeFrame <= 0:
        reading = 0
    else:   
        reading = gaussFunc(XplumeFrame,YplumeFrame,zFunc,QFunc,vFunc,DyFunc,DzFunc)*1000 # Converts to ppm
        
    if addNoise == False:
        return reading
    if addNoise:
        return reading*normal(1,0.075)

def MPSSensor(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    
    # thetaFunc = thetaFunc
    
    # Rotate frame
    Stheta = sin(thetaFunc)
    Ctheta = cos(thetaFunc)

    XplumeFrame = Ctheta  * xRobotDef + Stheta * yRobotDef + -Ctheta * xPlumeFunc - Stheta * yPlumeFunc
    YplumeFrame = -Stheta * xRobotDef + Ctheta * yRobotDef +  Stheta * xPlumeFunc - Ctheta * yPlumeFunc

    if XplumeFrame <= 0:
        reading = 0
    else:   
        reading = gaussFunc(XplumeFrame,YplumeFrame,zFunc,QFunc,vFunc,DyFunc,DzFunc)*1000 # Converts to ppm
    
    if reading < 0:
        reading = 0
    elif reading > 1500:
        reading = 1500
        
    return reading

def combinePlumes(plumeList, combineThreshold):
    plumeArray = np.stack((plumeList[0], plumeList[1], plumeList[2], plumeList[3], plumeList[4], plumeList[5], plumeList[6], plumeList[7]), axis=0).T
    
    X = np.stack((plumeList[0], plumeList[1], plumeList[2]), axis=0).T
    model = AgglomerativeClustering(n_clusters = None, distance_threshold=combineThreshold, affinity='euclidean', linkage='average')
    model.fit(X)
    labels = model.labels_
    
    xPlumesNew     = []
    yPlumesNew     = []
    zPlumesNew     = []
    thetaPlumesNew = []
    QPlumesNew     = []
    vPlumesNew     = []
    DyPlumesNew    = []
    DzPlumesNew    = []
    
    for i in range(max(labels+1)):
        averagePlume = np.average(plumeArray[labels==i],axis=0)
        xPlumesNew.append(averagePlume[0])
        yPlumesNew.append(averagePlume[1])
        zPlumesNew.append(averagePlume[2])
        thetaPlumesNew.append(averagePlume[3])
        QPlumesNew.append(averagePlume[4])
        vPlumesNew.append(averagePlume[5])
        DyPlumesNew.append(averagePlume[6])
        DzPlumesNew.append(averagePlume[7])

    return xPlumesNew, yPlumesNew, zPlumesNew, thetaPlumesNew, QPlumesNew, vPlumesNew, DyPlumesNew, DzPlumesNew

# =============================================================================
# classes
# =============================================================================

class GaussianMultiPlume:
    def __init__(self, thetaList, xList, yList, zList, QList, vList, DyList, DzList, addNoise = False):
        self.thetaList = thetaList
        self.xList = xList
        self.yList = yList
        self.zList = zList
        self.QList = QList
        self.vList = vList
        self.DyList = DyList
        self.DzList = DzList
        self.addNoise = addNoise
        
    def getReading(self, x, y, z):
        # need to fix z height
        reading = 0
        for i in range(len(self.xList)):
            reading = reading + GaussianSensor(x, y, self.thetaList[i], self.xList[i], self.yList[i], 0, self.QList[i], self.vList[i], self.DyList[i], self.DzList[i], self.addNoise)
            
        return reading