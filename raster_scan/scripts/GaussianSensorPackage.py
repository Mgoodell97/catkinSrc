# -*- coding: utf-8 -*-

import numpy as np

from math import pi, sqrt, exp, cos, sin
from numpy.random import normal

from numba import njit, jit, float32, boolean, float64, cuda
# from numba.experimental import jitclass

from scipy.cluster.hierarchy import dendrogram, linkage
from sklearn.cluster import AgglomerativeClustering

from scipy.cluster.hierarchy import ward, fcluster, average
from scipy.spatial.distance import pdist

# =============================================================================
# Functions
# =============================================================================

def accountForSensorDynamics(zCurrent, zPast, yPast, timeStep):

    if zCurrent <= -20:
        zCurrent = -20

    if zCurrent > zPast: # rise
        tau = 17.86
    else: # fall
        tau = 19.54

    a = (8000/5000)/tau
    B = 1/tau
    gamma = 10/tau

    # Working without LP
    uDot = (zCurrent - zPast)/timeStep

    zAdjusted = (uDot + zCurrent*B)/a

    # With LP
    zAdjusted = (zAdjusted*gamma + yPast/timeStep)/(1/timeStep+gamma)

    zPast = zCurrent
    yPast = zAdjusted

    if zAdjusted <= -20:
        zAdjusted = -20
    elif zAdjusted >= 5000:
        zAdjusted = 5000

    return zAdjusted, zPast, yPast

# =============================================================================
#
# =============================================================================

def gaussFunc(xFunc, yFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    con = (QFunc/(4 * pi * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * ((yFunc**2)/DyFunc + (zFunc**2)/DzFunc))
    return con

@njit
def gaussFuncNB(xFunc, yFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    con = (QFunc/(4 * pi * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * ((yFunc**2)/DyFunc + (zFunc**2)/DzFunc))
    return con

# =============================================================================
#
# =============================================================================

def GaussianSensor(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc, addNoise = False):
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

@njit
def GaussianSensorNB(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc, addNoise = False):
    # Rotate frame
    Stheta = np.sin(thetaFunc)
    Ctheta = np.cos(thetaFunc)

    XplumeFrame = Ctheta  * xRobotDef + Stheta * yRobotDef + -Ctheta * xPlumeFunc - Stheta * yPlumeFunc
    YplumeFrame = -Stheta * xRobotDef + Ctheta * yRobotDef +  Stheta * xPlumeFunc - Ctheta * yPlumeFunc

    if XplumeFrame <= 0:
        reading = 0
    else:
        reading = gaussFuncNB(XplumeFrame,YplumeFrame,zFunc,QFunc,vFunc,DyFunc,DzFunc)*1000 # Converts to ppm

    if addNoise == False:
        return reading
    if addNoise:
        return reading*normal(1,0.075)

# =============================================================================
#
# =============================================================================

def MPSSensor(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
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

@njit
def MPSSensorNB(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    # Rotate frame
    Stheta = sin(thetaFunc)
    Ctheta = cos(thetaFunc)

    XplumeFrame = Ctheta  * xRobotDef + Stheta * yRobotDef + -Ctheta * xPlumeFunc - Stheta * yPlumeFunc
    YplumeFrame = -Stheta * xRobotDef + Ctheta * yRobotDef +  Stheta * xPlumeFunc - Ctheta * yPlumeFunc

    if XplumeFrame <= 0:
        reading = 0
    else:
        reading = gaussFuncNB(XplumeFrame,YplumeFrame,zFunc,QFunc,vFunc,DyFunc,DzFunc)*1000 # Converts to ppm

    if reading < 0:
        reading = 0
    elif reading > 1500:
        reading = 1500

    return reading

# =============================================================================
#
# =============================================================================

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


    xPlumesNew = np.array(xPlumesNew)
    yPlumesNew = np.array(yPlumesNew)
    zPlumesNew = np.array(zPlumesNew)
    thetaPlumesNew = np.array(thetaPlumesNew)
    QPlumesNew = np.array(QPlumesNew)
    vPlumesNew = np.array(vPlumesNew)
    DyPlumesNew = np.array(DyPlumesNew)
    DzPlumesNew = np.array(DzPlumesNew)

    return xPlumesNew, yPlumesNew, zPlumesNew, thetaPlumesNew, QPlumesNew, vPlumesNew, DyPlumesNew, DzPlumesNew

def combinePlumesNew(plumeList, combineThreshold):
    plumeArray = np.stack((plumeList[0], plumeList[1], plumeList[2], plumeList[3], plumeList[4], plumeList[5], plumeList[6], plumeList[7]), axis=0).T

    X = np.stack((plumeList[0], plumeList[1], plumeList[2]), axis=0).T
    Z = average(pdist(X))
    labels = fcluster(Z, t=combineThreshold, criterion='distance')

    xPlumesNew     = []
    yPlumesNew     = []
    zPlumesNew     = []
    thetaPlumesNew = []
    QPlumesNew     = []
    vPlumesNew     = []
    DyPlumesNew    = []
    DzPlumesNew    = []

    for i in range(1,max(labels+1)):

        averagePlume = np.average(plumeArray[labels==i],axis=0)
        xPlumesNew.append(averagePlume[0])
        yPlumesNew.append(averagePlume[1])
        zPlumesNew.append(averagePlume[2])
        thetaPlumesNew.append(averagePlume[3])
        QPlumesNew.append(averagePlume[4])
        vPlumesNew.append(averagePlume[5])
        DyPlumesNew.append(averagePlume[6])
        DzPlumesNew.append(averagePlume[7])

    xPlumesNew = np.array(xPlumesNew)
    yPlumesNew = np.array(yPlumesNew)
    zPlumesNew = np.array(zPlumesNew)
    thetaPlumesNew = np.array(thetaPlumesNew)
    QPlumesNew = np.array(QPlumesNew)
    vPlumesNew = np.array(vPlumesNew)
    DyPlumesNew = np.array(DyPlumesNew)
    DzPlumesNew = np.array(DzPlumesNew)

    return xPlumesNew, yPlumesNew, zPlumesNew, thetaPlumesNew, QPlumesNew, vPlumesNew, DyPlumesNew, DzPlumesNew

# =============================================================================
#
# =============================================================================

def generatePlumes(numberOfPlumes, minDistanceBetweenPlumes, xLims, yLims, zLims, thetaLims, QLims, vLims, DyLims, DzLims):
    xPlumes = []
    yPlumes = []

    while len(xPlumes) < numberOfPlumes:

        xPlumeGuess = np.random.uniform(xLims[0], xLims[1]) # [m]
        yPlumeGuess = np.random.uniform(yLims[0], yLims[1]) # [m]
        if len(xPlumes) == 0:
            xPlumes.append(xPlumeGuess)
            yPlumes.append(yPlumeGuess)
        else:
            distanceFlag = True
            for i in range(len(xPlumes)):
                distance = sqrt( (xPlumeGuess - xPlumes[i])**2 + (yPlumeGuess - yPlumes[i])**2  )

                if distance <= minDistanceBetweenPlumes:
                    distanceFlag = False

            if distanceFlag:
                xPlumes.append(xPlumeGuess)
                yPlumes.append(yPlumeGuess)

    xPlumes     = np.array(xPlumes, dtype=np.float32)
    yPlumes     = np.array(yPlumes, dtype=np.float32)
    zPlumes     = np.random.uniform(zLims[0]    , zLims[1]    , numberOfPlumes).astype(dtype=np.float32) # [m]
    thetaPlumes = np.random.uniform(thetaLims[0], thetaLims[1], numberOfPlumes).astype(dtype=np.float32) # [rads]
    Qs          = np.random.uniform(QLims[0]    , QLims[1]    , numberOfPlumes).astype(dtype=np.float32) # [kg/s]
    vs          = np.random.uniform(vLims[0]    , vLims[1]    , numberOfPlumes).astype(dtype=np.float32) # [m/s]
    Dys         = np.random.uniform(DyLims[0]   , DyLims[1]   , numberOfPlumes).astype(dtype=np.float32) # [m/s]
    Dzs         = np.random.uniform(DzLims[0]   , DzLims[1]   , numberOfPlumes).astype(dtype=np.float32) # [m/s]

    return xPlumes, yPlumes, zPlumes, thetaPlumes, Qs, vs, Dys, Dzs

# =============================================================================
#
# =============================================================================

def computeGaussianPlumeMap(AMatrix, xLim, yLim, plumeHeights, arraySize, addNoise = False):

    xPlumePlot = np.linspace(xLim[0], xLim[1], arraySize)
    yPlumePlot = np.linspace(yLim[0], yLim[1], arraySize)
    conArray = np.zeros((arraySize ,arraySize))

    xPlumes     = AMatrix[0]  # [m]
    yPlumes     = AMatrix[1]  # [m]
    zPlumes     = AMatrix[2]  # [m]
    thetaPlumes = AMatrix[3]  # [rads]
    Qs          = AMatrix[4]  # [kg/s]
    vs          = AMatrix[5]  # [m/s]
    Dys         = AMatrix[6]  # [m^2/s]
    Dzs         = AMatrix[7]  # [m^2/s]

    multiPlume = GaussianMultiPlume(thetaPlumes, xPlumes, yPlumes, zPlumes, Qs, vs, Dys, Dzs, addNoise)

    for xIndex, x in enumerate(xPlumePlot):
        for yIndex, y in enumerate(yPlumePlot):
            conArray[yIndex,xIndex] = multiPlume.getReading(x, y, plumeHeights)

    return conArray

@njit
def computeGaussianPlumeMapNB(AMatrix, xLim, yLim, plumeHeights, arraySize, addNoise = False):

    xPlumePlot = np.linspace(xLim[0], xLim[1], arraySize)
    yPlumePlot = np.linspace(yLim[0], yLim[1], arraySize)
    conArray = np.zeros((arraySize ,arraySize))

    xPlumes     = AMatrix[0]  # [m]
    yPlumes     = AMatrix[1]  # [m]
    zPlumes     = AMatrix[2]  # [m]
    thetaPlumes = AMatrix[3]  # [rads]
    Qs          = AMatrix[4]  # [kg/s]
    vs          = AMatrix[5]  # [m/s]
    Dys         = AMatrix[6]  # [m^2/s]
    Dzs         = AMatrix[7]  # [m^2/s]

    multiPlume = GaussianMultiPlumeNB(thetaPlumes, xPlumes, yPlumes, zPlumes, Qs, vs, Dys, Dzs, addNoise)

    for xIndex, x in enumerate(xPlumePlot):
        for yIndex, y in enumerate(yPlumePlot):
            conArray[yIndex,xIndex] = multiPlume.getReading(x, y, plumeHeights)

    return conArray

@njit
def computeGaussianPlumeMapNBNoClass(AMatrix, xLim, yLim, plumeHeights, arraySize, addNoise = False):

    xPlumePlot = np.linspace(xLim[0], xLim[1], arraySize)
    yPlumePlot = np.linspace(yLim[0], yLim[1], arraySize)
    conArray = np.zeros((arraySize ,arraySize))

    for xIndex, x in enumerate(xPlumePlot):
        for yIndex, y in enumerate(yPlumePlot):
            conArray[yIndex,xIndex] = getReadingMultiPlume(x, y, plumeHeights, AMatrix, addNoise)

    return conArray

# =============================================================================
#
# =============================================================================

@njit
def computeGaussianPlumeMapTwo(Atrue, Ahat, xLim, yLim, plumeHeights, arraySize = 100):
    xPlumePlot = np.linspace(xLim[0], xLim[1], arraySize)
    yPlumePlot = np.linspace(yLim[0], yLim[1], arraySize)

    ATrueConArray = np.zeros((arraySize ,arraySize))
    AHatConArray  = np.zeros((arraySize ,arraySize))


    xPlumesTrue     = Atrue[0]  # [m]
    yPlumesTrue     = Atrue[1]  # [m]
    zPlumesTrue     = Atrue[2]  # [m]
    thetaPlumesTrue = Atrue[3]  # [rads]
    QsTrue          = Atrue[4]  # [kg/s]
    vsTrue          = Atrue[5]  # [m/s]
    DysTrue         = Atrue[6]  # [m^2/s]
    DzsTrue         = Atrue[7]  # [m^2/s]

    xPlumesHat     = Ahat[0]  # [m]
    yPlumesHat     = Ahat[1]  # [m]
    zPlumesHat     = Ahat[2]  # [m]
    thetaPlumesHat = Ahat[3]  # [rads]
    QsHat          = Ahat[4]  # [kg/s]
    vsHat          = Ahat[5]  # [m/s]
    DysHat         = Ahat[6]  # [m^2/s]
    DzsHat         = Ahat[7]  # [m^2/s]

    multiPlumeTrue = GaussianMultiPlumeNB(thetaPlumesTrue, xPlumesTrue, yPlumesTrue, zPlumesTrue, QsTrue, vsTrue, DysTrue, DzsTrue, False)
    multiPlumeHat = GaussianMultiPlumeNB(thetaPlumesHat, xPlumesHat, yPlumesHat, zPlumesHat, QsHat, vsHat, DysHat, DzsHat, False)

    for xIndex, x in enumerate(xPlumePlot):
        for yIndex, y in enumerate(yPlumePlot):
            ATrueConArray[yIndex,xIndex] = multiPlumeTrue.getReading(x, y, plumeHeights)
            AHatConArray[yIndex,xIndex] = multiPlumeHat.getReading(x, y, plumeHeights)

    mse = np.square(np.subtract(ATrueConArray, AHatConArray)).mean()

    return ATrueConArray, AHatConArray

# =============================================================================
#
# =============================================================================

@njit
def getReadingMultiPlume(x, y, z, AMatrix, addNoise = False):
    xPlumes     = AMatrix[0]  # [m]
    yPlumes     = AMatrix[1]  # [m]
    zPlumes     = AMatrix[2]  # [m]
    thetaPlumes = AMatrix[3]  # [rads]
    Qs          = AMatrix[4]  # [kg/s]
    vs          = AMatrix[5]  # [m/s]
    Dys         = AMatrix[6]  # [m^2/s]
    Dzs         = AMatrix[7]  # [m^2/s]

    reading = 0
    for i in range(len(xPlumes)):
        reading = reading + GaussianSensorNB(x, y, thetaPlumes[i], xPlumes[i], yPlumes[i], z - zPlumes[i], Qs[i], vs[i], Dys[i], Dzs[i], addNoise)

    return reading

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
        reading = 0
        for i in range(len(self.xList)):
            reading = reading + GaussianSensor(x, y, self.thetaList[i], self.xList[i], self.yList[i], z - self.zList[i], self.QList[i], self.vList[i], self.DyList[i], self.DzList[i], self.addNoise)
        return reading

# multiPlumeSpec = [
#     ('thetaList', float32[:]),
#     ('xList',     float32[:]),
#     ('yList',     float32[:]),
#     ('zList',     float32[:]),
#     ('QList',     float32[:]),
#     ('vList',     float32[:]),
#     ('DyList',    float32[:]),
#     ('DzList',    float32[:]),
#     ('addNoise',  boolean)
# ]
#
# @jitclass(multiPlumeSpec)
# class GaussianMultiPlumeNB:
#     def __init__(self, thetaList, xList, yList, zList, QList, vList, DyList, DzList, addNoise = False):
#         self.thetaList = thetaList
#         self.xList = xList
#         self.yList = yList
#         self.zList = zList
#         self.QList = QList
#         self.vList = vList
#         self.DyList = DyList
#         self.DzList = DzList
#         self.addNoise = addNoise
#
#     def getReading(self, x, y, z):
#         reading = 0
#         for i in range(len(self.xList)):
#             reading = reading + GaussianSensorNB(x, y, self.thetaList[i], self.xList[i], self.yList[i], z - self.zList[i], self.QList[i], self.vList[i], self.DyList[i], self.DzList[i], self.addNoise)
#
#         return reading
