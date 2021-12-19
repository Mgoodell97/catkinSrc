import numpy as np
# from math import pi, sqrt, exp, cos, sin
from GaussianSensorPackage import GaussianSensor, GaussianMultiPlume, GaussianSensorNB, getReadingMultiPlume
from numba import njit

# =============================================================================
#
# =============================================================================

@njit
def pdf(x, u, sigma):
    return (1/np.sqrt( 2 * np.pi * sigma**2 ))*np.exp(-(x-u)**2/(2*sigma**2))

# =============================================================================
#
# =============================================================================

def conditionalEntropyAndMeasurementEntropy(xInfoDes, zt, xp, wp, sigma, AMatrix):
    particleObsravtionAtXDes = np.zeros(len(wp))

    xPlumes     = AMatrix[0]  # [m]
    yPlumes     = AMatrix[1]  # [m]
    zPlumes     = AMatrix[2]  # [m]
    thetaPlumes = AMatrix[3]  # [rads]
    Qs          = AMatrix[4]  # [kg/s]
    vs          = AMatrix[5]  # [m/s]
    Dys         = AMatrix[6]  # [m^2/s]
    Dzs         = AMatrix[7]  # [m^2/s]

    multiPlume = GaussianMultiPlume(thetaPlumes, xPlumes, yPlumes, zPlumes, Qs, vs, Dys, Dzs, False)

    for j in range(len(wp)):
        # Get estimated reading
        xPF     = np.array(xp[j][0], dtype=np.float32)  # [m]
        yPF     = np.array(xp[j][1], dtype=np.float32)  # [m]
        zPF     = np.array(xp[j][2], dtype=np.float32)  # [m]
        thetaPF = np.array(xp[j][3], dtype=np.float32)  # [rads]
        QPF     = np.array(xp[j][4], dtype=np.float32)  # [kg/s]
        vPF     = np.array(xp[j][5], dtype=np.float32)  # [m/s]
        DyPF    = np.array(xp[j][6], dtype=np.float32)  # [m^2/s]
        DzPF    = np.array(xp[j][7], dtype=np.float32)  # [m^2/s]

        particleObsravtionAtXDes[j] = GaussianSensorNB(xInfoDes[0], xInfoDes[1], thetaPF, xPF, yPF, xInfoDes[2] - zPF, QPF, vPF, DyPF, DzPF) - multiPlume.getReading(xInfoDes[0], xInfoDes[1], xInfoDes[2])

    partileLikelihood = pdf(zt, particleObsravtionAtXDes, sigma)

    probabilityZt = wp * partileLikelihood
    probabilityZtSum = np.sum(probabilityZt)

    measurementEntropySum = probabilityZtSum * np.log(probabilityZtSum)
    conditionalEntropySum = np.sum(probabilityZt * np.log(partileLikelihood))

    return -measurementEntropySum, -conditionalEntropySum

@njit
def conditionalEntropyAndMeasurementEntropyNB(xInfoDes, zt, xp, wp, sigma, AMatrix):
    particleObsravtionAtXDes = np.zeros(len(wp))

    for j in range(len(wp)):
        # Get estimated reading
        xPF     = np.array(xp[j][0], dtype=np.float32)  # [m]
        yPF     = np.array(xp[j][1], dtype=np.float32)  # [m]
        zPF     = np.array(xp[j][2], dtype=np.float32)  # [m]
        thetaPF = np.array(xp[j][3], dtype=np.float32)  # [rads]
        QPF     = np.array(xp[j][4], dtype=np.float32)  # [kg/s]
        vPF     = np.array(xp[j][5], dtype=np.float32)  # [m/s]
        DyPF    = np.array(xp[j][6], dtype=np.float32)  # [m^2/s]
        DzPF    = np.array(xp[j][7], dtype=np.float32)  # [m^2/s]

        # multiPlumeReading = getReadingMultiPlume(xInfoDes[0], xInfoDes[1], xInfoDes[2] - zPF, AMatrix)
        #
        # particleObsravtionAtXDes[j] = GaussianSensorNB(xInfoDes[0], xInfoDes[1], thetaPF, xPF, yPF, xInfoDes[2] - zPF, QPF, vPF, DyPF, DzPF, False) - multiPlumeReading
        multiPlumeReading = getReadingMultiPlume(xInfoDes[0], xInfoDes[1], xInfoDes[2] - zPF, AMatrix)
        # GaussianSensorNB(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc, addNoise = False)
        # print(xInfoDes[2] - zPF)
        particleObsravtionAtXDes[j] = GaussianSensorNB(xInfoDes[0], xInfoDes[1], thetaPF, xPF, yPF, xInfoDes[2] - zPF, QPF, vPF, DyPF, DzPF, False) - multiPlumeReading


    partileLikelihood = pdf(zt, particleObsravtionAtXDes, sigma)

    probabilityZt = wp * partileLikelihood
    probabilityZtSum = np.sum(probabilityZt)

    measurementEntropySum = probabilityZtSum * np.log(probabilityZtSum)
    conditionalEntropySum = np.sum(probabilityZt * np.log(partileLikelihood))

    return -measurementEntropySum, -conditionalEntropySum

# =============================================================================
#
# =============================================================================

def informationAtXNew(xInfoDes,xp,wp,sigma,ztMin, ztMax, xGuass, wGuass, AMatrix):
    b = ztMin
    a = ztMax

    integralConditional = 0
    integralMeasurment = 0
    for i in range(len(xGuass)):
        convertedZt = ((b-a)/2) * xGuass[i] + ((b+a)/2)

        measurementEntropySum, conditionalEntropySum = conditionalEntropyAndMeasurementEntropy(xInfoDes, convertedZt, xp, wp, sigma, AMatrix)
        integralMeasurment = integralMeasurment + wGuass[i] * measurementEntropySum
        integralConditional = integralConditional + wGuass[i] * conditionalEntropySum

    MutualInfo = integralMeasurment - integralConditional

    return MutualInfo

@njit
def informationAtXNewNB(xInfoDes,xp,wp,sigma,ztMin, ztMax, xGuass, wGuass, AMatrix):
    b = ztMin
    a = ztMax

    integralConditional = 0
    integralMeasurment = 0

    for i in range(len(xGuass)):
        convertedZt = ((b-a)/2) * xGuass[i] + ((b+a)/2)

        measurementEntropySum, conditionalEntropySum = conditionalEntropyAndMeasurementEntropyNB(xInfoDes, convertedZt, xp, wp, sigma, AMatrix)
        integralMeasurment = integralMeasurment + wGuass[i] * measurementEntropySum
        integralConditional = integralConditional + wGuass[i] * conditionalEntropySum

    MutualInfo = integralMeasurment - integralConditional

    return MutualInfo

# =============================================================================
#
# =============================================================================

# @njit
@njit
def computeInfoMap( xLim, yLim, plumeHeights, xp, wp, sigma, ztMinMain, ztMaxMain, xGuass, wGuass, Ahat, arraySize = 100):
    xPlumePlot = np.linspace(xLim[0], xLim[1], arraySize)
    yPlumePlot = np.linspace(yLim[0], yLim[1], arraySize)

    infoArray = np.zeros((arraySize ,arraySize))

    for xIndex, x in enumerate(xPlumePlot):
        for yIndex, y in enumerate(yPlumePlot):
            xInfoDes = np.array([x, y, plumeHeights])
            infoArray[yIndex,xIndex] = informationAtXNewNB(xInfoDes, xp, wp, sigma, ztMinMain, ztMaxMain, xGuass, wGuass, Ahat)

    return infoArray


class RobotMotion:
    def __init__(self, PoseVec, VelVec = np.array([0,0,0]), MaxVel = 0.5):
        self.PoseVec = PoseVec
        self.MaxVel = MaxVel
        if VelVec[0] == 0 and VelVec[1] == 0 and VelVec[2] == 0:
            self.VelVec = VelVec
        else:
            self.VelVec = VelVec/( np.sqrt( pow(VelVec[0],2) + pow(VelVec[1],2) + pow(VelVec[2],2) ) )
        # self.VelVec = np.clip(VelVec, -MaxVel, MaxVel)

    def updatePose(self, DT = 1, Velinput = np.array([0,0,0]), poseHeight = 0):
        b = 0.5
        if Velinput[0] == 0 and Velinput[1] == 0 and Velinput[2] == 0:
            # Update velocity
            self.VelVec = np.clip(self.VelVec, -self.MaxVel, self.MaxVel)
            self.VelVec = b*self.VelVec

            # Update position
            self.PoseVec = self.PoseVec + self.VelVec*DT
            self.PoseVec[2] = poseHeight

        else:
            # Update velocity
            VelMagnitude = np.sqrt( pow(Velinput[0],2) + pow(Velinput[1],2) + pow(Velinput[2],2) )
            if VelMagnitude < self.MaxVel:
                self.VelVec = np.clip(Velinput, -self.MaxVel, self.MaxVel)
            else:
                self.VelVec = self.MaxVel * Velinput/( np.sqrt( pow(Velinput[0],2) + pow(Velinput[1],2) + pow(Velinput[2],2) ) )

            # Update position
            self.PoseVec = self.PoseVec + self.VelVec*DT


    def updatedPoseSim(self, Velinput, DT = 1, poseHeight = 0):

        # Update velocity
        VelMagnitude = np.sqrt( pow(Velinput[0],2) + pow(Velinput[1],2) + pow(Velinput[2],2) )
        if VelMagnitude < self.MaxVel:
            VelVec = np.clip(Velinput, -self.MaxVel, self.MaxVel)
        else:
            VelVec = self.MaxVel * Velinput/( np.sqrt( pow(Velinput[0],2) + pow(Velinput[1],2) + pow(Velinput[2],2) ) )

        # Update position
        PoseVecSim = self.PoseVec + VelVec*DT
        PoseVecSim[2] = poseHeight

        return PoseVecSim
