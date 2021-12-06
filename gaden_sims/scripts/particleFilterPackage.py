# -*- coding: utf-8 -*-

# =============================================================================
# Packages
# =============================================================================

import numpy as np

from math import cos, sin, pi, acos, sqrt, exp

from GaussianSensorPackage import GaussianSensorNB, getReadingMultiPlume

from numba import njit, jit, float32, boolean, float64, cuda, int32
# from numba.experimental import jitclass

from sklearn.metrics.pairwise import pairwise_distances_argmin
from sklearn.cluster import MiniBatchKMeans

# =============================================================================
# Functions
# =============================================================================

@njit
def pdf(x, u, sigma):
    return (1/np.sqrt( 2 * np.pi * sigma**2 ))*np.exp(-(x-u)**2/(2*sigma**2))

@njit
def observerEquationNB(robotPose, xp, NumOfParticles, AMatrix):

    yp = np.zeros(NumOfParticles)
    for i in range(NumOfParticles):
        multiPlumeReading = getReadingMultiPlume(robotPose[0], robotPose[1], robotPose[2] - xp[i,2], AMatrix)

        yp[i] = GaussianSensorNB(robotPose[0], robotPose[1], xp[i,3], xp[i,0], xp[i,1], robotPose[2] - xp[i,2], xp[i,4], xp[i,5], xp[i,6], xp[i,7], False) - multiPlumeReading
    return yp

@njit
def calculateXhatNumbaFunction(xp, wp, NumOfParticles, pdf_std, chem_reading, robotCurrentPose, Ahat, stdL2NormMax):
    # 1. Measurement prediction
    yi = observerEquationNB(robotCurrentPose, xp, NumOfParticles, Ahat)

    # 2. Likelihood
    xStd     = np.std(xp[:,0])
    yStd     = np.std(xp[:,1])
    zStd     = np.std(xp[:,2])
    stdVec = np.array([xStd, yStd, zStd])
    stdL2Norm = np.linalg.norm(stdVec)
    if stdL2NormMax <= stdL2Norm:
        stdL2NormMax = stdL2Norm

    r = pdf_std * stdL2Norm / stdL2NormMax

    wp = wp * pdf(chem_reading, yi, r)

    # 3. Normalization
    wpSum = np.sum(wp)
    if wpSum == 0:
        # print("No solution")
        wp = np.ones(NumOfParticles) * 1/NumOfParticles
    else:
        wp = wp/wpSum

    # 4. Estimate plume
    Xha_t     = np.sum(wp*xp[:,0])
    Yha_t     = np.sum(wp*xp[:,1])
    Zha_t     = np.sum(wp*xp[:,2])
    Theta_hat = np.sum(wp*xp[:,3])
    Q_hat     = np.sum(wp*xp[:,4])
    V_hat     = np.sum(wp*xp[:,5])
    Dy_hat    = np.sum(wp*xp[:,6])
    Dz_hat    = np.sum(wp*xp[:,7])

    # gaussHatVec = np.squeeze(np.array([Xha_t, Yha_t, Zha_t, Theta_hat, Q_hat, V_hat, Dy_hat, Dz_hat]))

    gaussHatVec = np.array([Xha_t, Yha_t, Zha_t, Theta_hat, Q_hat, V_hat, Dy_hat, Dz_hat])

    return gaussHatVec, wp, stdL2NormMax, stdL2Norm

@njit
def resamplingOldNB(xp, wp, NumOfParticles):
    wc = np.cumsum(wp)


    uj = (1.0/NumOfParticles) * np.random.rand(1)[0] # random number from 0 to np^-1

    i = 0
    ind = np.zeros(NumOfParticles,dtype = np.uint16)
    for j in range(NumOfParticles):
        while(uj > wc[i]):
            i+=1
        ind[j] = i
        uj = uj + 1.0/NumOfParticles

    xp = xp[ind]
    wp = np.ones(NumOfParticles)* 1/NumOfParticles

    return xp, wp

@njit
def updateParticlesNB(xp, wp, NumOfParticles, noiseVec, randomStateBoundsVec, resample = True):
    # 5. Resampling

    # resampleCheck = 1/sum(self.wp**2)

    # if (resampleCheck <= NumOfParticles/2):
        # xp, wp = resampling(xp, wp, NumberOfParticlesToReset, randomStateBoundsVec, NumOfParticles)
    # self.resampling(self.xp, self.wp, self.NumberOfParticlesToReset, self.randomStateBoundsVec, self.NumOfParticles)
    if resample:
        # print("Resampled")
        # self.resampling()
        xp, wp = resamplingOldNB(xp, wp, NumOfParticles)

    # 6. Random process noise
    xNoise     = np.array([np.random.normal(0, noiseVec[0], NumOfParticles)]).T
    yNoise     = np.array([np.random.normal(0, noiseVec[1], NumOfParticles)]).T
    zNoise     = np.array([np.random.normal(0, noiseVec[2], NumOfParticles)]).T
    thetaNoise = np.array([np.random.normal(0, noiseVec[3], NumOfParticles)]).T
    QNoise     = np.array([np.random.normal(0, noiseVec[4], NumOfParticles)]).T
    VNoise     = np.array([np.random.normal(0, noiseVec[5], NumOfParticles)]).T
    DyNoise    = np.array([np.random.normal(0, noiseVec[6], NumOfParticles)]).T
    DzNoise    = np.array([np.random.normal(0, noiseVec[7], NumOfParticles)]).T

    noiseMatrix = np.concatenate((xNoise, yNoise, zNoise, thetaNoise, QNoise, VNoise, DyNoise, DzNoise), axis=1)

    xp = xp + noiseMatrix

    # 6.5 limit values
    for i in range(8):
        xp[:,i][xp[:,i]>randomStateBoundsVec[i,1]] = randomStateBoundsVec[i,1]
        xp[:,i][xp[:,i]<randomStateBoundsVec[i,0]] = randomStateBoundsVec[i,0]

    return xp, wp

# =============================================================================
# Classes
# =============================================================================


class ParticleFilter:
    def __init__(self, randomStateBoundsVec, noiseVec, pdf_std, NumOfParticles, NumberOfParticlesToReset):
        xp     = np.asmatrix(np.random.uniform(randomStateBoundsVec[0,0], randomStateBoundsVec[0,1], NumOfParticles))
        yp     = np.asmatrix(np.random.uniform(randomStateBoundsVec[1,0], randomStateBoundsVec[1,1], NumOfParticles))
        zp     = np.asmatrix(np.random.uniform(randomStateBoundsVec[2,0], randomStateBoundsVec[2,1], NumOfParticles))
        thetap = np.asmatrix(np.random.uniform(randomStateBoundsVec[3,0], randomStateBoundsVec[3,1], NumOfParticles))
        Qp     = np.asmatrix(np.random.uniform(randomStateBoundsVec[4,0], randomStateBoundsVec[4,1], NumOfParticles))
        vp     = np.asmatrix(np.random.uniform(randomStateBoundsVec[5,0], randomStateBoundsVec[5,1], NumOfParticles))
        Dyp    = np.asmatrix(np.random.uniform(randomStateBoundsVec[6,0], randomStateBoundsVec[6,1], NumOfParticles))
        Dzp    = np.asmatrix(np.random.uniform(randomStateBoundsVec[7,0], randomStateBoundsVec[7,1], NumOfParticles))

        self.randomStateBoundsVec = randomStateBoundsVec
        self.noiseVec = noiseVec

        self.xp = np.concatenate((xp.T, yp.T, zp.T, thetap.T, Qp.T, vp.T, Dyp.T, Dzp.T), axis=1)
        self.wp = np.ones(NumOfParticles) * 1/NumOfParticles

        xStd     = np.std(xp[:,0])
        yStd     = np.std(xp[:,1])
        thetaStd = np.std(xp[:,3])
        stdVec = np.array([xStd, yStd, thetaStd])
        self.stdL2NormMax = np.linalg.norm(stdVec)
        self.pdf_std = pdf_std
        self.NumOfParticles = NumOfParticles
        self.NumberOfParticlesToReset = NumberOfParticlesToReset

        # Inital Estimate plume
        Xha_t     = sum(self.wp*self.xp[:,0])
        Yha_t     = sum(self.wp*self.xp[:,1])
        Zha_t     = sum(self.wp*self.xp[:,2])
        Theta_hat = sum(self.wp*self.xp[:,3])
        Q_hat     = sum(self.wp*self.xp[:,4])
        V_hat     = sum(self.wp*self.xp[:,5])
        Dy_hat    = sum(self.wp*self.xp[:,6])
        Dz_hat    = sum(self.wp*self.xp[:,7])

        np.squeeze(np.array([Xha_t, Yha_t, Zha_t, Theta_hat, Q_hat, V_hat, Dy_hat, Dz_hat]))

    def getXhat(self):
        # Inital Estimate plume
        Xha_t     = sum(self.wp*self.xp[:,0])
        Yha_t     = sum(self.wp*self.xp[:,1])
        Zha_t     = sum(self.wp*self.xp[:,2])
        Theta_hat = sum(self.wp*self.xp[:,3])
        Q_hat     = sum(self.wp*self.xp[:,4])
        V_hat     = sum(self.wp*self.xp[:,5])
        Dy_hat    = sum(self.wp*self.xp[:,6])
        Dz_hat    = sum(self.wp*self.xp[:,7])

        return np.squeeze(np.array([Xha_t, Yha_t, Zha_t, Theta_hat, Q_hat, V_hat, Dy_hat, Dz_hat]))

    def calculateXhat(self, chem_reading, robotCurrentPose, multiPlume):
        # 1. Measurement prediction
        yi = self.observerEquation(robotCurrentPose, multiPlume)

        # 2. Likelihood
        xStd     = np.std(self.xp[:,0])
        yStd     = np.std(self.xp[:,1])
        thetaStd = np.std(self.xp[:,3])
        stdVec = np.array([xStd, yStd, thetaStd])
        self.stdL2Norm = np.linalg.norm(stdVec)
        if self.stdL2NormMax <= self.stdL2Norm:
            self.stdL2NormMax = self.stdL2Norm

        r = self.pdf_std * self.stdL2Norm / self.stdL2NormMax

        # print("Wp_max = ", max(self.wp))
        # self.wp = self.wp * norm.pdf(chem_reading, yi, r)
        self.wp = self.wp * pdf(chem_reading, yi, r)
        # print("Wp_max = ", max(self.wp))

        # 3. Normalization
        wpSum = sum(self.wp)
        if wpSum == 0:
            print("No solution")
            self.wp = np.ones(self.NumOfParticles) * 1/self.NumOfParticles
        else:
            self.wp = self.wp/wpSum

        # 4. Estimate plume
        Xha_t     = np.sum(self.wp*self.xp[:,0])
        Yha_t     = np.sum(self.wp*self.xp[:,1])
        Zha_t     = np.sum(self.wp*self.xp[:,2])
        Theta_hat = np.sum(self.wp*self.xp[:,3])
        Q_hat     = np.sum(self.wp*self.xp[:,4])
        V_hat     = np.sum(self.wp*self.xp[:,5])
        Dy_hat    = np.sum(self.wp*self.xp[:,6])
        Dz_hat    = np.sum(self.wp*self.xp[:,7])

        self.gaussHatVec = np.squeeze(np.array([Xha_t, Yha_t, Zha_t, Theta_hat, Q_hat, V_hat, Dy_hat, Dz_hat]))

        return self.gaussHatVec

    def calculateXhatNumba(self, chem_reading, robotCurrentPose, Ahat):
        # 1. Measurement prediction
        yi = self.observerEquationNumba(robotCurrentPose, Ahat)

        # 2. Likelihood
        xStd     = np.std(self.xp[:,0])
        yStd     = np.std(self.xp[:,1])
        zStd     = np.std(self.xp[:,2])
        stdVec = np.array([xStd, yStd, zStd])
        self.stdL2Norm = np.linalg.norm(stdVec)
        if self.stdL2NormMax <= self.stdL2Norm:
            self.stdL2NormMax = self.stdL2Norm

        r = self.pdf_std * self.stdL2Norm / self.stdL2NormMax

        # print("Wp_max = ", max(self.wp))
        # self.wp = self.wp * norm.pdf(chem_reading, yi, r)
        self.wp = self.wp * pdf(chem_reading, yi, r)
        # print("Wp_max = ", max(self.wp))

        # 3. Normalization
        wpSum = sum(self.wp)
        if wpSum == 0:
            print("No solution")
            self.wp = np.ones(self.NumOfParticles) * 1/self.NumOfParticles
        else:
            self.wp = self.wp/wpSum

        # 4. Estimate plume
        Xha_t     = np.sum(self.wp*self.xp[:,0])
        Yha_t     = np.sum(self.wp*self.xp[:,1])
        Zha_t     = np.sum(self.wp*self.xp[:,2])
        Theta_hat = np.sum(self.wp*self.xp[:,3])
        Q_hat     = np.sum(self.wp*self.xp[:,4])
        V_hat     = np.sum(self.wp*self.xp[:,5])
        Dy_hat    = np.sum(self.wp*self.xp[:,6])
        Dz_hat    = np.sum(self.wp*self.xp[:,7])

        self.gaussHatVec = np.squeeze(np.array([Xha_t, Yha_t, Zha_t, Theta_hat, Q_hat, V_hat, Dy_hat, Dz_hat]))

        return self.gaussHatVec

    def calculateXhatNumbaNew(self, chem_reading, robotCurrentPose, Ahat):

        self.gaussHatVec, self.wp, self.stdL2NormMax, self.stdL2Norm = calculateXhatNumbaFunction(self.xp, self.wp, self.NumOfParticles, self.pdf_std, chem_reading, robotCurrentPose, Ahat, self.stdL2NormMax)

        return self.gaussHatVec

    def updateParticles(self, resample = True):
        # 5. Resampling

        # resampleCheck = 1/sum(self.wp**2)

        # if (resampleCheck <= NumOfParticles/2):
            # xp, wp = resampling(xp, wp, NumberOfParticlesToReset, randomStateBoundsVec, NumOfParticles)
        # self.resampling(self.xp, self.wp, self.NumberOfParticlesToReset, self.randomStateBoundsVec, self.NumOfParticles)
        if resample:
            # print("Resampled")
            # self.resampling()
            # self.resamplingOld()
            self.xp, self.wp = resamplingOldNB(self.xp, self.wp, self.NumOfParticles)

        # 6. Random process noise
        xNoise     = np.array([np.random.normal(0, self.noiseVec[0], self.NumOfParticles)]).T
        yNoise     = np.array([np.random.normal(0, self.noiseVec[1], self.NumOfParticles)]).T
        zNoise     = np.array([np.random.normal(0, self.noiseVec[2], self.NumOfParticles)]).T
        thetaNoise = np.array([np.random.normal(0, self.noiseVec[3], self.NumOfParticles)]).T
        QNoise     = np.array([np.random.normal(0, self.noiseVec[4], self.NumOfParticles)]).T
        VNoise     = np.array([np.random.normal(0, self.noiseVec[5], self.NumOfParticles)]).T
        DyNoise    = np.array([np.random.normal(0, self.noiseVec[6], self.NumOfParticles)]).T
        DzNoise    = np.array([np.random.normal(0, self.noiseVec[7], self.NumOfParticles)]).T

        noiseMatrix = np.concatenate((xNoise, yNoise, zNoise, thetaNoise, QNoise, VNoise, DyNoise, DzNoise), axis=1)

        self.xp = self.xp + noiseMatrix

        # 6.5 limit values
        for i in range(8):
            self.xp[:,i][self.xp[:,i]>self.randomStateBoundsVec[i,1]] = self.randomStateBoundsVec[i,1]
            self.xp[:,i][self.xp[:,i]<self.randomStateBoundsVec[i,0]] = self.randomStateBoundsVec[i,0]

    def updateParticlesFromPastMeasurements(self, zVec, xVec, multiPlume):
        # Calculate weights vec

        # Multiply weights vec
        # 1. Measurement prediction
        for k in range(len(zVec)):
            yi = self.observerEquation(xVec[k], multiPlume)

            # 2. Likelihood
            # r = len(zVec) * 1/100 * self.pdf_std

            r = self.pdf_std/1000

            # print("Wp_max = ", max(self.wp))
            # self.wp = np.logaddexp(self.wp, norm.pdf(zVec[k], yi, r))
            self.wp = np.logaddexp(self.wp, pdf(zVec[k], yi, r))
            # print("Wp_max = ", max(self.wp))

        # 3. Normalization
        wpSum = sum(self.wp)
        if wpSum == 0:
            print("No solution from past measurments!")
            self.wp = np.ones(self.NumOfParticles) * 1/self.NumOfParticles
        else:
            self.wp = self.wp/wpSum

        self.resamplingOld() # updated particle from previous measurements

    def updateParticlesFromPastMeasurementsNumba(self, zVec, xVec, Ahat):
        # Calculate weights vec

        # Multiply weights vec
        # 1. Measurement prediction
        for k in range(len(zVec)):
            yi = self.observerEquationNumba(xVec[k], Ahat)

            # 2. Likelihood
            r = self.pdf_std
            self.wp = np.logaddexp(self.wp, pdf(zVec[k], yi, r))

            # r = self.pdf_std*len(zVec)/25
            # self.wp = self.wp * pdf(zVec[k], yi, r)


        # 3. Normalization
        wpSum = sum(self.wp)
        if wpSum == 0:
            print("No solution from past measurments!")
            self.wp = np.ones(self.NumOfParticles) * 1/self.NumOfParticles
        else:
            self.wp = self.wp/wpSum

        self.xp, self.wp = resamplingOldNB(self.xp, self.wp, self.NumOfParticles)

    def updateParticlesFromPastMeasurementsNumbaNew(self, zVec, xVec, Ahat):
        # Calculate weights vec

        # Multiply weights vec
        # 1. Measurement prediction
        for k in range(len(zVec)):
            yi = self.observerEquationNumba(xVec[k], Ahat)

            # 2. Likelihood
            # r = self.pdf_std
            # self.wp = np.logaddexp(self.wp, pdf(zVec[k], yi, r))

            # r = self.pdf_std*len(zVec)/25
            r = self.pdf_std*len(zVec)/50
            self.wp = self.wp * pdf(zVec[k], yi, r)

            # 3. Normalization
            wpSum = sum(self.wp)
            if wpSum == 0:
                print("No solution from past measurments!")
                self.wp = np.ones(self.NumOfParticles) * 1/self.NumOfParticles
            else:
                self.wp = self.wp/wpSum

        self.xp, self.wp = resamplingOldNB(self.xp, self.wp, self.NumOfParticles)

    def resetParticles(self):
        xp     = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[0,0], self.randomStateBoundsVec[0,1], self.NumOfParticles))
        yp     = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[1,0], self.randomStateBoundsVec[1,1], self.NumOfParticles))
        zp     = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[2,0], self.randomStateBoundsVec[2,1], self.NumOfParticles))
        thetap = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[3,0], self.randomStateBoundsVec[3,1], self.NumOfParticles))
        Qp     = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[4,0], self.randomStateBoundsVec[4,1], self.NumOfParticles))
        vp     = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[5,0], self.randomStateBoundsVec[5,1], self.NumOfParticles))
        Dyp    = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[6,0], self.randomStateBoundsVec[6,1], self.NumOfParticles))
        Dzp    = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[7,0], self.randomStateBoundsVec[7,1], self.NumOfParticles))

        self.xp = np.concatenate((xp.T, yp.T, zp.T, thetap.T, Qp.T, vp.T, Dyp.T, Dzp.T), axis=1)
        self.wp = np.ones(self.NumOfParticles) * 1/self.NumOfParticles

    def gaussFunc(self, xFunc, yFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
        con = (QFunc/(4 * pi * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * ((yFunc**2)/DyFunc + (zFunc**2)/DzFunc))
        return con * 1000 # convert from kg/m^3 to ppm

    def getReading(self, xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
        # Rotate frame

        Stheta = sin(thetaFunc)
        Ctheta = cos(thetaFunc)

        xRobotRotated = (Ctheta  * xRobotDef + Stheta * yRobotDef + -Ctheta * xPlumeFunc - Stheta * yPlumeFunc)
        yRobotRotated = (-Stheta * xRobotDef + Ctheta * yRobotDef +  Stheta * xPlumeFunc - Ctheta * yPlumeFunc)

        if xRobotRotated <= 0:
            reading = 0
        else:
            reading = self.gaussFunc(xRobotRotated,yRobotRotated,zFunc,QFunc,vFunc,DyFunc,DzFunc)

        return reading

    def observerEquation(self, robotPose, multiPlume):
        yp = np.zeros(self.NumOfParticles)
        for i in range(self.NumOfParticles):
                                 # 0    1   2     3    4    5   6    7
            # xp = np.concatenate((xp, yp, zp, thetap, Qp, vp, Dyp, Dzp), axis=1)
            # (xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc)
            multiPlumeReading = multiPlume.getReading(robotPose[0], robotPose[1], robotPose[2] - self.xp[i,2])
            # multiPlumeReading = 0
            yp[i] = self.getReading(robotPose[0], robotPose[1], self.xp[i,3], self.xp[i,0], self.xp[i,1], robotPose[2] - self.xp[i,2], self.xp[i,4], self.xp[i,5], self.xp[i,6], self.xp[i,7]) - multiPlumeReading
        return yp

    def observerEquationNumba(self, robotPose, Ahat):
        return observerEquationNB(robotPose, self.xp, self.NumOfParticles, Ahat)

    def resampling(self):
        #               0          1          2              3             4         5          6           7
        # np.array([[X_l, X_u],[Y_l, Y_u],[Z_l, Z_u],[Theta_l,Theta_u],[Q_l,Q_u],[V_l,V_u],[Dy_l,Dy_u],[Dz_l, Dz_u]])

        wpIndex = np.argsort(self.wp)
        # print(self.NumberOfParticlesToReset)
        wpHigh = self.wp[wpIndex[self.NumberOfParticlesToReset:]]
        xpHigh = self.xp[wpIndex[self.NumberOfParticlesToReset:]]
        # print(wpHigh.shape)
        # Resample low weighted particles
        xpLow     = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[0,0], self.randomStateBoundsVec[0,1], self.NumberOfParticlesToReset))
        ypLow     = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[1,0], self.randomStateBoundsVec[1,1], self.NumberOfParticlesToReset))
        zpLow     = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[2,0], self.randomStateBoundsVec[2,1], self.NumberOfParticlesToReset))
        thetapLow = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[3,0], self.randomStateBoundsVec[3,1], self.NumberOfParticlesToReset))
        QpLow     = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[4,0], self.randomStateBoundsVec[4,1], self.NumberOfParticlesToReset))
        vpLow     = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[5,0], self.randomStateBoundsVec[5,1], self.NumberOfParticlesToReset))
        DypLow    = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[6,0], self.randomStateBoundsVec[6,1], self.NumberOfParticlesToReset))
        DzpLow    = np.asmatrix(np.random.uniform(self.randomStateBoundsVec[7,0], self.randomStateBoundsVec[7,1], self.NumberOfParticlesToReset))

        xpNewLow = np.concatenate((xpLow.T, ypLow.T, zpLow.T, thetapLow.T, QpLow.T, vpLow.T, DypLow.T, DzpLow.T), axis=1)

        # Resample high weighted particles
        xpNewHigh = self.pinWheelResampler(xpHigh, wpHigh/sum(wpHigh));

        t1 = xpNewHigh.shape
        t2 = xpNewLow.shape

        self.xp = np.concatenate((xpNewLow, xpNewHigh), axis=0)
        self.wp = np.ones(self.NumOfParticles)* 1/self.NumOfParticles

    def resamplingOld(self):
        wc = np.cumsum(self.wp)

        uj = (1.0/self.NumOfParticles) * np.random.rand(1)[0] # random number from 0 to np^-1

        i = 0
        ind = np.zeros(self.NumOfParticles,dtype = int)
        for j in range(self.NumOfParticles):
            while(uj > wc[i]):
                i+=1
            ind[j] = i
            uj = uj + 1.0/self.NumOfParticles

        self.xp = self.xp[ind]
        self.wp = np.ones(self.NumOfParticles)* 1/self.NumOfParticles

    def pinWheelResampler(self, xp, wp):
        wc = np.cumsum(wp)

        uj = (1.0/(self.NumOfParticles - self.NumberOfParticlesToReset)) * np.random.rand(1)[0] # random number from 0 to np^-1

        i = 0
        ind = np.zeros((self.NumOfParticles - self.NumberOfParticlesToReset),dtype = int)

        for j in range((self.NumOfParticles - self.NumberOfParticlesToReset)):
            while(uj > wc[i]):
                i+=1
            ind[j] = i
            uj = uj + 1.0/(self.NumOfParticles - self.NumberOfParticlesToReset)

        xNew = xp[ind]

        return xNew

    def clusterParticles(self, n_clusters = 100):
        # particleArray = np.array([np.asarray(self.xp[:,0]), np.asarray(self.xp[:,1]), np.asarray(self.xp[:,2]), np.asarray(self.xp[:,3]), np.asarray(self.xp[:,4]), np.asarray(self.xp[:,5]), np.asarray(self.xp[:,6]), np.asarray(self.xp[:,7]), self.wp]).T

        particleArray = np.c_[self.xp, self.wp]

        X = particleArray[:,0:2]
        k_means = MiniBatchKMeans(init='k-means++', n_clusters=n_clusters, n_init=10, max_no_improvement=10, batch_size=4096)
        k_means.fit(X) # Fit particles

        condensedParticles = np.zeros((n_clusters, particleArray.shape[1]))
        k_means_cluster_centers = k_means.cluster_centers_
        k_means_labels = pairwise_distances_argmin(X, k_means_cluster_centers)

        for k in range(n_clusters):
            my_members = k_means_labels == k
            groupedParticles = particleArray[my_members]
            condensedParticles[k,:] = np.mean(groupedParticles, axis=0)

        condensedParticles[:,-1] = condensedParticles[:,-1]/sum(condensedParticles[:,-1]) # normalize weights

        self.xpCompressed = condensedParticles[:,:-1]
        self.wpCompressed = condensedParticles[:,-1]

        # return condensedParticles
