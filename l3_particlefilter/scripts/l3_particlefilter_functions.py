# -*- coding: utf-8 -*-
"""
Created on Mon Aug 30 13:47:19 2021

@author: smcke
"""
import numpy as np

# =============================================================================
# Functions
# =============================================================================
# create normal distribution of x given mu and sigma
def normalDist(x, mu, sigma):
    hold1 = (1/(sigma*np.sqrt(2*np.pi)))
    hold2 = np.exp((-np.square(x-mu))/(2*np.square(sigma)))
    prob_density = np.multiply(hold1, hold2)
    return prob_density



# find error and likelihood model
def likeModel(meas, partMeas, w, sigma):
    # meas = actual measurement
    # partMeas = particle measurement 
    # error calculation
    err = np.squeeze(np.subtract(meas, partMeas))
    mu = 0.0
    prob = np.squeeze(normalDist(err, mu, sigma))
    
    # create likelihood model by multiplying by weights and normalize
    w = np.multiply(w, prob)
    w = np.divide(w, np.sum(w))
    
    return w


def resample2D(w, xp, yp):
    # find number of particles used in this iteration
    Np = len(w)
    
    # create random probability vector of the same size as w
    u = np.squeeze(np.random.rand(Np, 1))
    
    # find cumulative likelihood model (create bins)
    wc = np.cumsum(w)
    # normalize likelihood model
    wc = wc/wc[-1]
    
    #concatenate u with wc to add randomness to likelihood model
    j = np.concatenate([u, wc])
    
    #sort array (debugging purposes)
    #dum = np.sort(j)
    # indices related to sorted array
    ind1 = np.argsort(j)
    
    # find most likely particles
        # (Np-1) because index starts at 0
    hold = np.where(ind1<(Np))
    # convert from tuple output to array
    ind2 = np.asarray(hold)
    
    
    # create array to shift back to original size
    shift = list(range(0, Np))
    
    #find indices correlated
    ind = ind2 - shift
    ind = np.squeeze(ind)
    # ind = np.transpose(ind)
    
    xp = np.squeeze(np.array(xp)[ind.astype(int)])
    yp = np.squeeze(np.array(yp)[ind.astype(int)])
    
    w = np.squeeze(np.ones((Np,1))/Np)
    
    return xp, yp, w, 

