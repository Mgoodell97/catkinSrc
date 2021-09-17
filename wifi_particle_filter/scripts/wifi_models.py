#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""

Created on Wed Jun 16 16:03:18 2021

@author: nathan
"""
import numpy as np

def dBm2mV(dBm):
    mV = 10**(dBm/10)
    return mV

def find_dist(point1, point2):
    x = point1[0]-point2[0]
    y = point1[1]-point2[1]
    dist = np.sqrt(x**2+y**2)
    return dist

def ITU_indoor(measurement_location, transmitter_location, transmitter_Mhz, transmitter_dBm, floor_term, N, const_term):
#    floor_term = 8
#    N = 28
#    const_term = -27.55
    Tran_mW = dBm2mV(transmitter_dBm)
    dist = find_dist(measurement_location,transmitter_location)
    
    freq_term = 20*(np.log(transmitter_Mhz)/np.log(10)) # megahertz
    dist_term = N*(np.log(dist)/np.log(10))
    L = freq_term+dist_term+floor_term+const_term
    L = 10**(L/10)
    reading = Tran_mW/L # reading in mV
    reading = 10*np.log(reading)/np.log(10) # reading in dBm
    return reading

