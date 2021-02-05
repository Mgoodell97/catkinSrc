#!/usr/bin/env python
import numpy as np


class MAFAngles:
    # filter for angles with magnitudes
    def __init__(self, n):
        self.vals = []
        self.n = n
        self.ii = 0
        self.full = 0
        self.sum = np.array([0.0,0.0]) # sum pof x and y directions

    def add(self,x,y):
        if not self.full:
            print(np.array([x,y]))
            self.sum += np.array([x,y])
            self.vals.append(np.array([x,y]))
        else:
            self.sum -= self.vals[self.ii]
            self.sum += np.array([x,y])
            self.vals[self.ii] = np.array([x,y])
        self.ii += 1
        if self.ii == self.n:
            self.full = 1
            self.ii = 0
        return self.ave()

    def ave(self):
        if self.full:
            temp = self.sum/float(self.n)
            return temp.tolist()
        else:
            temp = self.sum/float(self.ii)
            return temp.tolist()

    def clear(self):
        self.vals = []
        self.ii = 0
        self.full = 0
        self.sum = np.array([0.0,0.0]) # sum pof x and y directions

class MAF:
    # Moving average filter
    # initialize with array length
    # add functions adds number to array

    def __init__(self, n):
        self.vals = []
        self.n = n
        self.ii = 0
        self.full = 0
        self.sum = 0

    def add(self, x):
        if not self.full:
            self.sum += x
            self.vals += [x]
        else:
            self.sum -= self.vals[self.ii]
            self.sum += x
            self.vals[self.ii] = x
        self.ii += 1
        if self.ii == self.n:
            self.full = 1
            self.ii = 0
        return self.ave()

    def ave(self):
        if self.full:
            return self.sum/float(self.n)
        else:
            return self.sum/float(self.ii)

    def clear(self):
        self.vals = []
        self.ii = 0
        self.full = 0
        self.sum = 0


class MAFList:
    # Moving average filter
    # initialize with array length
    # add functions adds number to array

    def __init__(self, n):
        self.vals = None
        self.n = n
        self.ii = 0
        self.full = 0
        self.sum = 0

    def add(self, x):
        x = np.matrix(x)
        if not self.full:
            if self.ii == 0:
                self.vals = x
            else:
                self.vals = np.concatenate((self.vals, x), axis=0)
        else:
            self.vals[self.ii, :] = x
        self.ii += 1
        if self.ii == self.n:
            self.full = 1
            self.ii = 0

    def ave(self):
        temp = self.vals.mean(0)
        return temp.tolist()[0]


    def clear(self):
        self.vals = []
        self.ii = 0
        self.full = 0
        self.sum = 0
