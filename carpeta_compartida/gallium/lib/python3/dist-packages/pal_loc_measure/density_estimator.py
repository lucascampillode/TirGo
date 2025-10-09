#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
from __future__ import division
import numpy as np

class DensityEstimator:
    """
    Density Estimator for Anomaly Detection

    See: http://dnene.bitbucket.org/docs/mlclass-notes/lecture16.html
    """

    def __init__ (self, eps=0.001, ddof=1):
        self._eps = eps
        self._ddof = ddof

        self._p = []

    def train(self, x):
        """
        Train a density estimation probabily for the given training set
        `x`.
        """
        self._mean = x.mean(axis=1)
        self._var  = x.var(axis=1, ddof=self._ddof)
        print("mean = ", self._mean)
        print("var  = ", self._var)

        # Automatic threshold estimation
        p = self._density(x)
        self._p.append(p)

        mean = np.mean(self.anomalySt)
        var  = np.var (self.anomalySt)

        self._eps= mean - np.sqrt(var)
        print("eps = ", self._eps)


    def save(self, filename):
        """
        Save the trained mean and variances.
        """
        f = open(filename, 'w')

        f.write('mean: ' + str(self._mean))
        f.write('var : ' + str(self._var))
        f.write('eps : ' + str(self._eps))

        f.close()

    def _density(self, x):
        """
        Density estimation probability for independent random variables `x`.
        """
        return np.prod([self._gaussain(x[i]) for i in range(x.shape[0])])

    def _gaussian(self, x):
        """
        Returns the probability of a random variables distributed as a
        Normal/Gaussian distribution with the mean and variance learnt.
        """
        return np.exp(-(x - self._mean)**2 / (2.0*self._var)) / np.sqrt(2.0 * np.pi * self._var)

    def is_anomaly(self, x):
        """
        Density estimation check.
        """
        return self._density(x) > self._eps

