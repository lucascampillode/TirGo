#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
from __future__ import division
import numpy


class gauss_array:

    def __init__ (self, mX=0.0, mY=0.0, mT=0.0, vX=0.0, vY=0.0, vT=0.0, th1=0.001, th2=0.0001):
        # The variances
        self.vX = vX
        self.vY = vY
        self.vT = vT

        # The means
        self.mX = mX
        self.mY = mY
        self.mT = mT

        # The two thresholds
        self.th1 = th1
        self.th2 = th2

        self.anomalySt = []

    def train_anomaly_funcs (self, arrXs, arrYs, arrTs):
        self.mX = numpy.mean (arrXs)
        self.mY = numpy.mean (arrYs)
        self.mT = numpy.mean (arrTs)

        self.vX = numpy.var (arrXs, axis=0, ddof=1)
        self.vY = numpy.var (arrYs, axis=0, ddof=1)
        self.vT = numpy.var (arrTs, axis=0, ddof=1)

        print("Gaussian values computed with %d values:" % len (arrTs))
        print("\t in X defined by (%f,%f)" % (self.mX,self.vX))
        print("\t in Y defined by (%f,%f)" % (self.mY,self.vY))
        print("\t in Theta defined by (%f,%f)" % (self.mT,self.vT))
        print(" ======================= ")

        n = len (arrXs)
        if n>1:
            res = self.anomalous_calc (arrXs[n-1], arrYs[n-1], arrTs[n-1])
            self.anomalySt += (res,)
            anomalyMean = numpy.mean(self.anomalySt)
            anomalyVar = numpy.var (self.anomalySt)
            print("\tAnomaly value : %f" % res)
            print("\tAnomaly gaussian (%f,%f)"% (anomalyMean, anomalyVar))
            print(" ======================= ")

            self.th1 = anomalyMean - numpy.sqrt (anomalyVar)
            self.th2 = self.th1 / 10
            print("\t thresholds set to (%f,%f)" % (self.th1, self.th2))
            print(" ======================= ")

            self.save_training_params ()

    def save_training_params (self):
        with open('anomaly_gaussians2.yaml', 'w') as f:
            f.write('loc_mes_mX: ' + str(self.mX) + '\n')
            f.write('loc_mes_vX: ' + str(self.vX) + '\n')
            f.write('loc_mes_mY: ' + str(self.mY) + '\n')
            f.write('loc_mes_vY: ' + str(self.vY) + '\n')
            f.write('loc_mes_mT: ' + str(self.mT) + '\n')
            f.write('loc_mes_vT: ' + str(self.vT) + '\n')

            f.write('loc_mes_th1: '+str(self.th1) + '\n')
            f.write('loc_mes_th2: '+str(self.th2) + '\n')

    def anomalous_calc (self, covarX, covarY, covarT):
        res = numpy.exp(-(covarX - self.mX)**2/(2*self.vX))
        res = res * numpy.exp(-(covarY - self.mY)**2/(2*self.vY))
        return res * numpy.exp(-(covarT - self.mT)**2/(2*self.vT))

    def is_anomalous (self, covarX, covarY, covarT):
        res = self.anomalous_calc (covarX, covarY, covarT)

        #if res < self.th1:
        if res < self.th2:
            return "ANOMALOUS"
        #   else:
        #        return "DOUBTFUL"
        return "NORMAL"
