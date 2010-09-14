# -*- coding: utf-8 -*-
from PyQt4 import QtCore, QtGui
from ..Node import Node
from scipy.signal import detrend
from scipy.ndimage import median_filter, gaussian_filter
from SignalProxy import *
from metaarray import *
import functions
from common import *
import numpy as np


class Downsample(CtrlNode):
    """Downsample by averaging samples together."""
    nodeName = 'Downsample'
    uiTemplate = [
        ('n', 'intSpin', {'min': 1, 'max': 1000000})
    ]
    
    def processData(self, data):
        return functions.downsample(data, self.ctrls['n'].value(), axis=0)


class Subsample(CtrlNode):
    """Downsample by selecting every Nth sample."""
    nodeName = 'Subsample'
    uiTemplate = [
        ('n', 'intSpin', {'min': 1, 'max': 1000000})
    ]
    
    def processData(self, data):
        return data[::self.ctrls['n'].value()]


class Bessel(CtrlNode):
    """Bessel filter. Input data must have time values."""
    nodeName = 'BesselFilter'
    uiTemplate = [
        ('band', 'combo', {'values': ['lowpass', 'highpass'], 'index': 0}),
        ('cutoff', 'spin', {'value': 1000., 'step': 1, 'dec': True, 'range': [0.0, None], 'suffix': 'Hz', 'siPrefix': True}),
        ('order', 'intSpin', {'value': 4, 'min': 1, 'max': 16}),
        ('bidir', 'check', {'checked': True})
    ]
    
    def processData(self, data):
        s = self.stateGroup.state()
        if s['band'] == 'lowpass':
            mode = 'low'
        else:
            mode = 'high'
        return functions.besselFilter(data, bidir=s['bidir'], btype=mode, cutoff=s['cutoff'], order=s['order'])


class Butterworth(CtrlNode):
    """Butterworth filter"""
    nodeName = 'ButterworthFilter'
    uiTemplate = [
        ('band', 'combo', {'values': ['lowpass', 'highpass'], 'index': 0}),
        ('wPass', 'spin', {'value': 1000., 'step': 1, 'dec': True, 'range': [0.0, None], 'suffix': 'Hz', 'siPrefix': True}),
        ('wStop', 'spin', {'value': 2000., 'step': 1, 'dec': True, 'range': [0.0, None], 'suffix': 'Hz', 'siPrefix': True}),
        ('gPass', 'spin', {'value': 2.0, 'step': 1, 'dec': True, 'range': [0.0, None], 'suffix': 'dB', 'siPrefix': True}),
        ('gStop', 'spin', {'value': 20.0, 'step': 1, 'dec': True, 'range': [0.0, None], 'suffix': 'dB', 'siPrefix': True}),
        ('bidir', 'check', {'checked': True})
    ]
    
    def processData(self, data):
        s = self.stateGroup.state()
        if s['band'] == 'lowpass':
            mode = 'low'
        else:
            mode = 'high'
        ret = functions.butterworthFilter(data, bidir=s['bidir'], btype=mode, wPass=s['wPass'], wStop=s['wStop'], gPass=s['gPass'], gStop=s['gStop'])
        return ret


class Mean(CtrlNode):
    """Filters data by taking the mean of a sliding window"""
    nodeName = 'MeanFilter'
    uiTemplate = [
        ('n', 'intSpin', {'min': 1, 'max': 1000000})
    ]
    
    @metaArrayWrapper
    def processData(self, data):
        n = self.ctrls['n'].value()
        return functions.rollingSum(data, n) / n


class Median(CtrlNode):
    """Filters data by taking the median of a sliding window"""
    nodeName = 'MedianFilter'
    uiTemplate = [
        ('n', 'intSpin', {'min': 1, 'max': 1000000})
    ]
    
    @metaArrayWrapper
    def processData(self, data):
        return median_filter(data, self.ctrls['n'].value())


class Denoise(CtrlNode):
    """Removes anomalous spikes from data, replacing with nearby values"""
    nodeName = 'DenoiseFilter'
    uiTemplate = [
        ('radius', 'intSpin', {'value': 2, 'min': 0, 'max': 1000000}),
        ('threshold', 'doubleSpin', {'value': 4.0, 'min': 0, 'max': 1000})
    ]
    
    def processData(self, data):
        #print "DENOISE"
        s = self.stateGroup.state()
        return functions.denoise(data, **s)


class Gaussian(CtrlNode):
    """Gaussian smoothing filter."""
    nodeName = 'GaussianFilter'
    uiTemplate = [
        ('sigma', 'doubleSpin', {'min': 0, 'max': 1000000})
    ]
    
    @metaArrayWrapper
    def processData(self, data):
        return gaussian_filter(data, self.ctrls['sigma'].value())


class Derivative(CtrlNode):
    """Returns the pointwise derivative of the input"""
    nodeName = 'DerivativeFilter'
    
    def processData(self, data):
        if isinstance(data, MetaArray):
            info = data.infoCopy()
            if 'values' in info[0]:
                info[0]['values'] = info[0]['values'][:-1]
            return MetaArray(data[1:] - data[:-1], info=info)
        else:
            return data[1:] - data[:-1]


class Integral(CtrlNode):
    """Returns the pointwise integral of the input"""
    nodeName = 'IntegralFilter'
    
    @metaArrayWrapper
    def processData(self, data):
        data[1:] += data[:-1]
        return data


class Detrend(CtrlNode):
    """Removes linear trend from the data"""
    nodeName = 'DetrendFilter'
    
    @metaArrayWrapper
    def processData(self, data):
        return detrend(data)


class AdaptiveDetrend(CtrlNode):
    """Removes baseline from data, ignoring anomalous events"""
    nodeName = 'AdaptiveDetrend'
    uiTemplate = [
        ('threshold', 'doubleSpin', {'value': 3.0, 'min': 0, 'max': 1000000})
    ]
    
    def processData(self, data):
        return functions.adaptiveDetrend(data, threshold=self.ctrls['threshold'].value())

class HistogramDetrend(CtrlNode):
    """Removes baseline from data by computing mode (from histogram) of beginning and end of data."""
    nodeName = 'HistogramDetrend'
    uiTemplate = [
        ('windowSize', 'intSpin', {'value': 500, 'min': 10, 'max': 1000000}),
        ('numBins', 'intSpin', {'value': 50, 'min': 3, 'max': 1000000})
    ]
    
    def processData(self, data):
        ws = self.ctrls['windowSize'].value()
        bn = self.ctrls['numBins'].value()
        return functions.histogramDetrend(data, window=ws, bins=bn)


class ExpDeconvolve(CtrlNode):
    """Exponential deconvolution filter."""
    nodeName = 'ExpDeconvolve'
    uiTemplate = [
        ('tau', 'spin', {'value': 10e-3, 'step': 1, 'minStep': 100e-6, 'dec': True, 'range': [0.0, None], 'suffix': 's', 'siPrefix': True})
    ]
    
    def processData(self, data):
        tau = self.ctrls['tau'].value()
        return functions.expDeconvolve(data, tau)
        #dt = 1
        #if isinstance(data, MetaArray):
            #dt = data.xvals(0)[1] - data.xvals(0)[0]
        #d = data[:-1] + (self.ctrls['tau'].value() / dt) * (data[1:] - data[:-1])
        #if isinstance(data, MetaArray):
            #info = data.infoCopy()
            #if 'values' in info[0]:
                #info[0]['values'] = info[0]['values'][:-1]
            #return MetaArray(d, info=info)
        #else:
            #return d

class ExpReconvolve(CtrlNode):
    """Exponential reconvolution filter. Only works with MetaArrays that were previously deconvolved."""
    nodeName = 'ExpReconvolve'
    #uiTemplate = [
        #('tau', 'spin', {'value': 10e-3, 'step': 1, 'minStep': 100e-6, 'dec': True, 'range': [0.0, None], 'suffix': 's', 'siPrefix': True})
    #]
    
    def processData(self, data):
        return functions.expReconvolve(data)

class Tauiness(CtrlNode):
    """Sliding-window exponential fit"""
    nodeName = 'Tauiness'
    uiTemplate = [
        ('window', 'intSpin', {'value': 100, 'min': 3, 'max': 1000000}),
        ('skip', 'intSpin', {'value': 10, 'min': 0, 'max': 10000000})
    ]
    
    def processData(self, data):
        return functions.tauiness(data, self.ctrls['window'].value(), self.ctrls['skip'].value())
        
        
    
    