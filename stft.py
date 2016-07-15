#!/usr/bin/env python

import numpy as np
from scipy.signal import lfilter, get_window, butter

class STFT():
    def __init__(self, fs, frame, stride, channel_count = 1):
        # Signal properties
        self.fs = fs
        self.T = 1.0/self.fs
        self.N = int(frame*fs)        
        self.N_stride = int(stride*fs)
        self.N_old = self.N - self.N_stride

        self.removing_dc = False
        self.filters = []
        
        # Initialize buffer
        self.index = 0
        self.buff = np.empty((self.N, channel_count))
        
        # Initialize FFT stuff
        self.freq_bins = np.fft.rfftfreq(self.N, self.T)

    def remove_dc(self):
        self.removing_dc = True

    def bandpass(self, lowcut, highcut, order=4):
        nyq = 0.5 * self.fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = butter(order, [low, high], btype = 'bandpass')
        self.filters.append((b,a))

    def window(self, window):
        self.window = get_window(window, self.N)
        self.windowing = True

    def ingestSample(self, sample):
        self.buff[self.index] = sample
        self.index += 1

        if self.index == self.N:

            X = self.processFrame(self.buff)

            self.buff = np.roll(self.buff, shift = self.N_old, axis = 0)
            self.index = self.N_old

            return X

        return None

    def processFrame(self, x):
        if self.removing_dc:
            x = x - np.mean(x, axis = 0)

        for b,a in self.filters:
            x = lfilter(b, a, x, axis = 0)

        if self.windowing:
            x = x*self.window[:, np.newaxis]

        return np.abs(np.fft.rfft(x, axis = 0))*2.0/self.N

    def processBlock(self, block, block_time = None):
        periods = range(0, len(block)-self.N, self.N_stride)
        X = np.array([self.processFrame(block[i:i+self.N, :]) for i in periods])
        if block_time is not None:
            return X, block_time[range(self.N, len(block), self.N_stride)]
        else:
            return X, None

