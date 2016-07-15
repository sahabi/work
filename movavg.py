#!/usr/bin/env python

import numpy as np

class MovAvg():
	def __init__(self, length):
		self.length = length
		self.index = 0
		self.reset()

	def step(self, value):
		self.buff[self.index] = value
		self.index = (self.index + 1) % self.length
		return np.mean(self.buff)

	def reset(self):
		self.buff = np.zeros(self.length)

if __name__ == '__main__':
	avg = MovAvg(4)
	for i in range(8):
		print(avg.step(2))