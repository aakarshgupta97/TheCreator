#!/usr/bin/env python
"""
Arduino communication function for TheCreator
Authors: Ahad Rauf
"""

import sys
import time
import serial

PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
ser = serial.Serial(PORT, BAUD_RATE)

def inflate():
	print('Inflating')
	for _ in range(3):	
		ser.write(b'1')
		time.sleep(0.33)

def deflate():
	print('Deflating')
	for _ in range(3):	
		ser.write(b'0')
		time.sleep(0.33)

if __name__ == "__main__":
	"""
	You can test this function by calling 
	>>> python arduino_communcation.py 1
	Inflating
	Finished. Praise me now.
	>>> python arduino_communcation.py 0
	Deflating
	Finished. Praise me now. 
	"""
	# Use this format if you want to get command line parameters
	# if len(sys.argv) == 3:
	# 	x, y = *sys.argv[1:]
	if len(sys.argv) == 2:
		val = int(sys.argv[1])
		if val == 0:
			deflate()
		else:
			inflate()

	print('Finished. Praise me now.')
