#!/usr/bin/env python
import herkulex
from herkulex import servo

#connect to the serial port
herkulex.connect("/dev/ttyUSB0", 115200)

#scan for servos, it returns a tuple with servo id & model number
servos = herkulex.scan_servos()

print servos