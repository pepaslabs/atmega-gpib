#!/usr/bin/env python

import serial
import sys

ser = serial.Serial(
    port = sys.argv[1],
    baudrate = 9600,
    bytesize=serial.EIGHTBITS,
    stopbits = serial.STOPBITS_ONE,
    parity = serial.PARITY_NONE,
    timeout = 10
)

ser.write("++ver\n")
ser.flush()
print ser.readline()
