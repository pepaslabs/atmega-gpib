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

ser.write("++addr 17\n")
print ser.readline().rstrip()

ser.write("++stream\n")
while True:
    print ser.readline().rstrip()
