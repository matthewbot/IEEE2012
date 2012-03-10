#!/usr/bin/python

import serial
from time import time
import sys

# Constants
wantID			= 0xF0
gotID			= 0xF5
beginReading	= 0xFF
boardCount		= 0
assignID		= 0xF1
makeReading		= 0xF6

class SensorBoard:
	assigned 	= False
	ID			= 0
	trigger 	= 0x0
	data		= chr(0xBE) + chr(0xEF)

	def init(self):
		while (not self.assigned):
			xbee.write(chr(wantID))
			print "Requesting ID"
			byte = read()
			if byte is not None:
				self.ID = byte - assignID
				if (self.ID >= 0 and self.ID <= 3):
					self.data += chr(self.ID)
					print "Got ID: " + hex(self.ID + assignID)
					print "Board Number: " + str(self.ID)
					print "Trigger: " + hex(self.ID + makeReading)
					self.assigned = True
					xbee.write(chr(gotID))

	def run(self, byte):
		if (byte != ""):
			if (byte > 0xF0):
				print "Board ", self.ID
				print "Heard: " + hex(byte)
			if (byte == assignID + self.ID):
				xbee.write(chr(gotID))
				print "Was assign ID."
			elif (byte == self.ID + makeReading):
				print "Was Trigger."
				# Got signal to make reading and report
				xbee.write(chr(beginReading))
				xbee.write(chr(len(self.data)))
				checksum = beginReading + len(self.data)
				for ch in self.data:
					xbee.write(ch)
					checksum += ord(ch)
				checksum = checksum % 256
				xbee.write(chr(checksum))
				print "Wrote:\n" + hex(beginReading) + "\n" + hex(len(self.data))
				for ch in self.data:
					print hex(ord(ch))
				print "Checksum: " + hex(checksum)


def read():
	timer = 0
	while (timer < 1):
		start = time()
		byte = xbee.read()
		if (byte is not ""):
			if (ord(byte) > 0xF0):
				return ord(byte)
		timer = timer + time() - start
	return None

# MAIN

xbee = serial.Serial('/dev/ttyUSB0', 9600, timeout = 1)
xbee.flushInput()
xbee.flushOutput()
boards = []

if len(sys.argv) > 1:		# Initializing Boards
	boardCount = int(sys.argv[1])
else:
	print "Restart with number of boards as an argument, stupid!"

for i in range(boardCount):
	boards.append(SensorBoard())

for num, board in enumerate(boards):
	print "Initializing new board:", num
	board.init()

xbee.timeout = None;

while (True):			# Running Boards
	gotByte = ord(xbee.read())
	
	for board in boards:
		board.run(gotByte)

