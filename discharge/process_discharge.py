#!/usr/bin/python

from optparse import OptionParser
import sys

parser = OptionParser()

(options,args) = parser.parse_args()

if len(args) < 1:
	print "Need to give a file name"
	sys.exit(1)

datafile = open(args[0])

lastvoltage_val = 0
lastvoltage_time = 0

lastbsoc_val = 0
lastbsoc_time = 0

lastcurrent_val = 0
lastcurrent_time = 0

for line in datafile:
	tokens = line.strip().split()

	#print tokens[0]
	if len(tokens) < 1:
		continue

	try:

		if tokens[0] != "C:":
			continue

		#print tokens

		node = int(tokens[3])
		chan = int(tokens[4])
		val  = int(tokens[5])
		chantime = int(tokens[6])

		if node is not 29:
			continue

		# Current is channel 0
		if chan is 0:
			lastcurrent_val = val
			lastcurrent_time = chantime

    	# Voltage is channel 1
		elif chan is 1:
			lastvoltage_val = val
			lastvoltage_time = chantime

		elif chan is 2:
			lastbsoc_val = val
			lastbsoc_time = chantime

		else:
			continue

		if (lastbsoc_time == lastcurrent_time) and (lastcurrent_time == lastvoltage_time):
			print "%d,%d,%d,%d"%(lastbsoc_time, lastvoltage_val, lastbsoc_val, lastcurrent_val)

	except:
		sys.stderr.write("Had a dodgy line\n")
		continue



