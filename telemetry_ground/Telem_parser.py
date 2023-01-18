#!/usr/bin/env python3

import sys


# read from file
inFile = sys.stdin

values = {}
strmessages = []

class telemLine:
	values = {}
	timestamp = ""

	def __str__(self):
		return self.timestamp + " " + str(self.values)

def read_line():
	line = inFile.readline().strip()
	if (line):
		parsedLine = telemLine()
		parts = line.split(" ", 2)
		parsedLine.timestamp = parts[0]
		label = parts[1]
		# basic syntax check
		if label[-1] != ':':
			raise Exception("Invalid telem line: no label")
		# remove colon
		label = label.rstrip(":")
		# special handling for strmessage
		if label == "strmessage":
			# todo
			pass
		else:
			valuesStrings = parts[2].split(",")
			#valuesStrings = list(map(lambda x : x.strip(), valuesStrings))
			#print(valuesStrings)
			for valueString in valuesStrings:
				valueString = valueString.strip()
				if valueString:
					keyval = valueString.split("=")
					if len(keyval) != 2:
						raise Exception("Invalid telem line: no =")
					parsedLine.values[keyval[0]] = keyval[1]
			values[label] = parsedLine




def get(category, key):
	try:
		return values[category].values[key]
	except KeyError:
		return "XX"



