import sys


# read from file
inFile = sys.stdin

values = {}
strMessage = ""
strMessageNew = False

class telemLine:
	values = {}
	timestamp = ""

	def __str__(self):
		return self.timestamp + " " + str(self.values)

def read_line():
	global strMessage
	global strMessageNew
	line = inFile.readline()
	if not line:
		print("Read error")
		return False
	line = line.strip()
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
		# print(label)
		if label == "strmessage":
			strMessage = parts[2]
			strMessageNew = True
		else:
			valuesStrings = parts[2].split(",")
			#valuesStrings = list(map(lambda x : x.strip(), valuesStrings))
			#print(valuesStrings)
			for valueString in valuesStrings:
				valueString = valueString.strip()
				if valueString:
					keyval = valueString.split("=")
					if len(keyval) != 2:
						raise Exception('Invalid telem line "' + line + '": no =')
					parsedLine.values[keyval[0]] = keyval[1]
			values[label] = parsedLine

	return label


def newWarning():
	if strMessageNew:
		return "ERROR" in strMessage
	else:
		return False

def get(category, key):
	try:
		return values[category].values[key]
	except KeyError:
		return "XX"



