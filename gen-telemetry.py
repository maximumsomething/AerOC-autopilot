#!/usr/bin/env python3


# Text log format:
# label: name=value, name2=value2, ...
# strmessage: "message string" (quotes and backslashes are escaped)

import sys

#srcfilename = sys.argv[1];
srcfilename = "telemetry_spec.txt"

dstAirHName = "main-control/telemetry_autogen.h"
dstAirCName = "main-control/telemetry_autogen.cpp"
dstGroundCName = "telemetry_ground/autogen.cpp"


srcfile = open(srcfilename, "r")

idEnum = ('enum telem_ids {\n'
	'telem_id_special_strmessage = 0,\n')
structs = ''
senders = ''
sendersH = ''
lengthChecker = ('int expectedLength(uint16_t id) {\n'
	'\tswitch (id) {\n')
reciever = ('void recievePacket(uint16_t id, uint16_t length, void* data) {\n'
	'\tswitch (id) {\n')

srclines = srcfile.readlines()
for line in srclines:
	line = line.strip()
	if (not line) or line[0] == '#':
		continue
	nameanddefs = line.split(maxsplit=1)
	name = nameanddefs[0]
	defs = nameanddefs[1]

	idEnum += f'\ttelem_id_{name},\n'

	structdefs = defs.replace(",", ";\n\t")

	structs += (f'struct telem_data_{name}' ' {\n'
	f'\t{structdefs};\n'
	'};\n')

	splitdefs = list(map(lambda x: x.rsplit(maxsplit=1), defs.split(',')))
	defnames = list(map(lambda x: x[1], splitdefs))
	deftypes = list(map(lambda x: x[0], splitdefs))

	commaDefnames = ', '.join(defnames)

	printItems = [f'"{name}: "']
	for defname in defnames:
		printItems += [f'"{defname}="', defname, '", "']


	streamPrint = "\n".join(list(map(lambda x: "\ttelem_save_stream->print("+x+");", printItems)))

	senders += ((f'void telem_{name}({defs})' ' {\n'
		   f'\tstruct telem_data_{name} data = {{ {commaDefnames} }};\n'
		   f'\tsend_telem_packet(telem_id_{name}, sizeof(data), (void *)&data);\n\n')
			+ streamPrint
			+ '\n}\n')
	sendersH += f'void telem_{name}({defs});\n'


	coutPrints = "".join(list(map(lambda x: " << +" + x, printItems)))
	unpacking = ' '.join(list(map(lambda n, t: f'{t} {n} = str->{n};', defnames, deftypes)))

	reciever += (f'\tcase telem_id_{name}:{{\n'
		f'\t\tstruct telem_data_{name}* str = (struct telem_data_{name}*) data;\n'
		f'\t\t{unpacking}\n'
		'\t\t' f'std::cout{coutPrints} << std::endl; \n'
		'\tbreak; }\n'
		)

	lengthChecker += (f'\tcase telem_id_{name}:\n'
				   f'\t\treturn sizeof(telem_data_{name});\n')


idEnum += '};\n'
reciever += "\t}\n}\n"
lengthChecker += ("\tdefault: return -1;\n"
	"\t}\n}\n")

dstAirH = open(dstAirHName, "w")
dstAirH.write("#pragma once\n#include <stdint.h>\n")
dstAirH.write(sendersH)

dstAirC = open(dstAirCName, "w")
dstAirC.write('#include "telemetry.h"\n')
dstAirC.write(idEnum)
dstAirC.write(structs)
dstAirC.write(senders)

dstGroundC = open(dstGroundCName, "w")
dstGroundC.write("#include <iostream>\n#include <stdint.h>\n")
dstGroundC.write(idEnum)
dstGroundC.write(structs)
dstGroundC.write(reciever)
dstGroundC.write(lengthChecker)




