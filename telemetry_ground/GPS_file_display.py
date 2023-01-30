#!/usr/bin/env python3
#This file reads in our telemetry from stdin and prints a trackfile described here (https://www.gpsvisualizer.com/tutorials/tracks.html) to stdout

import sys
import Telem_parser as tp

print("Type, Time, Latitude, Longitude, Alt")

desiredLabel = "gpsFix"
label = ""
while(label := tp.read_line()):
    try:
        if(label == desiredLabel):
            print("T,"+tp.values[desiredLabel].timestamp+","+str(int(tp.get(desiredLabel, "lat"))*1.0e-7)+","+str(int(tp.get(desiredLabel, "lng"))*1.0e-7) + "," + tp.get("pose", "altitude"))
    except ValueError as e:
        pass
