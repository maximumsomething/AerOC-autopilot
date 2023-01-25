#!/usr/bin/env python3
#This file reads in our telemetry from stdin and prints a trackfile described here (https://www.gpsvisualizer.com/tutorials/tracks.html) to stdout

import sys
import Telem_parser as tp

print("Type, Latitude, Longitude")

desiredLabel = "gpsFix"
label = ""
while(label := tp.read_line()):
    try:
        if(label == desiredLabel):
            print("T,"+str(int(tp.get(desiredLabel, "lat"))*1.0e-7)+","+str(int(tp.get(desiredLabel, "lng"))*1.0e-7))
    except ValueError as e:
        pass
