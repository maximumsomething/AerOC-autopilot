#This file reads in our telemetry from stdin and prints a trackfile described here (https://www.gpsvisualizer.com/tutorials/tracks.html) to stdout

import sys
import Telem_parser as tp

print("Type, Latitude, Longitude")

while(tp.read_line()):
    try:
        print("T,"+tp.get("gpsReckon", "lat")*1.0e-7+","+tp.get("gpsReckon", "lng")*1.0e-7)
    except ValueError as e:
        pass