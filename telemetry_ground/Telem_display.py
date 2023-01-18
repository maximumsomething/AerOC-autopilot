#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from tkinter import *
from tkinter import ttk
import Telem_parser as tp
from threading import Thread
import time
  
#main window setup 
root = Tk()
root.title("AerOC Autopilot Ground Telemetry Viewer")
#create internal frame with padded edges
mainframe = ttk.Frame(root, padding="10 10 10 10")
mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
#setup window resizing behavior
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

mainframe.columnconfigure(0, weight=1)
mainframe.columnconfigure(1, weight=1)
mainframe.columnconfigure(2, weight=1)
mainframe.columnconfigure(3, weight=1)


#all telemetry variables:
curRoll = StringVar()
tarRoll = StringVar()
curPitch = StringVar()
tarPitch = StringVar()
curBearing = StringVar()
tarBearing = StringVar()
curAirspeed = StringVar()
pressureDiff = StringVar()
curVertSpeed = StringVar()
tarVertSpeed = StringVar()
curAltitude = StringVar()
tarAltitude = StringVar()
aileronSignal = StringVar()
elevatorSignal = StringVar()
throttleSignal = StringVar()
rudderSignal = StringVar()
forAcc = StringVar()
leftAcc = StringVar()
downAcc = StringVar()
totalAcc = StringVar()
GPSLat = StringVar()
GPSLon = StringVar()
#ground variables
status = StringVar()
alarm = False
status.set("Awaiting signals...")

def updateValsForLine(linekey, valkeys, stringvars):
	for (key, var) in zip(valkeys, stringvars):
		var.set(tp.get(linekey, key))


def updateVals():
	while tp.read_line():
		updateValsForLine("pose", ["pitch", "roll", "bearing", "verticalSpeed", "altitude"], [curPitch, curRoll, curBearing, curVertSpeed, curAltitude])
		updateValsForLine("calInertial", ["ax", "ay", "az", "anorm"], [forAcc, leftAcc, downAcc, totalAcc])
		updateValsForLine("airspeed", ["speed"], [curAirspeed])
		updateValsForLine("controlOut", ["targetPitch", "targetVertSpeed", "elevators", "ailerons", "throttle"], [tarPitch, tarVertSpeed, elevatorSignal, aileronSignal, throttleSignal])

		# todo

		# for testing
		time.sleep(0.005)

#DISPLAY AND FORMATTING STUFF LIVES DOWN HERE
#status and alarm display
statusHeader = ttk.Label(mainframe, text="Status: ").grid(column = 0, row = 1, sticky = E)
statusDisp = ttk.Label(mainframe, textvariable = status).grid(column = 1, row = 1, sticky = W, padx = (0, 10))
alarmLabel = ttk.Label(mainframe, text="Alarm: ").grid(column = 2, row = 1, sticky = E, padx = (10, 0))
alarmAk = ttk.Button(mainframe, text = "Aknowledge").grid(column = 3, row = 1, sticky = W, padx = (0, 15))
#pitch information
pitchHeader = ttk.Label(mainframe, text="PITCH").grid(column = 0, row = 2, columnspan = 2, sticky = N, pady = (10, 0))
pitchCurLabel = ttk.Label(mainframe, text="Current: ").grid(column = 0, row = 3, sticky = E)
pitchCurDisp = ttk.Label(mainframe, textvariable = curPitch).grid(column = 1, row = 3, sticky = W)
pitchTarLabel = ttk.Label(mainframe, text="Target: ").grid(column = 0, row = 4, sticky = E)
pitchTarDisp = ttk.Label(mainframe, textvariable = tarPitch).grid(column = 1, row = 4, sticky = W)
#roll information
rollHeader = ttk.Label(mainframe, text="ROLL").grid(column = 2, row = 2, columnspan = 2, sticky = N, pady = (10, 0))
rollCurLabel = ttk.Label(mainframe, text="Current: ").grid(column = 2, row = 3, sticky = E)
rollCurDisp = ttk.Label(mainframe, textvariable = curRoll).grid(column = 3, row = 3, sticky = W)
rollTarLabel = ttk.Label(mainframe, text="Target: ").grid(column = 2, row = 4, sticky = E)
rollTarDisp = ttk.Label(mainframe, textvariable = tarRoll).grid(column = 3, row = 4, sticky = W)
#bearing information
bearingHeader = ttk.Label(mainframe, text="BEARING").grid(column = 0, row = 5, columnspan = 2, sticky = N, pady = (10, 0))
bearingCurLabel = ttk.Label(mainframe, text="Current: ").grid(column = 0, row = 6, sticky = E)
bearingCurDisp = ttk.Label(mainframe, textvariable = curBearing).grid(column = 1, row = 6, sticky = W)
bearingTarLabel = ttk.Label(mainframe, text="Target: ").grid(column = 0, row = 7, sticky = E)
bearingTarDisp = ttk.Label(mainframe, textvariable = tarBearing).grid(column = 1, row = 7, sticky = W)
#airspeed information
airspeedHeader = ttk.Label(mainframe, text="AIRSPEED").grid(column = 2, row = 5, columnspan = 2, sticky = N, pady = (10, 0))
airspeedCurLabel = ttk.Label(mainframe, text="Current: ").grid(column = 2, row = 6, sticky = E)
airspeedCurDisp = ttk.Label(mainframe, textvariable = curAirspeed).grid(column = 3, row = 6, sticky = W)
airspeedTarLabel = ttk.Label(mainframe, text="Pressure Diff: ").grid(column = 2, row = 7, sticky = E)
airspeedTarDisp = ttk.Label(mainframe, textvariable = pressureDiff).grid(column = 3, row = 7, sticky = W)
#altitude information
altitudeHeader = ttk.Label(mainframe, text="ALTITUDE").grid(column = 0, row = 8, columnspan = 2, sticky = N, pady = (10, 0))
altitudeCurLabel = ttk.Label(mainframe, text="Current: ").grid(column = 0, row = 9, sticky = E)
altitudeCurDisp = ttk.Label(mainframe, textvariable = curAltitude).grid(column = 1, row = 9, sticky = W)
altitudeTarLabel = ttk.Label(mainframe, text="Target: ").grid(column = 0, row = 10, sticky = E)
altitudeTarDisp = ttk.Label(mainframe, textvariable = tarAltitude).grid(column = 1, row = 10, sticky = W)
#vertSpeed information
vertSpeedHeader = ttk.Label(mainframe, text="VERTICAL SPEED").grid(column = 2, row = 8, columnspan = 2, sticky = N, pady = (10, 0))
vertSpeedCurLabel = ttk.Label(mainframe, text="Current: ").grid(column = 2, row = 9, sticky = E)
vertSpeedCurDisp = ttk.Label(mainframe, textvariable = curVertSpeed).grid(column = 3, row = 9, sticky = W)
vertSpeedTarLabel = ttk.Label(mainframe, text="Target: ").grid(column = 2, row = 10, sticky = E)
vertSpeedTarDisp = ttk.Label(mainframe, textvariable = tarVertSpeed).grid(column = 3, row = 10, sticky = W)
#control outputs
controlOutHeader = ttk.Label(mainframe, text="CONTROL OUTPUTS").grid(column = 0, row = 11, columnspan = 4, sticky = N, pady = (10, 0))
aileronSignalLabel = ttk.Label(mainframe, text="ailerons: ").grid(column = 0, row = 12, sticky = E)
aileronsSignalDisp = ttk.Label(mainframe, textvariable = aileronSignal).grid(column = 1, row = 12, sticky = W)
elevatorSignalLabel = ttk.Label(mainframe, text="elevator: ").grid(column = 2, row = 12, sticky = E)
elevatorSignalDisp = ttk.Label(mainframe, textvariable = elevatorSignal).grid(column = 3, row = 12, sticky = W)
throttleSignalLabel = ttk.Label(mainframe, text="throttles: ").grid(column = 0, row = 13, sticky = E)
throttlesSignalDisp = ttk.Label(mainframe, textvariable = throttleSignal).grid(column = 1, row = 13, sticky = W)
rudderSignalLabel = ttk.Label(mainframe, text="rudder: ").grid(column = 2, row = 13, sticky = E)
rudderSignalDisp = ttk.Label(mainframe, textvariable = rudderSignal).grid(column = 3, row = 13, sticky = W)
#inertial calibration
calInertialHeader = ttk.Label(mainframe, text="INERTIAL CALIBRATION DATA").grid(column = 0, row = 14, columnspan = 4, sticky = N, pady = (10, 0))
aileronAccLabel = ttk.Label(mainframe, text="forward gs: ").grid(column = 0, row = 15, sticky = E)
aileronsAccDisp = ttk.Label(mainframe, textvariable = forAcc).grid(column = 1, row = 15, sticky = W)
elevatorAccLabel = ttk.Label(mainframe, text="left gs: ").grid(column = 2, row = 15, sticky = E)
elevatorAccDisp = ttk.Label(mainframe, textvariable = leftAcc).grid(column = 3, row = 15, sticky = W)
throttleAccLabel = ttk.Label(mainframe, text="downward gs: ").grid(column = 0, row = 16, sticky = E)
throttlesAccDisp = ttk.Label(mainframe, textvariable = downAcc).grid(column = 1, row = 16, sticky = W)
rudderAccLabel = ttk.Label(mainframe, text="overall gs: ").grid(column = 2, row = 16, sticky = E)
rudderAccDisp = ttk.Label(mainframe, textvariable = totalAcc).grid(column = 3, row = 16, sticky = W)
#GPS data
GPSDisplayHeader = ttk.Label(mainframe, text="GPS DATA").grid(column = 0, row = 17, columnspan = 4, sticky = N, pady = (10, 0))
GPSLatLabel = ttk.Label(mainframe, text="Latitude: ").grid(column = 0, row = 18, sticky = E)
GPSLatDisp = ttk.Label(mainframe, textvariable = GPSLat).grid(column = 1, row = 18, sticky = W)
GPSLonLabel = ttk.Label(mainframe, text="Longitude: ").grid(column = 2, row = 18, sticky = E)
GPSLonDisp = ttk.Label(mainframe, textvariable = GPSLon).grid(column = 3, row = 18, sticky = W)


#UPDATE LOOPS AND STUFF START HERE
Thread(None, updateVals, "reading thread").start()

root.mainloop()
