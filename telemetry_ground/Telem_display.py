# -*- coding: utf-8 -*-

from tkinter import *
from tkinter import ttk
  
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
tarAirspeed = StringVar()
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
#ground variables
status = StringVar()
alarm = False
status.set("Awaiting signals...")

#status and alarm display
statusLabel = ttk.Label(mainframe, text="Status: ").grid(column = 0, row = 1, sticky = E)
statusDisp = ttk.Label(mainframe, textvariable = status).grid(column = 1, row = 1, sticky = W, padx = (0, 10))
alarmLabel = ttk.Label(mainframe, text="Alarm: ").grid(column = 2, row = 1, sticky = E, padx = (10, 0))
alarmAk = ttk.Button(mainframe, text = "Aknowledge").grid(column = 3, row = 1, sticky = W)
#pitch information
pitchDisp = ttk.Label(mainframe, text="PITCH").grid(column = 0, row = 2, columnspan = 2, sticky = N)
pitchCurLabel = ttk.Label(mainframe, text="Current: ").grid(column = 0, row = 3, sticky = E)
pitchCurDisp = ttk.Label(mainframe, textvariable = curPitch).grid(column = 1, row = 3, sticky = W)
pitchTarLabel = ttk.Label(mainframe, text="Target: ").grid(column = 0, row = 4, sticky = E)
pitchTarDisp = ttk.Label(mainframe, textvariable = tarPitch).grid(column = 1, row = 4, sticky = W)
#roll information
rollDisp = ttk.Label(mainframe, text="ROLL").grid(column = 2, row = 2, columnspan = 2, sticky = N)
rollCurLabel = ttk.Label(mainframe, text="Current: ").grid(column = 2, row = 3, sticky = E)
rollCurDisp = ttk.Label(mainframe, textvariable = curRoll).grid(column = 3, row = 3, sticky = W)
rollTarLabel = ttk.Label(mainframe, text="Target: ").grid(column = 2, row = 4, sticky = E)
rollTarDisp = ttk.Label(mainframe, textvariable = tarRoll).grid(column = 3, row = 4, sticky = W)
#bearing information
bearingDisp = ttk.Label(mainframe, text="BEARING").grid(column = 0, row = 5, columnspan = 2, sticky = N)
bearingCurLabel = ttk.Label(mainframe, text="Current: ").grid(column = 0, row = 6, sticky = E)
bearingCurDisp = ttk.Label(mainframe, textvariable = curPitch).grid(column = 1, row = 6, sticky = W)
bearingTarLabel = ttk.Label(mainframe, text="Target: ").grid(column = 0, row = 7, sticky = E)
bearingTarDisp = ttk.Label(mainframe, textvariable = tarPitch).grid(column = 1, row = 7, sticky = W)
#airspeed information
airspeedDisp = ttk.Label(mainframe, text="AIRSPEED").grid(column = 2, row = 5, columnspan = 2, sticky = N)
airspeedCurLabel = ttk.Label(mainframe, text="Current: ").grid(column = 2, row = 6, sticky = E)
airspeedCurDisp = ttk.Label(mainframe, textvariable = curPitch).grid(column = 3, row = 6, sticky = W)
airspeedTarLabel = ttk.Label(mainframe, text="Target: ").grid(column = 2, row = 7, sticky = E)
airspeedTarDisp = ttk.Label(mainframe, textvariable = tarPitch).grid(column = 3, row = 7, sticky = W)

root.mainloop()