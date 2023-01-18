# -*- coding: utf-8 -*-

from tkinter import *
from tkinter import ttk
  
#main window setup 
root = Tk()
root.title("AerOC Autopilot Ground Telemetry Viewer")
#create internal frame with padded edges
mainframe = ttk.Frame(root, padding="3 3 12 12")
mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
#setup window resizing behavior
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

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


ttk.Label(mainframe, text="Status: ").grid(column = 0, row = 1, sticky = (W))
ttk.Label(mainframe, textvariable = status).grid(column = 1, row = 1, sticky = (W))

root.mainloop()