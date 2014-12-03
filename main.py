import krpc
import time
import sys
import os
import math
import numpy as np
import threading
import operator
from Tkinter import *


stop_event = threading.Event()
def reload_script():
  global stop_event
  stop_event.set()
  threading.Thread(target=reload_thread).start()

import testscript
script_thread = None
def reload_thread():
  global script_thread
  if script_thread:
    print "joining thread"
    script_thread.join()
  reload(testscript)
  print "starting thread"
  script_thread = threading.Thread(target=testscript.update,
                                   args=(var, label, stop_event))
  global stop_event
  stop_event.clear()
  script_thread.start()

tk = Tk()
tk.title("Avionics")
tk.geometry("1200x250+2180+750")
app = Frame(tk)
app.grid()
button = Button(app, text="reload", command=reload_script, font=("Liberation Mono", 10))
button.grid()
var = StringVar()
var.set("")
label = Label(app, textvariable = var, font=("Liberation Mono", 10))
label.grid()



def align(ship, target_vec, stop_rotation = False, strength = 1):
  target_vec = norm(target_vec)
  av = ship.avionics.angular_velocity
  strength = max(0.1, max(min(10, strength), abs(av[1])))
  p_angle = vdot(ship.avionics.up, target_vec)
  y_angle = vdot(ship.avionics.right, target_vec)
  av_mag = abs(av[0]) + abs(av[2])
  error = abs(p_angle) + abs(y_angle)
  pitch = p_angle * strength + av[0]
  yaw = y_angle * strength + av[2]
  roll = 0
  if stop_rotation:
    roll = av[1]
    av_mag += abs(av[1])
  ship.auto_pilot.set_steering(pitch, yaw, roll)
  return error, av_mag


def update():
  while True:
    error = 1
    av_mag = 1
    angle = math.pi/8
    error, av_mag = align(ship, ship.avionics.surface_velocity)
    out = ""
    out += "error:   %10.3f\n" % error
    out += "av_mag:  %10.3f\n" % av_mag
    out += "forward: %10.3f %10.3f %10.3f\n" % ship.avionics.forward
    var.set(out)
    label.update_idletasks()

reload_script()
tk.mainloop()
stop_event.set()
if script_thread:
  script_thread.join()
