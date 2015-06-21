import krpc
import time
import sys
import os
import math
import numpy as np
import threading
import operator
#from Tkinter import *
import Tkinter as Tk

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends.backend_tkagg import NavigationToolbar2TkAgg
from matplotlib.backend_bases import key_press_handler



restart_event = threading.Event()
restart_event.clear()
error_event = threading.Event()
error_event.clear()
terminate = False
def reload_script():
  global restart_event
  global script
  restart_event.set()
  error_event.clear()
  if not script:
    threading.Thread(target=reload_thread).start()

f = Figure(figsize=(9,4), dpi=100)

import sp1
import mjolnir2
script = None
def reload_thread():
  global script
  global restart_event
  global terminate
  while not terminate and not error_event.isSet():
    reload(mjolnir2)
    restart_event.clear()
    script = threading.Thread(target=mjolnir2.update, args=(var, restart_event, error_event, f))
    print "starting thread"
    script.start()
    script.join()
    print "finishing thread"
  script = None




tk = Tk.Tk()
tk.title("Avionics")
#tk.geometry("1200x250+2180+750")
tk.geometry("1200x450+2180+550")
app = Tk.Frame(tk)
app.grid(sticky=Tk.W)
button = Tk.Button(app, text="reload", command=reload_script, font=("Liberation Mono", 10))
button.grid(sticky=Tk.W)
var = Tk.StringVar()
var.set("")
label = Tk.Label(app, textvariable = var, font=("Liberation Mono", 10),
              justify=Tk.LEFT)
label.grid(sticky=Tk.W)


canvasframe = Tk.Frame(tk)
canvasframe.grid(sticky=Tk.E, column=1, row=0)


canvas = FigureCanvasTkAgg(f, master=canvasframe)
canvas.show()
canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)

toolbar = NavigationToolbar2TkAgg( canvas, canvasframe )
toolbar.update()
canvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)

def on_key_event(event):
    print('you pressed %s'%event.key)
    key_press_handler(event, canvas, toolbar)

canvas.mpl_connect('key_press_event', on_key_event)
#plt.ion()
#plt.show()


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
print "terminating"
terminate = True
restart_event.set()
