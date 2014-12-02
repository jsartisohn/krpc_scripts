import krpc
import time
import sys
import os
import math
import numpy as np

import operator

from Tkinter import *


connection = krpc.connect(name="Luna 1")
ship = connection.space_center.active_vessel
transd = connection.space_center.transform_direction
ap = ship.auto_pilot
kerbin = connection.space_center.bodies['Kerbin']
mun = connection.space_center.bodies['Mun']
sun = connection.space_center.bodies['Sun']
moho = connection.space_center.bodies['Moho']


tk = Tk()
tk.title("Avionics")
tk.geometry("1200x250+2180+750")
app = Frame(tk)
app.grid()
var = StringVar()
var.set("")
label = Label(app, textvariable = var, font=("Liberation Mono", 10))
label.grid()

right = (1, 0, 0)
forward = (0, 1, 0)
top = (0, 0, -1)

def add(s, v):
  return tuple([s[0] + v[0], s[1] + v[1], s[2] + v[2]])

def mul(s, v):
  return tuple([s * x for x in v])

def mag(v):
  return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

def norm(v):
  return (v[0]/mag(v), v[1]/mag(v), v[2]/mag(v))

def vdot(a, b):
  return float(np.vdot(a, b))

def vcross(a, b):
  return np.cross(a, b)



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


os.system('clear')
target = mun
error = 1
vel_mag = 1
#ship.control.activate_next_stage()
ap.set_throttle(0)
#sd_last = ship.direction(ship.non_rotating_reference_frame)
ship.control.sas = False
ap.set_steering(0, 0, 0)
def update():
  #sv = ap.surface_velocity
  #pos = ship.position(kerbin.non_rotating_reference_frame)
  #s_prog = connection.space_center.transform_velocity(sv, pos, kerbin.non_rotating_reference_frame, ship.reference_frame)
  #print "%10.3f %10.3f %10.3f" % norm(ship.avionics.orbit_velocity)
  #print "%10.3f %10.3f %10.3f" % ship.avionics.forward_axis
  #print "%10.3f %10.3f %10.3f" % ship.avionics.up_axis
  #print "="*30
  error, vel_mag = align3(ship, ship.avionics.orbit_velocity)

import thread
def update():
  while True:
    error = ""
    av_mag = ""
    angle = math.pi/8
    #error, av_mag = align(ship, add(norm(ship.avionics.surface_up), norm(ship.avionics.surface_velocity)))
    error, av_mag = align(ship, ship.avionics.surface_velocity)
    out = ""
    out += "error:   %10.3f\n" % error
    out += "av_mag:  %10.3f\n" % av_mag
    out += "forward: %10.3f %10.3f %10.3f\n" % ship.avionics.forward
    var.set(out)
    label.update_idletasks()

thread.start_new_thread(update, ())

#tk.update_idletasks = update
tk.mainloop()


#import thread, time
#def myfunc(a1,a2):
#  while True:
#    print a1,a2
#    time.sleep(1)
#thread.start_new_thread(myfunc,("test","arg2")

  #error, vel_mag = align(ship, ship.flight(ship.reference_frame).prograde, True)
  #error, vel_mag = align(ship, ship.flight(ship.reference_frame).prograde)
  #error, vel_mag = align(ship, ap.surface_velocity())
  #error, vel_mag = align(ship, ship.velocity(kerbin.surface_reference_frame))
  #print "%10.5f %10.5f %10.5f" % ship.velocity(kerbin.surface_reference_frame)
  # % (error, vel_mag)
  #time.sleep(1)

#ap.set_steering(0, 0, 0)
#ship.control.sas = True

































#ch * 0.9, yaw * 0.9, av[1] * 0.9)
#
#def align2(ship, vector, stop_rotation = False, strength = 1):
#  vector = norm(vector)
#  av = ship.angular_velocity(ship.non_rotating_reference_frame)
#  strength = max(0.1, max(min(10, strength), abs(av[1])))
#  error = abs(vdot(vector, right))
#  vel_mag = abs(av[0]) + abs(av[2])
#  pitch = -vdot(vector, top) * strength + av[2]
#  yaw = vdot(vector, right) * strength - av[0]
#  roll = 0
#  if stop_rotation:
#    roll = av[1]
#    vel_mag += abs(av[1])
#  print "="*30
#  #print "%10.3f %10.3f %10.3f" % av
#  #print "%10.3f %10.3f %10.3f" % (pitch, yaw, roll)
#  print "="*30
#  ship.auto_pilot.set_steering(pitch, yaw, roll)
#  return error, vel_mag
#
#os.system('clear')
#target = mun
#error = 1
#vel_mag = 1
##ship.control.activate_next_stage()
#ap.set_throttle(0)
##sd_last = ship.direction(ship.non_rotating_reference_frame)
#ship.control.sas = False
#ap.set_steering(0, 0, 0)
#while error > 0.001 or vel_mag > 0.00001:
#  #print "%10.3f %10.3f %10.3f" % transd(ap.surface_velocity(), )
#
#  print "%10.3f %10.3f %10.3f" % ship.flight(ship.reference_frame).prograde
#  print "%10.3f %10.3f %10.3f" % ship.flight(ship.orbital_reference_frame).prograde
#  print "%10.3f %10.3f %10.3f" % ship.flight(ship.surface_reference_frame).prograde
#  print "="*30
#  print "%10.3f %10.3f %10.3f" % ship.flight(kerbin.reference_frame).prograde
#  print "%10.3f %10.3f %10.3f" % ship.flight(kerbin.orbital_reference_frame).prograde
#  print "%10.3f %10.3f %10.3f" % ship.flight(kerbin.surface_reference_frame).prograde
#  print "="*30
#  error, vel_mag = align(ship, ship.flight(ship.reference_frame).prograde, True)
#  #error, vel_mag = align(ship, ship.flight(ship.reference_frame).prograde)
#  #error, vel_mag = align(ship, ap.surface_velocity())
#  #error, vel_mag = align(ship, ship.velocity(kerbin.surface_reference_frame))
#  #print "%10.5f %10.5f %10.5f" % ship.velocity(kerbin.surface_reference_frame)
#  # % (error, vel_mag)
#  #time.sleep(1)
#  print "%10.3f %10.3f %10.3f" % (error, vel_mag)
#  os.system("clear")
#  sys.stdout.flush()
#ap.set_steering(0, 0, 0)
#ship.control.sas = True
#
#
