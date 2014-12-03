import socket
import krpc
import math
import time
import numpy as np

connection = None
def connect():
  global connection
  try:
    connection = krpc.connect(name="testscript")
    return True
  except socket.error:
    return False


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

#def update(var, label):
#  ship.control.sas = False
#  ap.set_throttle(0)
#  ap.set_steering(0, 0, 0)
#
#  while True:
#    error = 1
#    av_mag = 1
#    angle = math.pi/8
#    error, av_mag = align(ship, ship.avionics.surface_velocity)
#    out = ""
#    out += "error:   %10.3f\n" % error
#    out += "av_mag:  %10.3f\n" % av_mag
#    out += "forward: %10.3f %10.3f %10.3f\n" % ship.avionics.forward
#    var.set(out)
#    label.update_idletasks()

def update(var, stop_event):
  print "thread started"
  global connection
  while not connection:
    if stop_event.isSet():
      exit()
    if not connect():
      time.sleep(3)
      print "."
      continue
    print "connected!"
    while connection:
      ship = connection.space_center.active_vessel
      transd = connection.space_center.transform_direction
      ap = ship.auto_pilot
      kerbin = connection.space_center.bodies['Kerbin']
      mun = connection.space_center.bodies['Mun']
      sun = connection.space_center.bodies['Sun']
      moho = connection.space_center.bodies['Moho']

      error, av_mag = align(ship, ship.avionics.surface_velocity, True)
      out = ""
      out += "error:   %10.3f\n" % error
      out += "av_mag:  %10.3f\n" % av_mag
      out += "forward: %10.3f %10.3f %10.3f\n" % ship.avionics.forward
      if stop_event.isSet():
        connection._connection.close()
        exit()
      var.set(out)



