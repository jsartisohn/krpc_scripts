import socket
import krpc
import math
import time
import numpy as np

connection = None
def script_connect():
  global connection
  try:
    print 'trying to connect'
    connection = krpc.connect(name="kRPC dev")
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

def align(ship, control, target_vec, stop_rotation = False, strength = 1):
  target_vec = norm(target_vec)
  av = ship.angular_velocity
  strength = max(0.1, max(min(10, strength), abs(av[1])))
  p_angle = vdot(ship.up, target_vec)
  y_angle = vdot(ship.right, target_vec)
  av_mag = abs(av[0]) + abs(av[2])
  error = abs(p_angle) + abs(y_angle)
  pitch = p_angle * strength + av[0]
  yaw = y_angle * strength + av[2]
  roll = 0
  if stop_rotation:
    roll = av[1]
    av_mag += abs(av[1])
  control.set_steering(pitch, yaw, roll)
  return error, av_mag


while True:
  global connection
  while not connection:
    if not script_connect():
      time.sleep(3)
      print "."
  print "connected!"
  raw_ship = connection.raw.active_vessel
  ship = connection.space_center.active_vessel
  control = connection.raw.control
  transd = connection.space_center.transform_direction
  kerbin = connection.space_center.bodies['Kerbin']
  mun = connection.space_center.bodies['Mun']
  sun = connection.space_center.bodies['Sun']
  moho = connection.space_center.bodies['Moho']
  while connection:
    if raw_ship.situation == connection.raw.vessel_situation.pre_launch:
      continue
    if ship.flight().surface_altitude < 500:
      align(raw_ship, control, raw_ship.surface_up)
    align(raw_ship, control, raw_ship.orbit_velocity)
