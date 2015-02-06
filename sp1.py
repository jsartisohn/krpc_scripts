import socket
import krpc
import math
import time
import numpy as np
from numpy import array
import matplotlib.pyplot as plt


connection = None
def connect():
  global connection
  try:
    connection = krpc.connect(name="Sounding Probe 1")
    return True
  except socket.error:
    return False


right = (1, 0, 0)
forward = (0, 1, 0)
top = (0, 0, -1)

def add(s, v):
  return tuple([s[0] + v[0], s[1] + v[1], s[2] + v[2]])

def sub(s, v):
  return tuple([s[0] - v[0], s[1] - v[1], s[2] - v[2]])

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

def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    theta = np.asarray(theta)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2)
    b, c, d = -axis*math.sin(theta/2)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])

def head(ship, pitch, yaw):
  if pitch < -90 or pitch > 90:
    print 'head(): pitch out of bounds'
    return
  if yaw < 0 or yaw > 360:
    print 'head(): yaw out of bounds'
    return
  north = ship.main_body.transform.up
  east = vcross(norm(ship.surface_up), ship.main_body.transform.up)
  alpha = float(pitch) / 180 * math.pi
  m = rotation_matrix(list(north), alpha)
  result = np.dot(m, east)
  beta = float(yaw - 90) / 180 * math.pi
  m = rotation_matrix(list(ship.surface_up), beta)
  result = np.dot(m, result)
  return align(ship, norm(tuple(result)), False)

def heading2vector(ship, pitch, yaw):
  if pitch < -90 or pitch > 90:
    print 'head(): pitch out of bounds'
    return
  if yaw < 0 or yaw > 360:
    print 'head(): yaw out of bounds'
    return
  north = ship.main_body.transform.up
  east = vcross(norm(ship.surface_up), ship.main_body.transform.up)
  alpha = float(pitch) / 180 * math.pi
  m = rotation_matrix(list(north), alpha)
  result = np.dot(m, east)
  beta = float(yaw - 90) / 180 * math.pi
  m = rotation_matrix(list(ship.surface_up), beta)
  return np.dot(m, result)

def align(ship, target_vec, stop_rotation = False, last = None):
  target_vec = norm(target_vec)
  p_angle = vdot(ship.up, target_vec)
  y_angle = vdot(ship.right, target_vec)
  av = ship.angular_velocity
  # swap y and z to conform to [pitch, yaw, roll]
  av = array([av[0], av[2], 0])
  error = array([p_angle, y_angle, av[1] if stop_rotation else 0])
  if not last:
    last = {
      'time': time.time(),
      'error': error,
      'av': av,
      'steering': array([0, 0, 0])
    }
  dt = time.time() - last['time']

  delta_error = last['error'] - error
  delta_av = last['av'] - av
  de_over_dt = delta_error / dt
  da_over_dt = delta_av / dt
  mag = np.linalg.norm(error) * 5
  av_correction = np.clip(av / de_over_dt, -mag, mag)
  av_correction = np.nan_to_num(av_correction)

  de_corr = np.nan_to_num(de_over_dt / error)

  error_over_av = np.clip(error / da_over_dt, -np.linalg.norm(da_over_dt), np.linalg.norm(da_over_dt))

  global out
  out += "error  : %8.3f\n" % error[2]
  out += "av     : %8.3f\n" % av[1]
  out += "de / dt: %8.3f\n" % de_over_dt[1]
  bla = np.nan_to_num(de_over_dt / error)
  out += "de/dt/e: %8.3f\n" % bla[0]
  out += "de/dt/e: %8.3f\n" % bla[1]
  out += "de/dt/e: %8.3f\n" % bla[2]

  steering = last['steering'] + error * 0.5 - bla

  steering = np.nan_to_num(steering)
  ship.control.set_steering(float(steering[0]),
                            float(steering[1]),
                            float(steering[2]))
  return error, av, steering

debug_vec_0 = None
out = ""
figure = None
plot = None
def update(var, stop_event, error_event, fig):
  print "thread started"
  global figure
  global plot
  figure = fig
  plot = figure.add_subplot(111, xlim=(-20, 0), ylim=(-1.5, 1.5))
  plot.hold(False)
  np.set_printoptions(precision=1, threshold=110, suppress=True)
  try:
    global connection
    while not connection:
      if stop_event.isSet():
        print "thread stopped"
        exit()
      if not connect():
        time.sleep(3)
        print "."
    print "connected!"
    raw = connection.raw
    ship = raw.active_vessel
    control = ship.control
    kerbin = connection.raw.bodies['Kerbin']
    mun = connection.raw.bodies['Mun']
    sun = connection.raw.bodies['Sun']
    moho = connection.raw.bodies['Moho']
    global debug_vec_0
    debug_vec_0 = raw.debug_vector
    #debug_vec_0.position = mul(7, ship.transform.up)
    #debug_vec_0.direction = ship.surface_velocity * 10
    #debug_vec_0.direction = (0, 10, 0)

    cur_time = time.time()
    error = array([0, 0, 0])
    av = array([0, 0, 0])
    steering = array([0, 0, 0])
    last_time = cur_time
    last_av = array([0, 0, 0])
    last_error = array([0, 0, 0])
    last_steering = array([0, 0, 0])

    values = {
      'time': [],
      'error': [],
      'av': [],
      'steering': [],
    }

    global out
    while connection:
      out = ""
      if stop_event.isSet():
        connection._connection.close()
        print "thread stopped"
        exit()

      #if ship.situation == raw.VesselSituation.pre_launch:
      #  previous_error = array([0, 0, 0])
      #  integral = array([0, 0, 0])
      #  output = array([0, 0, 0])
      #  last_time = time.time()
      #  continue

      ship.control.set_throttle(1.0)


      target_vec = heading2vector(ship, 35, 90)
      last_time = cur_time
      last_error = error
      last_av = av
      last_frame = {
        'time': last_time,
        'error': last_error,
        'av': last_av,
        'steering': last_steering,
      }
      debug_vec_0.direction = mul(15, target_vec)
      error, av, steering = align(ship, target_vec, False, last_frame)
      cur_time = time.time()
      dt = cur_time - last_time

      if (ship.terrain_altitude > 65 or
          abs(ship.surface_velocity[0]) > 0.1 or
          abs(ship.surface_velocity[1]) > 0.1 or
          abs(ship.surface_velocity[2]) > 0.1):

        for i, x in enumerate(values['time']):
          values['time'][i] -= dt
        while len(values['time']) and values['time'][0] < -20:
          for i in values:
            values[i] = values[i][1:]
        values['time'].append(0)
        values['error'].append(error[1])
        values['av'].append(av[1])
        values['steering'].append(-steering[1])
        #for i in values:
        #  out += "%s: %8.3f\n" % (i, values[i][-1])

        plot.plot(values['time'], values['error'], 
                  values['time'], values['av'],
                  values['time'], values['steering'],
                  #values['time'], values[3],
                  scaley=False, scalex=False)
        figure.canvas.draw()










      #integral = np.clip(integral + error*dt, -1, 1)
      #derivative = np.clip((error - previous_error)/dt, -1, 1)
      #output = Kp*error + Ki*integral + Kd*derivative
      #previous_error = error
      #out += "%s\n" % error
      #out += "%s\n" % integral
      #out += "%s\n" % derivative
      #out += "%s\n" % output

      
      var.set(out)

      #debug_vec_0.direction = ship.surface_velocity * 10

#      # 1
#      altitude = ship.main_body.get_altitude(ship.position)
#      if ship.situation == raw.VesselSituation.pre_launch:
#        continue
#
#      var.set("ship.height_from_terrain: %s" % altitude)
#      if altitude < 500:
#        align(ship, ship.surface_up)
#      else:
#        align(ship, ship.orbit_velocity)

#      # 2
#      ship.control.set_throttle(1)
#      if ship.situation == raw.VesselSituation.pre_launch:
#        continue
#      if ship.orbit_velocity < 50:
#        var.set("aligning: ship.surface_up")
#        align(ship, ship.surface_up)
#      if altitude < 10000:
#        east = vcross(ship.surface_up, ship.main_body.transform.up)
#        var.set("aligning: east")
#        align(ship, mul(0.1, norm(east)))z

  except (AttributeError, TypeError, ValueError, NameError, KeyError, SyntaxError) as e:
    error_event.set()
    connection._connection.close()
    raise
  except Exception as e:
    if e.errno != 140:
      raise

