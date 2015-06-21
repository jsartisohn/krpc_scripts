import socket
import krpc
import math
import time
import numpy as np
from numpy import array
import matplotlib.pyplot as plt

def static_var(varname, value):
  def decorate(func):
    setattr(func, varname, value)
    return func
  return decorate

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
  m = mag(v) if mag(v) > 0 else 1
  return (v[0]/m, v[1]/m, v[2]/m)

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

@static_var('next_ship_surface_up_update', 0)
@static_var('next_npole_pos_update', 0)
@static_var('ship_surface_up', 0)
@static_var('npole_pos', 0)
def heading2vector(ship, pitch, yaw,
                   delta_ship_surface_up=1,
                   delta_npole_pos=10):
  if pitch < -90 or pitch > 90:
    print 'head(): pitch out of bounds'
    return
  if yaw < 0 or yaw > 360:
    print 'head(): yaw out of bounds'
    return

  h2v = heading2vector
  if h2v.next_npole_pos_update - time.time() < 0:
    h2v.npole_pos = norm(ship.main_body.get_world_surface_position(90, 0, 0))
    h2v.next_npole_pos_update = time.time() + delta_npole_pos
  if h2v.next_ship_surface_up_update - time.time() < 0:
    h2v.ship_surface_up = ship.surface_up
    h2v.next_ship_surface_up_update = time.time() + delta_ship_surface_up

  east = vcross(norm(h2v.ship_surface_up), h2v.npole_pos)
  pitch_axis = vcross(h2v.ship_surface_up, east)
  yaw_axis = h2v.ship_surface_up

  pitch_arc = float(-pitch) / 180 * math.pi
  result = np.dot(rotation_matrix(pitch_axis, pitch_arc), east)
  yaw_arc = float(yaw - 90) / 180 * math.pi
  result = np.dot(rotation_matrix(yaw_axis, yaw_arc), result)
  return result


def align(ship, target_vec, stop_rotation = False, last = None):
  target_vec = norm(target_vec)
  global out

  ship_up = ship.up
  ship_right = ship.right
  ship_forward = ship.forward
  av = ship.angular_velocity




  # swap y and z to conform to [pitch, yaw, roll]
  stop_rotation = False
  av = (av[0], av[2], av[1] if stop_rotation else 0)
  angular_vec = norm(add(mul(av[0], ship_up), mul(av[1], ship_right)))

  p_delta = sub(target_vec, mul(vdot(ship_forward, target_vec), ship_forward))
  y_delta = sub(target_vec, mul(vdot(ship_up, target_vec), ship_up))
  comb = add(p_delta, y_delta)
  error_vec = norm(sub(comb, mul(vdot(ship_forward, comb), ship_forward)))

  Tf = 1.5
  bla = 3
  Kp = 2.5 * Tf
  Kd = 3 * Tf
  angular_vec = mul(mag(av) * Kd, angular_vec)
  magnitude = np.arccos(vdot(norm(target_vec), ship_forward)) * Kp
  error_vec = mul(magnitude, error_vec)
  steering_vec = add(error_vec, angular_vec)

  y_angle = vdot(ship_right, steering_vec)
  p_angle = vdot(ship_up, steering_vec)





  steering = array([p_angle, y_angle, 0])


  error = steering
  if last is None:
    last = {
      'time': time.time(),
      'error': error,
      'av': av,
      'integral': array([0, 0, 0]),
      'steering': array([0, 0, 0]),
      'ep': array([0, 0, 0]),
      'ei': array([0, 0, 0]),
      'ed': array([0, 0, 0]),
    }

  dt = time.time() - last['time']
  de = sub(error, last['error'])
  #de = np.clip(de, [-lim, -lim, -lim], [lim, lim, lim])

  tau = 1
  Kp = 1 * tau
  Ki = 0.3 * tau * 0
  Kd = 3 * tau
  integral = add(last['integral'], mul(dt, error))
  ep = mul(Kp, error)
  lim = mag(ep) * 3
  ei = np.clip(mul(Ki, integral), [-lim, -lim, -lim], [lim, lim, lim])
  lim = mag(ei)
  ed = mul(Kd, av)


  steering = np.nan_to_num(steering)
  #steering = add(last['steering'], mul(1 / (tau/dt + 1), (sub(steering, last['steering']))))
  if False:
    #ship.control.set_steering(0, 0, 0)
    ship.control.set_neutral_controls()
  else:
    ship.control.set_steering(float(steering[0]),
                              float(steering[1]),
                              float(steering[2]))


#  debug_vec_1.position = mul(-0.25, ship_forward)
#  debug_vec_1.direction = target_vec
#  debug_vec_1.scale = 10
#
#  debug_vec_2.position = mul(-0.25, ship_forward)
#  debug_vec_2.color = (0, 1, 0, 1)
#  debug_vec_2.direction = angular_vec
#  debug_vec_2.scale = 10
#
#  debug_vec_3.position = mul(-0.25, ship_forward)
#  debug_vec_3.color = (1, 0, 0, 1)
#  debug_vec_3.direction = steering_vec
#  debug_vec_3.scale = 10
#
#  debug_vec_4.position = mul(-0.25, ship_forward)
#  debug_vec_4.color = (0, 0, 1, 1)
#  debug_vec_4.direction = add(steering_vec, angular_vec)
#  debug_vec_4.scale = 10


  return {
    'time': time.time(),
    'error': error,
    'integral': integral,
    'av': av,
    'steering': steering,
    'ep': ep,
    'ei': ei,
    'ed': ed,
  }

@static_var('launched', False)
def launched(ship):
  if not launched.launched:
    if ship.situation == connection.raw.VesselSituation.pre_launch:
      return False
    launched.launched = True
  return True

debug_vec_0 = None
debug_vec_1 = None
debug_vec_2 = None
debug_vec_3 = None
debug_vec_4 = None
out = ""
figure = None
plot = None
def update(var, stop_event, error_event, fig):
  dt_error_margin = 0.1
  global out
  
  
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
    global debug_vec_1
    global debug_vec_2
    global debug_vec_3
    global debug_vec_4
    #import inspect
    #from pprint import pprint as pp
    #pp(inspect.getmembers(connection.krpc.get_services))
    #print connection.krpc.get_services()
    raw.clear_debug_vectors2()
    debug_vec_0 = raw.new_debug_vector()
    debug_vec_1 = raw.new_debug_vector()
    debug_vec_2 = raw.new_debug_vector()
    debug_vec_3 = raw.new_debug_vector()
    debug_vec_4 = raw.new_debug_vector()
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
      'ep': [],
      'ei': [],
      'ed': [],
    }
    last = {
      'time': time.time(),
      'error': [[0, 0, 0]],
      'av': [[0, 0, 0]],
      'integral': array([0, 0, 0]),
      'steering': array([0, 0, 0]),
      'ep': array([0, 0, 0]),
      'ei': array([0, 0, 0]),
      'ed': array([0, 0, 0]),
    }
    last = None

    mission_start = 0
    spawned_at = time.time()
    while connection:
      out = ""
      if stop_event.isSet():
        connection._connection.close()
        print "thread stopped"
        exit()
      start = time.time()

      last_time = cur_time
      last_error = error
      last_av = av


      if not launched(ship):
        ship.control.set_throttle(1.0)
        ship.control.set_sas(False)
        ship.control.set_rcs(False)
        mission_start = time.time()
        continue

      #if time.time() - mission_start < 4:
      #  pass
      #elif False:
      #  last = align2(ship, heading2vector(ship, 70, 160), last)
      #  pass
      #elif time.time() - mission_start < 5:
      #  ship.control.set_sas(False)
      #  ship.control.set_steering(0.5, -0.25, 0)
      #elif time.time() - mission_start < 60:
      #  ship.control.set_steering(0, 0, 0)
      #  if not ship.sas:
      #    ship.control.set_sas(True)
      #  if ship.current_stage >= 4:
      #    ship.control.activate_next_stage()
      #elif time.time() - mission_start < 2*60:
      #  out += "< 2*60"
      #  if ship.orbit.inclination > 85 * (360 / (2 * math.pi)):
      #    last = align(ship, heading2vector(ship, 50, 180), last=last)
      #  if ship.sas:
      #    ship.control.set_sas(False)
      #elif time.time() - mission_start < 2*60 + 55:
      #  out += "< 2*60 + 55"
      #  last = align(ship, heading2vector(ship, 40, 180), last=last)
      #elif time.time() - mission_start < 3*60 + 10:
      #  out += "< 3*60 + 10"
      #  if ship.current_stage >= 2:
      #    ship.control.activate_next_stage()
      #  last = align(ship, heading2vector(ship, 30, 180), last=last)
      #elif time.time() - mission_start < 4*60 + 10:
      #  out += "< 4*60 + 10"
      #  if ship.sas:
      #    ship.control.set_sas(False)
      #  last = align(ship, heading2vector(ship, 20, 180), last=last)
      #elif time.time() - mission_start < 5*60 + 10:
      #  last = align(ship, heading2vector(ship, 10, 180), last=last)

      last = align(ship, heading2vector(ship, 0, 0), True, last=last)


      if (False and
          (abs(ship.surface_velocity[0]) > 0.1 or
           abs(ship.surface_velocity[1]) > 0.1 or
           abs(ship.surface_velocity[2]) > 0.1)):
        dt_error_margin += 0.1

        cur_time = time.time()
        dt = cur_time - last_time

        if not last:
          continue
        for i, x in enumerate(values['time']):
          values['time'][i] -= dt
        while len(values['time']) and values['time'][0] < -20:
          for i in values:
            values[i] = values[i][1:]
        values['time'].append(0)
        values['error'].append(last['error'][0])
        values['av'].append(last['av'][0])
        values['steering'].append(-last['steering'][0])
        values['ep'].append(last['ep'][0])
        values['ei'].append(last['ei'][0])
        values['ed'].append(last['ed'][0])

        plot.plot(values['time'], values['ep'],  # blue
                  values['time'], values['ei'],  # green
                  values['time'], values['ed'],  # red
                  scaley=False, scalex=False)
        figure.canvas.draw()


      if time.time() - start > dt_error_margin:
        out += "!delta_t: %8.3f\n" % (time.time() - start)
      var.set(out)


  except (AttributeError, TypeError, ValueError, NameError, KeyError, SyntaxError) as e:
    error_event.set()
    connection._connection.close()
    raise
  except Exception as e:
    if e.errno != 140:
      raise

