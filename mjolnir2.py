import socket
from tkinter import Canvas, RIGHT, BOTH
import krpc
import math
import time
import numpy as np
from numpy import array
import matplotlib.pyplot as plt
from krcc_module import KRCCModule

# DECLARE_KRCC_MODULE
def load(root):
  return Mjolnir(root)


class Mjolnir(KRCCModule):

  def __init__(self, root):
    super().__init__()
    self.canvas = Canvas(root)
    self.canvas.pack(side=RIGHT, fill=BOTH, expand=True)
    self.canvas.configure(background='#000000')

  def __del__(self):
    self.canvas.destroy()

  def run(self):
    while not self.terminate:
      pass

  def name(self):
    return 'Mjolnir'


def static_var(varname, value):
  def decorate(func):
    setattr(func, varname, value)
    return func
  return decorate

connection = None
def connect():
  global connection
  try:
    connection = krpc.connect(name="Mjolnir 2")
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


@static_var('state', None)
def set_sas(ship, state):
  if set_sas.state is None or set_sas.state != state:
    ship.control.set_sas(state)
    set_sas.state = state


@static_var('state', None)
def set_rcs(ship, state):
  if set_rcs.state is None or set_rcs.state != state:
    ship.control.set_rcs(state)
    set_rcs.state = state


@static_var('state', None)
def set_throttle(ship, state):
  if set_throttle.state is None or set_throttle.state != state:
    ship.control.set_throttle(state if state > 0 else -1)
    set_throttle.state = state
  return state, set_throttle.state, out


@static_var('state', None)
def set_steering(ship, steering):
  ship.control.set_steering(float(steering[0]),
                            float(steering[1]),
                            float(steering[2]))
  #if (set_steering.state is None or
  #    any(set_steering.state[i] != 0 for i in xrange(3))):
  #  set_steering.state = steering
  #  set_neutral_controls(ship, False)


@static_var('state', None)
def set_neutral_controls(ship, state=True):
  if state and not set_neutral_controls.state:
    set_steering(ship, (0, 0, 0))
    #ship.control.set_neutral_controls()
  set_neutral_controls.state = state


@static_var('stage', None)
def proceed_to_stage(ship, stage):
  if proceed_to_stage.stage is None or proceed_to_stage.stage > stage:
    proceed_to_stage.stage = ship.control.activate_next_stage()
    proceed_to_stage.stage = ship.current_stage
  return proceed_to_stage.stage



@static_var('next_ship_surface_up_update', 0)
@static_var('next_npole_pos_update', 0)
@static_var('ship_surface_up', 0)
@static_var('npole_pos', 0)
def heading2vector(ship, pitch, yaw,
                   delta_ship_surface_up=1,
                   delta_npole_pos=10):
  if pitch < -90 or pitch > 90:
    print('head(): pitch out of bounds')
    return
  if yaw < 0 or yaw > 360:
    print('head(): yaw out of bounds')
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


def align(ship, target_vec, stop_rotation=False, rotation_vec=None, last=None):
  target_vec = norm(target_vec)
  if rotation_vec is not None:
    rotation_vec = norm(rotation_vec)

  if last is None:
    last = {
      'time': time.time(),
      'error': array([0, 0, 0]),
      'av': array([0, 0, 0]),
      'integral': array([0, 0, 0]),
      'steering': array([0, 0, 0]),
      'ep': array([0, 0, 0]),
      'ei': array([0, 0, 0]),
      'ed': array([0, 0, 0]),
      'Tf': 1.5,
    }

  ship_up = ship.up
  ship_right = ship.right
  ship_forward = ship.forward
  av = ship.angular_velocity

  # swap y and z to conform to [pitch, yaw, roll]
  av = (av[0], av[2], av[1])
  angular_vec = norm(add(mul(av[0], ship_up), mul(av[1], ship_right)))

  p_delta = sub(target_vec, mul(vdot(ship_forward, target_vec), ship_forward))
  y_delta = sub(target_vec, mul(vdot(ship_up, target_vec), ship_up))
  comb = add(p_delta, y_delta)
  error_vec = norm(sub(comb, mul(vdot(ship_forward, comb), ship_forward)))

  Tf = last['Tf']
  Kp = 2.5 * Tf
  Kd = 3 * Tf
  angular_vec = mul(math.sqrt(av[0]*av[0] + av[1]*av[1]) * Kd, angular_vec)
  magnitude = np.arccos(vdot(norm(target_vec), ship_forward)) * Kp
  error_vec = mul(magnitude, error_vec)
  steering_vec = add(error_vec, angular_vec)

  r_delta = 0
  if (rotation_vec is not None and
      np.arccos(vdot(rotation_vec, target_vec)) > 10 * math.pi / 180):
    rotation_proj = sub(rotation_vec, mul(vdot(ship_forward, rotation_vec), ship_forward))
    sign = 1 if vdot(ship_right, rotation_proj) > 0 else -1
    r_delta = sign * np.arccos(vdot(norm(rotation_proj), ship_up)) * Tf + av[2]
  elif stop_rotation:
    r_delta = av[2]

  y_angle = vdot(ship_right, steering_vec)
  p_angle = vdot(ship_up, steering_vec)
  steering = array([p_angle, y_angle, r_delta])
  steering = np.nan_to_num(steering)
  set_steering(ship, steering)



#  debug_vec_0.position = (0, 0, 0)
#  debug_vec_0.direction = norm(rotation_vec)
#  #debug_vec_0.scale = (0, 10, 0)
#  debug_vec_0.scale = 3
#
#  debug_vec_1.position = (0, 0, 0)
#  debug_vec_1.direction = ship.up
#  debug_vec_1.scale = 3
#
#  if rotation_proj is not None:
#    debug_vec_2.position = (0, 0, 0)
#    debug_vec_2.direction = rotation_proj
#    debug_vec_2.scale = 3


  return {
    'time': time.time(),
    'error': steering,
    'integral': array([0, 0, 0]),
    'av': av,
    'steering': steering,
    'ep': array([0, 0, 0]),
    'ei': array([0, 0, 0]),
    'ed': array([0, 0, 0]),
    'Tf': Tf,
  }

@static_var('launched', False)
def launched(ship):
  if not launched.launched:
    if ship.situation == connection.raw.VesselSituation.pre_launch:
      return False
    launched.launched = True
  return True


def roll_up_toward(target_vec):
  ship_forward = ship.forward
  proj = sub(target_vec, mul(vdot(ship_forward, target_vec), ship_forward))
  error = abs(1 - vdot(ship.up, proj)) + ship.angular_velocity[2]
  set_steering(ship, (0, 0, error))


def perpendicular_vector(ship, sun):
  sun_to_earth = sub(sun.position, ship.position)
  random_other_vector = (sun_to_earth[1], sun_to_earth[2], sun_to_earth[0])
  return vcross(norm(sun_to_earth), norm(random_other_vector))


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
  
  
  print('thread started')
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
        print('thread stopped')
        exit()
      if not connect():
        time.sleep(3)
        print('.')
    print('connected!')
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
    #print'connection.krpc.get_services(')
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
    stage = ship.current_stage

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
      'Tf': 0,
    }
    last = None

    mission_start = 0
    first_burn_start = -1
    first_burn_end = -1
    second_burn_start = -1
    second_burn_end = -1

    spawned_at = time.time()
    while connection:
      out = ""
      if stop_event.isSet():
        connection._connection.close()
        print('thread stopped')
        exit()
      start = time.time()

      last_time = cur_time
      last_error = error
      last_av = av

      if not launched(ship):
        set_throttle(ship, 1)
        set_sas(ship, False)
        set_rcs(ship, False)
        mission_start = time.time()
        out += "Pre launch.\n"
        var.set(out)
        continue


      mission_time = time.time() - mission_start
      out += "mission_time: %8.3f\n" % mission_time
      out += "stage: %s\n" % stage
      if mission_time < 4:
        out += "Leaving launch pad.\n"
      elif mission_time < 65:
        last = align(ship, heading2vector(ship, 87, 190), last=last)
      elif mission_time < 3*60 + 00:
        proceed_to_stage(ship, 3)
        if ship.orbit.inclination > 89.8 * math.pi / 180:
          last = align(ship, heading2vector(ship, 35, 180), last=last)
        else:
          last = align(ship, ship.surface_velocity, last=last)
      elif mission_time < 3*60 + 35:
        stage = proceed_to_stage(ship, 2)
        last = align(ship, heading2vector(ship, 25, 180), last=last)
      elif mission_time < 4*60 + 00:
        out += "Extend antenna!\n"
        proceed_to_stage(ship, 1)
        last = align(ship, heading2vector(ship, 10, 180), last=last)
      elif mission_time < 4*60 + 30:
        out += "Extend antenna!\n"
        last = align(ship, heading2vector(ship, 0, 180), last=last)
      elif mission_time < 6*60 + 30:
        last['Tf'] = 0.5
        if ship.orbit.apoapsis_altitude > 1310000:
          set_throttle(ship, 0)
        last = align(ship, heading2vector(ship, -4, 180), last=last)
      elif first_burn_start < 0 and mission_time > 6*60 + 10:
        if ship.orbit.time_to_apoapsis < 60:
          set_rcs(ship, True)
          last['Tf'] = 1
          last = align(ship, ship.orbit_velocity, last=last, stop_rotation=True)
          if mag(last['error']) < 0.01:
            set_throttle(ship, 1)
            first_burn_start = time.time()
        else:
          set_throttle(ship, 0)
          proceed_to_stage(ship, 0)
          target_vec = perpendicular_vector(ship, sun)
          rotation_vec = sub(ship.position, sun.position)
          last = align(ship, target_vec, last=last, rotation_vec=rotation_vec)
      elif first_burn_start > 0 and first_burn_end < 0:
        if ship.orbit.apoapsis_altitude < 1326943:
          last = align(ship, ship.orbit_velocity, last=last)
        else:
          set_throttle(ship, 0)
          first_burn_end = time.time()
      elif second_burn_start < 0:
        if ship.orbit.time_to_apoapsis < 60:
          set_rcs(ship, True)
          last = align(ship, ship.orbit_velocity, last=last)
          if mag(last['error']) < 0.01:
            set_throttle(ship, 1)
            second_burn_start = time.time()
        else:
          target_vec = perpendicular_vector(ship, sun)
          rotation_vec = sub(ship.position, sun.position)
          last = align(ship, target_vec, last=last, rotation_vec=rotation_vec)
      elif second_burn_start > 0 and second_burn_end < 0:
        if ship.orbit.periapsis_altitude < 1326776 and ship.orbit.periapsis_altitude < 1326776:
          last = align(ship, ship.orbit_velocity, last=last)
        else:
          set_throttle(ship, 0)
          set_rcs(ship, False)
          second_burn_end = time.time()
      else:
          target_vec = perpendicular_vector(ship, sun)
          rotation_vec = sub(ship.position, sun.position)
          last = align(ship, target_vec, last=last, rotation_vec=rotation_vec)


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

