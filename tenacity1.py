import socket
import krpc
import math
import time
import numpy as np
from numpy import array
import matplotlib.pyplot as plt

def load(root):
  return Tenacity1(root)

class Tenacity1(KRCCModule):
  def unload(self):
    pass

  def __init__(self, root):
    super().__init__()
    self.canvas = Canvas(root)
    self.load()

  def update(self):
    print('update...')
    pass

  def name(self):
    return 'Tenacity 1'

  def load(self):
    self.canvas.pack(side=RIGHT, fill=BOTH, expand=True)

def static_var(varname, value):
  def decorate(func):
    setattr(func, varname, value)
    return func
  return decorate

connection = None
def connect():
  global connection
  try:
    connection = krpc.connect(name="Tenacity 1")
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


@static_var('next_ship_surface_up_update', 0)
@static_var('next_npole_pos_update', 0)
@static_var('ship_surface_up', 0)
@static_var('npole_pos', 0)
def vector2heading(ship, vector,
                   delta_ship_surface_up=1,
                   delta_npole_pos=10):
  vector = norm(vector)

  v2h = vector2heading
  if v2h.next_npole_pos_update - time.time() < 0:
    v2h.npole_pos = norm(ship.main_body.get_world_surface_position(90, 0, 0))
    v2h.next_npole_pos_update = time.time() + delta_npole_pos
  if v2h.next_ship_surface_up_update - time.time() < 0:
    v2h.ship_surface_up = ship.surface_up
    v2h.next_ship_surface_up_update = time.time() + delta_ship_surface_up

  east = vcross(norm(v2h.ship_surface_up), v2h.npole_pos)
  north = vcross(east, norm(v2h.ship_surface_up))

  pangle = np.arccos(vdot(vector, norm(v2h.ship_surface_up)))
  pitch = (math.pi / 2 - pangle) * 360 / 2 / math.pi
  yaw = np.arccos(vdot(north, vector)) * 360 / 2 / math.pi
  if vdot(vector, east) < 0:
    yaw += 180

  return pitch, yaw


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
      'Tf': 0,
    }
    last = None

    mission_start = 0
    first_burn_start = -1
    first_burn_end = -1
    second_burn_start = -1
    second_burn_end = -1
    pitch_end = -1
    ignition = -1

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
        set_throttle(ship, 1)
        set_sas(ship, False)
        set_rcs(ship, False)
        mission_start = raw.ut
        out += "Pre launch.\n"
        var.set(out)
        continue


      mission_time = raw.ut - mission_start
      out += "mission_time: %8.3f\n" % mission_time
      pitch, yaw = vector2heading(ship, ship.orbit_velocity, delta_npole_pos=1)
      out += "pitch: %8.3f\n" % pitch
      out += "yaw: %8.3f\n" % yaw
      if mag(ship.surface_velocity) < 60:
        last = align(ship, heading2vector(ship, 90, 0), last=last)
      elif pitch_end < 0:
        last = align(ship, heading2vector(ship, 87, 80), last=last)
        sangle = np.arccos(vdot(norm(ship.surface_velocity), norm(ship.surface_up)))
        spitch = (math.pi / 2 - sangle) * 360 / 2 / math.pi
        out += "spitch: %8.3f\n" % spitch
        if spitch < 87:
          pitch_end = raw.ut
      elif mission_time < 3*60 + 15:
        if mission_time > 2*60 + 30:
          last = align(ship, heading2vector(ship, pitch - 2, yaw), last=last, stop_rotation=True)
        else:
          last = align(ship, ship.surface_velocity, last=last)
      elif mission_time < 3*60 + 17:
        stage = proceed_to_stage(ship, 5)
        last = align(ship, heading2vector(ship, pitch - 2, yaw), last=last, stop_rotation=True)
      elif mission_time < 8*60 + 35:
        if kerbin.get_altitude(ship.position) > 130000:
          stage = proceed_to_stage(ship, 3)
          ship.control.set_action_group(4, True)
        else:
          stage = proceed_to_stage(ship, 4)
        ap = ship.orbit.apoapsis_altitude
        pe = ship.orbit.periapsis_altitude
        if ap > 300000 and pe > 250000:
          set_throttle(ship, 0)
        pitchsub = max(pitch, 5) * (1 - (min(max(300000 - ap, 0), 100000) / 100000))
        last = align(ship, heading2vector(ship, pitch - pitchsub, yaw), last=last)
      elif first_burn_start < 0 and mission_time > 8*60:
        if ship.latitude < 2 and ship.latitude > 0:
          last['Tf'] = 1
          set_rcs(ship, True)
          set_throttle(ship, 0)
        elif mag(last['error']) < 0.1 and ship.latitude < 1:
          last['Tf'] = 0.5
          set_rcs(ship, False)
          set_throttle(ship, 1)
          proceed_to_stage(ship, 1)
          first_burn_start = raw.ut
        else:
          proceed_to_stage(ship, 2)
        last = align(ship, ship.orbit_velocity, last=last)
      elif first_burn_start > 0 and first_burn_end < 0:
        ap = ship.orbit.apoapsis_altitude
        out += "ap: %8.3f\n" % ap
        if ap < 1000000 and ignition < mission_time:
          ship.control.set_action_group(3, False)
          ship.control.set_action_group(3, True)
          ignition = mission_time + 1
        if ap < 17000000:
          last = align(ship, ship.orbit_velocity, last=last)
        else:
          set_throttle(ship, 0)
          first_burn_end = raw.ut
      elif second_burn_start < 0:
        out += "align"
        if ship.latitude > 1.2:
          set_rcs(ship, True)
          last = align(ship, heading2vector(ship, 0, 90 + 42), last=last)
          if mag(last['error']) < 0.1:
            set_rcs(ship, False)
            set_throttle(ship, 1)
            second_burn_start = raw.ut
        else:
          rotation_vec = sub(ship.position, sun.position)
          last = align(ship, rotation_vec, last=last)
      elif second_burn_start > 0 and second_burn_end < 0:
        pe = ship.orbit.periapsis_altitude
        incl = ship.orbit.inclination * 360 / (2*math.pi)
        out += "pe: %8.3f\n" % pe
        out += "incl: %8.3f\n" % incl
        if pe < 1000000 and ignition < mission_time:
          ship.control.set_action_group(3, False)
          ship.control.set_action_group(3, True)
          ignition = mission_time + 1
        if pe < 16000000 and incl > 1.5:
          last = align(ship, heading2vector(ship, 0, 90 + 42), last=last)
          first_burn_duration = first_burn_end - first_burn_start
          if raw.ut - second_burn_start + first_burn_duration > 50:
            proceed_to_stage(ship, 0)
            set_rcs(ship, True)
        elif pe < 16000000 and incl < 1.5:
          out += "2nd"
          last = align(ship, ship.orbit_velocity, last=last)
        elif pe > 16000000 and incl > 1.5:
          out += "3rd"
          target_vec = vcross(ship.surface_up, ship.orbit_velocity)
          last = align(ship, target_vec, last=last)
        else:
          out += "4rd"
          set_throttle(ship, 0)
          second_burn_end = raw.ut
      else:
          out += "5rd"
          rotation_vec = sub(sun.position, ship.position)
          last = align(ship, rotation_vec, last=last)


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

