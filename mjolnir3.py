import io
import logging
import sys
import time
import traceback
from tkinter import Listbox, LEFT, BOTH, Label, \
  StringVar, NW, BooleanVar, DISABLED, NORMAL, X, NE

import krpc
from ttk import Checkbutton, Entry

from krcc_module import KRCCModule
from mjolnir2 import align, heading2vector, rotation_matrix, add, mul, sub, norm, vdot, vcross
from numpy import array
import math
import numpy as np

def h2v(pitch, yaw):
  east = (0, 0, 1)
  pitch_axis = (0, -1, 0)
  yaw_axis = (1, 0, 0)

  pitch_arc = float(-pitch) / 180 * math.pi
  result = np.dot(rotation_matrix(pitch_axis, pitch_arc), east)
  yaw_arc = float(yaw - 90) / 180 * math.pi
  result = np.dot(rotation_matrix(yaw_axis, yaw_arc), result)
  return tuple(result)


def align(vessel, pitch, yaw, roll=None, stop_rotation=True, last=None):
  flight = vessel.flight(vessel.surface_reference_frame)
  current = {
    'time': time.time(),
    'heading': (flight.pitch, flight.heading, flight.roll),
    'error': (0, 0, 0),
    'error_sum': (0, 0, 0),
    'kp': 1,
    'kd': 1,
    'ki': 1,
    'steering': (0, 0, 0),
  }
  if last is None:
    return current
  if stop_rotation or roll is None:
    roll = current['heading'][2]

  target_heading = (pitch, yaw, roll)

  # Potentially adjust constants.
  current['kp'] = last['kp']
  current['kd'] = last['kd']
  current['ki'] = last['ki']

  current['error'] = tuple(np.subtract(target_heading, current['heading']))
  d = np.subtract(current['heading'], last['heading'])
  current['error_sum'] = tuple(np.add(current['error_sum'], current['error']))
  if stop_rotation:
    current['error'] = (current['error'][0], current['error'][1], 0)

  current['steering'] = tuple(np.divide(np.add(current['error'], current['error_sum']) + d, 360))
  return current

#  current['sum']['pitch'] -= current['pitch']
#  p = (target['pitch'] - current['pitch']) * current['kd']
#  delta += (current['pitch'] - last['pitch']) * current['kp']
#  delta -= current['error_sum']['pitch'] * current['ki']
#  delta = delta - 360 if delta > 180 else delta + 360 if delta < 180 else delta
#
#  pitch_delta = (last['pitch'] - 180)
#  current['error'] = (0, 0, 0)
#  current['steering'] = (0, 0, 0)
#  return current


def align2(ship, target_vec, stop_rotation=False, rotation_vec=None, last=None, transform_direction=None):
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
      'yaw': 0,
      'pitch': 0,
      'roll': 0,
    }

  ship_ref = ship.reference_frame
  surf_ref = ship.surface_reference_frame

  ship_up = transform_direction((0, 0, -1), ship_ref, surf_ref)
  ship_right = transform_direction((1, 0, 0), ship_ref, surf_ref)
  ship_forward = ship.direction(surf_ref)
  av = ship.angular_velocity(surf_ref)

  # swap y and z to conform to [pitch, yaw, roll]
  av = (av[0], -av[2], av[1])
  last['av'] = av
  angular_vec = norm(add(mul(av[0], ship_up), mul(av[1], ship_right)))

  p_delta = sub(target_vec, mul(vdot(ship_forward, target_vec), ship_forward))
  y_delta = sub(target_vec, mul(vdot(ship_up, target_vec), ship_up))
  comb = add(p_delta, y_delta)
  error_vec = norm(sub(comb, mul(vdot(ship_forward, comb), ship_forward)))

  Tf = last['Tf']
  Kp = 0.25 * Tf
  Kd = 1 * Tf
  angular_vec = mul(math.sqrt(av[0] * av[0] + av[1] * av[1]) * Kd, angular_vec)
  magnitude = np.arccos(vdot(norm(target_vec), ship_forward)) * Kp
  last['magnitude'] = magnitude
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
  #steering = array([p_angle, y_angle, r_delta])
  #steering = np.nan_to_num(steering)
  #ship.control.pitch = p_angle
  #ship.control.yaw = -y_angle
  #ship.control.roll = r_delta
  #set_steering(ship, steering)
  #last['av'] = angular_vec
  last['steering'] = (-p_angle, y_angle, r_delta)
  return last


class PreLaunch(object):
  def update(self, connection):
    vessel = connection.space_center.active_vessel
    self.yolo = vessel.situation
    if vessel.situation != connection.space_center.VesselSituation.pre_launch:
      return AtmosphericAscent(connection)
    return self

  def display(self):
    return ['Pre Launch.', self.yolo]


class AtmosphericAscent(object):
  def __init__(self, connection):
    self.atm = 1
    self.drag = []
    self.last = None
    self.last_time = time.time()
    self.sc = connection.space_center
    self.ship = self.sc.active_vessel
    self.transform_direction = self.sc.transform_direction
    self.ship_ref = self.ship.reference_frame
    self.surf_ref = self.ship.surface_reference_frame
    self.control = self.ship.control

  def align(self, target_vec, stop_rotation=False, last=None):
    current = {
      'time': time.time(),
      'error': (0, 0, 0),
      'delta': (0, 0, 0),
      'error_sum': (0, 0, 0),
      'kp': 1,
      'kd': 3,
      'ki': 0.1,
      'steering': (0, 0, 0),
    }
    if last is None:
      return current

    # Potentially adjust constants.
    current['kp'] = last['kp']
    current['kd'] = last['kd']
    current['ki'] = last['ki']

    ship_forward = (0, 1, 0)
    ship_right = (1, 0, 0)
    ship_up = (0, 0, -1)

    pitch = vdot(ship_up, target_vec)
    yaw = vdot(ship_right, target_vec)
    behind = vdot(ship_forward, target_vec)
    if behind < 0:
      s = (abs(pitch) + abs(yaw)) / 2
      pitch /= s
      yaw /= s

    p_angle, y_angle, r_angle = (pitch, yaw, 0)

    av = self.ship.angular_velocity(self.surf_ref)
    av = (-av[0], -av[2], av[1])

    current['error'] = mul(current['kp'], (p_angle, y_angle, r_angle))
    current['delta'] = tuple(mul(current['kd'], av))
    current['error_sum'] = (0, 0, 0) if 'error_sum' not in current else tuple(
      mul(current['ki'], np.add(current['error_sum'], current['error'])))
    if not stop_rotation:
      current['delta'] = (current['delta'][0], current['delta'][1], 0)

    steering = tuple(
      np.add(current['error'], current['delta']) + current['error_sum'])
    current['steering'] = (float(steering[0]),
                           float(steering[1]),
                           float(steering[2]))
    return current

  def update(self, connection):
    vec = self.transform_direction(h2v(0, 0), self.surf_ref, self.ship_ref)
    self.last = self.align(vec, stop_rotation=True, last=self.last)

    delta_time = [time.time() - self.last_time]
    self.last_time = time.time()

    #connection.space_center.draw_direction(h2v(0, 0), self.surf_ref, (1, 0, 0))
    error = self.last['error']
    delta = self.last['delta']
    error_sum = self.last['error_sum']
    steering = self.last['steering']
    self.drag = [delta_time,
                 'error:',
                 '%.6f' % error[0], '%.6f' % error[1], '%.6f' % error[2],
                 'delta:',
                 '%.6f' % delta[0], '%.6f' % delta[1], '%.6f' % delta[2],
                 'error_sum:',
                 '%.6f' % error_sum[0], '%.6f' % error_sum[1], '%.6f' % error_sum[2],
                 'steering:',
                 '%.6f' % steering[0], '%.6f' % steering[1], '%.6f' % steering[2],
                 ]
    if 'steering' in self.last:
      self.control.pitch = float(self.last['steering'][0])
      self.control.yaw = float(self.last['steering'][1])
      self.control.roll = float(self.last['steering'][2])
      pass
    return self
    #  self.drag = [delta_time, self.last['steering'], self.last['p_angle'], self.last['y_angle'], self.last['dp_angle'], self.last['dy_angle']]
    #ship_forward = connection.space_center.transform_direction((0, 1, 0), vessel.reference_frame, vessel.surface_reference_frame)
    #connection.space_center.draw_direction(ship_forward, vessel.surface_reference_frame, (1,1,0))
    #vessel.auto_pilot.disengage()

    #self.last = align(vessel, heading2vector(vessel, 90, 180), last=self.last)
    #if mission_time > 
    #if self.drag > 0.5:
    #  vessel.auto_pilot.target_pitch_and_heading(90, 180)
    #else:
    #  drag_delta = self.drag - last_drag
    #  vessel.auto_pilot.target_pitch_and_heading(90 - drag_delta, 180)
    return self

  def display(self):
    return ['AtmosphericAscent', self.atm] + self.drag


# DECLARE_KRCC_MODULE
def load(root):
  return Mjolnir3(root)


class Mjolnir3(KRCCModule):
  def __init__(self, root):
    super().__init__()
    self.root = root
    self.exception = None

    self.list_string = StringVar()
    self.listbox = Listbox(root, listvariable=self.list_string,
                           font='TkFixedFont', width=300)

    self.load()

  def establish_connection_and_run(self):
    error = None
    dots = 0
    connection = None
    while not self.terminate:
      try:
        if connection is None:
          connection = krpc.connect(name=self.name)
        self.run_with_connection(connection)
        error = None
        dots = 0
      except Exception as e:
        if error != e.args[0]:
          error = e.args[0]
          print('\n')
          print(traceback.format_exc())
          sys.stdout.write('Retrying...\n')
        if dots > 80:
          dots = 0
          sys.stdout.write('\n')
        sys.stdout.write('.')
        dots += 1
        sys.stdout.flush()
        time.sleep(1)
    if connection is not None:
      connection.close()

  def run_with_connection(self, connection):
    logging.debug('KRPC connection established')
    strategy = PreLaunch()
    while not self.terminate:
      strategy = strategy.update(connection)
      self.list_string.set(tuple(strategy.display()))

  def run(self):
    try:
      self.establish_connection_and_run()
      self.listbox.destroy()
    except RuntimeError:
      # Should only happen when KeyboardInterrupt is thrown in the MainThread.
      pass

  @property
  def name(self):
    return 'Mjolnir 3'

  def load(self):
    self.listbox.pack(side=LEFT, fill=BOTH)
