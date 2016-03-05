import io
import logging
import sys
import time
import traceback
from tkinter import Listbox, LEFT, BOTH, StringVar

import krpc

from krcc_module import KRCCModule
from mjolnir2 import align, rotation_matrix, mul, vdot
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


def align(ship, target_vec, stop_rotation=False, last=None):
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

  av = ship.angular_velocity(ship.surface_reference_frame)
  av = (-av[0], -av[2], av[1])

  current['error'] = mul(current['kp'], (pitch, yaw, 0))
  current['delta'] = tuple(mul(current['kd'], av))
  current['error_sum'] = (0, 0, 0) if 'error_sum' not in current else tuple(
    mul(current['ki'], np.add(current['error_sum'], current['error'])))
  if not stop_rotation:
    current['delta'] = (current['delta'][0], current['delta'][1], 0)

  pd_steering = np.add(current['error'], current['delta'])
  steering = tuple(pd_steering + current['error_sum'])
  current['steering'] = (float(steering[0]),
                         float(steering[1]),
                         float(steering[2]))
  return current


class PreLaunch(object):
  def __init__(self, connection):
    self.connection = connection
    self.vessel = connection.space_center.active_vessel
    self.pre_launch = connection.space_center.VesselSituation.pre_launch

  def update(self):
    if self.vessel.situation != self.pre_launch:
      return AtmosphericAscent(self.connection)
    return self

  def display(self):
    return ['Pre Launch.', self.vessel.situation]


class AtmosphericAscent(object):
  def __init__(self, connection):
    self.connection = connection
    self.space_center = connection.space_center
    self.transform_direction = self.space_center.transform_direction
    self.ship = self.space_center.active_vessel
    self.surf_ref = self.ship.surface_reference_frame
    self.ship_ref = self.ship.reference_frame
    self.control = self.ship.control
    self.mission_start = self.space_center.ut
    self.last = None

  def update(self):
    if self.space_center.ut - self.mission_start > 10:
      return InclineToTargetDirection(self.connection, self.mission_start)
    vec = self.transform_direction(h2v(90, 0), self.surf_ref, self.ship_ref)
    self.last = align(self.ship, vec, last=self.last)
    self.control.pitch = float(self.last['steering'][0])
    self.control.yaw = float(self.last['steering'][1])
    return self

  def display(self):
    return ['AtmosphericAscent', self.mission_start + self.space_center.ut]


class InclineToTargetDirection(object):
  def __init__(self, connection, mission_start):
    self.connection = connection
    self.space_center = connection.space_center
    self.transform_direction = self.space_center.transform_direction
    self.ship = self.space_center.active_vessel
    self.surf_ref = self.ship.surface_reference_frame
    self.ship_ref = self.ship.reference_frame
    self.control = self.ship.control
    self.module_start = self.space_center.ut
    self.mission_start = mission_start
    self.last = None

  def update(self):
    if self.space_center.ut - self.module_start > 3:
      return GravityTurn(self.connection, self.mission_start)
    vec = self.transform_direction(h2v(89, 190), self.surf_ref, self.ship_ref)
    self.last = align(self.ship, vec, last=self.last)
    self.control.pitch = float(self.last['steering'][0])
    self.control.yaw = float(self.last['steering'][1])
    return self

  def display(self):
    return ['InclineToTargetDirection']


class GravityTurn(object):
  def __init__(self, connection, mission_start):
    self.connection = connection
    self.space_center = connection.space_center
    self.transform_direction = self.space_center.transform_direction
    self.ship = self.space_center.active_vessel
    self.surf_ref = self.ship.surface_reference_frame
    self.ship_ref = self.ship.reference_frame
    self.control = self.ship.control
    self.mission_start = mission_start
    self.last = None

  def update(self):
    if self.space_center.ut - self.mission_start > 2*60 + 53:
      return SeparateToInsertionStage(self.connection, self.mission_start )
    vec = self.transform_direction(self.ship.flight(self.surf_ref).prograde, self.surf_ref, self.ship_ref)
    self.last = align(self.ship, vec, last=self.last)
    #self.space_center.clear_drawing()
    self.space_center.draw_direction(vec, self.ship_ref, (1, 0, 0))
    #self.control.pitch = float(self.last['steering'][0])
    #self.control.yaw = float(self.last['steering'][1])
    return self

  def display(self):
    return ['GravityTurn']


class SeparateToInsertionStage(object):
  def __init__(self, connection, mission_start):
    self.connection = connection
    self.space_center = connection.space_center
    self.transform_direction = self.space_center.transform_direction
    self.ship = self.space_center.active_vessel
    self.surf_ref = self.ship.surface_reference_frame
    self.ship_ref = self.ship.reference_frame
    self.control = self.ship.control
    self.mission_start = mission_start
    self.last = None

  def update(self):
    if self.space_center.ut - self.mission_start > 2*60 + 53:
      return self#InsertIntoOrbit(self.connection)
    vec = self.ship.flight().prograde
    self.last = align(self.ship, vec, last=self.last)
    self.control.pitch = float(self.last['steering'][0])
    self.control.yaw = float(self.last['steering'][1])
    return self

  def display(self):
    return ['SeparateToInsertionStage']


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
    strategy = PreLaunch(connection)
    while not self.terminate:
      strategy = strategy.update()
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
