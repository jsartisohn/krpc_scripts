import logging
import math
import sys
import time
import traceback
from pprint import pprint
from tkinter import Listbox, LEFT, BOTH, StringVar

import krpc
import numpy as np

import krcc
from krcc_module import KRCCModule
from mjolnir2 import align, rotation_matrix, mul, vdot


g = 9.81

def is_pre_launch_situation(s, c):
  return s in [
    c.space_center.VesselSituation.pre_launch,
    c.space_center.VesselSituation.landed,
  ]


def execute(state: krcc.State, connection: krpc.Connection):
  try:
    if state is None:
      state = {
        'current_game_scene': connection.add_stream(getattr, connection.krpc, 'current_game_scene'),
        'next_heartbeat': 0,
        'streams': [],
        'updated': False,
      }
    if time.time() > state['next_heartbeat']:
      state['next_heartbeat'] = time.time() + 2
      #print('=' * 80)
      if state['current_game_scene']() == connection.krpc.GameScene.flight:
        if 'flight' not in state:
          state['flight'] = {
            'program': prepare_for_launch,
          }
        elif state['updated']:
          state['flight']['program'] = prepare_for_launch
          for stream in state['streams']:
            stream.remove()
          state['streams'] = []
          state['updated'] = False
        return state['flight']['program'](state, connection)
      if 'flight' in state:
        del state['flight']
      print('Not in flight.')
      state['next_heartbeat'] = time.time() + 5
  except krpc.error.RPCError as e:
    print(e)
  return state


def prepare_for_launch(state: krcc.State, c: krpc.Connection):
  if 'flight' not in state:
    return state
  flight = state['flight']
  if 'active_vessel' not in flight:
    flight['active_vessel'] = c.space_center.active_vessel
  vessel = flight['active_vessel']
  if vessel.name != 'PT4':
    return state
  vessel.control.throttle = 1
  flight['engines'] = []
  engines = flight['engines']
  for engine in vessel.parts.engines:
    if engine.part.title == 'LR89 Series':
      stream = c.add_stream(getattr, engine, 'part')
      state['streams'].append(stream)
      engines.append(stream)
  flight['program'] = await_launch
  return state


def await_launch(state: krcc.State, c: krpc.Connection):
  flight = state['flight']
  vessel = flight['active_vessel']
  if is_pre_launch_situation(vessel.situation, c):
    percentages = []
    for engine in flight['engines']:
      e = engine().engine
      if e is not None:
        percentages.append(e.thrust / e.max_thrust)
    if any(x > 0.01 for x in percentages):
      state['next_heartbeat'] = 0
      print(['%.3f' % x for x in percentages])
    if all(x > 0.99 for x in percentages):
      flight['program'] = check_twr_and_release_launch_clamps
  return state


def check_twr_and_release_launch_clamps(state: krcc.State, c: krpc.Connection):
  flight = state['flight']
  vessel = flight['active_vessel']
  twr = vessel.thrust / (vessel.mass * g)
  print('TWR: %s.5' % twr)
  print(vessel.situation)
  if twr > 1 and is_pre_launch_situation(vessel.situation, c):
    vessel.control.activate_next_stage()
    flight['program'] = open_loop_ascent
    return state
  if vessel.met > 5:
    flight['program'] = disable_engines_and_abort_launch
  return state

def disable_engines_and_abort_launch(state: krcc.State, c: krpc.Connection):
  flight = state['flight']
  vessel = flight['active_vessel']
  for engine in flight['engines']:
      e = engine().engine
      if e is not None:
        e.active = False
  vessel.control.throttle = 0
  vessel.control.abort = True
  flight['program'] = display_aborted_message
  return state

def display_aborted_message(state: krcc.State, c: krpc.Connection):
  print('Launch aborted')
  state['next_heartbeat'] = time.time() + 10
  return state

def open_loop_ascent(state: krcc.State, c: krpc.Connection):
  flight = state['flight']
  vessel = flight['active_vessel']
  ap = vessel.auto_pilot
  if 'open_loop_ascent' not in flight:
    ap.reference_frame = vessel.surface_reference_frame
    ap.engage()
    ap.target_direction = (1,0,0)
    flight['open_loop_ascent'] = {
      'initial_twr': vessel.thrust / (vessel.mass * g),
    }
  ola = flight['open_loop_ascent']
  fraction = max(0, min(1, (ola['initial_twr'] - 1.2) / (1.6 - 1.2)))
  target_speed = 50 + 50 * fraction
  print(vessel.flight(vessel.orbit.body.reference_frame).vertical_speed)
  if vessel.flight(vessel.orbit.body.reference_frame).vertical_speed > target_speed:
    del flight['open_loop_ascent']
    return do_initial_pitchover(state, c, 2 + 3 * fraction)
  return state

def do_initial_pitchover(state: krcc.State, c: krpc.Connection, pitch=None):
  flight = state['flight']
  vessel = flight['active_vessel']
  if pitch is not None:
    flight['program'] = do_initial_pitchover
    flight['initial_pitch'] = pitch
    ap = vessel.auto_pilot
    flight['initial_pitch_ap'] = ap
    ap.reference_frame = vessel.surface_reference_frame
    ap.engage()
    ap.target_pitch_and_heading(90 - pitch, 90)
  ap = flight['initial_pitch_ap']
  print(ap.error)
  if ap.error < 0.5:
    ap.disengage()
    del flight['initial_pitch_ap']
    flight['initial_pitch'] = None
    flight['program'] = follow_prograde
  return state

def follow_prograde(state: krcc.State, c: krpc.Connection):
  flight = state['flight']
  vessel = flight['active_vessel']
  if 'follow_prograde_ap' not in flight:
    flight['follow_prograde_ap'] = vessel.auto_pilot
    flight['follow_prograde_ap'].reference_frame = vessel.surface_velocity_reference_frame
    flight['follow_prograde_ap'].engage()
    flight['follow_prograde_ap'].target_direction = (0,1,0)
    #flight['follow_prograde_ap'].set_pid_parameters(ki=1.2)
  if vessel.flight().surface_altitude > 20000:
    flight['program'] = disable_engines_and_abort_launch
  return state

def template(state: krcc.State, c: krpc.Connection):
  flight = state['flight']
  vessel = flight['active_vessel']
  return state



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
    if self.space_center.ut - self.mission_start > 2 * 60 + 53:
      return SeparateToInsertionStage(self.connection, self.mission_start)
    vec = self.transform_direction(self.ship.flight(self.surf_ref).prograde,
                                   self.surf_ref, self.ship_ref)
    self.last = align(self.ship, vec, last=self.last)
    # self.space_center.clear_drawing()
    self.space_center.draw_direction(vec, self.ship_ref, (1, 0, 0))
    # self.control.pitch = float(self.last['steering'][0])
    # self.control.yaw = float(self.last['steering'][1])
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
    if self.space_center.ut - self.mission_start > 2 * 60 + 53:
      return self  # InsertIntoOrbit(self.connection)
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
