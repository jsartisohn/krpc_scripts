import logging
import sys
import time
import traceback
from tkinter import Listbox, LEFT, BOTH, StringVar

import krpc

from krcc_module import KRCCModule
from mjolnir2 import align


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
#    if self.space_center.ut - self.mission_start > 10:
#      return InclineToTargetDirection(self.connection, self.mission_start)
#    vec = self.transform_direction(h2v(90, 0), self.surf_ref, self.ship_ref)
#    self.last = align(self.ship, vec, last=self.last)
#    self.control.pitch = float(self.last['steering'][0])
#    self.control.yaw = float(self.last['steering'][1])
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

#  def update(self):
#    if self.space_center.ut - self.module_start > 3:
#      return GravityTurn(self.connection, self.mission_start)
#    vec = self.transform_direction(h2v(89, 190), self.surf_ref, self.ship_ref)
#    self.last = align(self.ship, vec, last=self.last)
#    self.control.pitch = float(self.last['steering'][0])
#    self.control.yaw = float(self.last['steering'][1])
#    return self

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
  return Scarlet(root)


class Scarlet(KRCCModule):
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
    state = None
    strategy = PreLaunch(connection)
    while not self.terminate:
      strategy = strategy.update(state)
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
    return 'Scarlet'

  def load(self):
    self.listbox.pack(side=LEFT, fill=BOTH)
