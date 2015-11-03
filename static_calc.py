import logging
import krpc
import sys
import time
from ttk import Style, Checkbutton
from krcc_module import KRCCModule
from tkinter import Listbox, Y, END, LEFT, Canvas, RIGHT, BOTH, Button, Label, \
  StringVar


def load(root):
  return StaticCalc(root)


class StaticCalc(KRCCModule):
  def __init__(self, root):
    super().__init__()
    self.root = root
    self.vars = []
    self.enable_logging_checkbox = Checkbutton(root)
    self.list_string = StringVar()
    self.listbox = Listbox(root, listvariable=self.list_string,
                           font='TkFixedFont', width=30)
    self.canvas = Canvas(root)
    self.load()

  def __del__(self):
    self.canvas.destroy()
    self.listbox.destroy()

  @staticmethod
  def maybe_open_logfile():
    return open('yolo.log', 'w+')
    #return open('{}.log'.format(time.time()), 'w')

  def establish_connection_and_run(self):
    message = None
    while not self.terminate:
      try:
        with krpc.connect(name=self.name) as connection:
          self.run_with_connection(connection)
      except (krpc.error.NetworkError, krpc.error.RPCError) as e:
        if message != e.args[0]:
          message = e.args[0]
          print('\n' + message)
          sys.stdout.write('Reconnecting')
        sys.stdout.write('.')
        sys.stdout.flush()
        time.sleep(1)

    #except krpc.error.RPCError as e:
    #  print('KRPC error: {}. Reconnecting in 3 sec...'.format(e))
    #  time.sleep(3)
    logging.debug('KRPC connection closed')

  def run_with_connection(self, connection):
    logging.debug('KRPC connection established')
    vessel = connection.space_center.active_vessel
    ref = vessel.surface_reference_frame
    flight = connection.add_stream(vessel.flight, ref)
    floats = [
        'mean_altitude',
        'atmosphere_density',
        'ballistic_coefficient',
        'drag_coefficient',
    ]
    vectors = [
        'velocity',
    ]
    colon_pos_float = max([len(v) for v in floats])
    colon_pos_vec = max([len(v) + 3 for v in vectors])
    self.listbox.configure(width=max(colon_pos_float, colon_pos_vec) + 11)

    #while not self.terminate:
    #  print('Run func... yolo: 1')
    #  time.sleep(1)
    #self.terminate = True
    #return
    streams = []
    #for name, func in var:
    #  print(name)
    #  print(func)
    #  streams.append(connection.add_stream(func))

    with self.maybe_open_logfile() as f:
      # Write the log file header.
      f.write('time\t' + '\t'.join(floats) + '\t')
      s = '\t'.join('{{}}[{}]'.format(x) for x in [0, 1, 2])
      f.write('\t'.join(s.format(*(v for _ in [0, 1, 2])) for v in vectors))
      f.write('\n')
      log_sample_interval = 0.01
      next_log_sample = time.time()
      fl = flight()
      while not self.terminate:
        values = [time.time()]
        strings = []
        for name in floats:
          value = flight().__getattribute__(name)
          values.append(value)
          padding = colon_pos_float - len(name) + 9
          format_string = '{{}}: {{:>{}.3f}}'.format(padding)
          strings.append(format_string.format(name, value))
        for name in vectors:
          value = flight().__getattribute__(name)
          padding = colon_pos_vec - len(name) + 2
          format_string = '{{}}[{{}}]: {{:>{}.3f}}'.format(padding)
          for i in [0, 1, 2]:
            values.append(value[i])
            strings.append(format_string.format(name, i, value[i]))
        if time.time() > next_log_sample:
          f.write('\t'.join(['{}'.format(v) for v in values]) + '\n')
          next_log_sample = time.time() + log_sample_interval
        self.list_string.set(tuple(strings))

  def run(self):
    self.establish_connection_and_run()
    self.enable_logging_checkbox.destroy()
    self.listbox.destroy()
    self.canvas.destroy()

  @property
  def name(self):
    return 'Static Calc'

  def load(self):
    self.listbox.pack(side=LEFT, fill=BOTH)
    self.enable_logging_checkbox.pack(side=LEFT)
