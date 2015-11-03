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


# DECLARE_KRCC_MODULE
def load(root):
  return AvionicsLogger(root)


class AvionicsLogger(KRCCModule):
  def __init__(self, root):
    super().__init__()
    self.root = root
    self.exception = None

    self.list_string = StringVar()
    self.listbox = Listbox(root, listvariable=self.list_string,
                           font='TkFixedFont', width=30)

    self.write_cache = ''
    self.logfile = None
    self.enable_logging = BooleanVar()
    self.enable_logging_checkbox = Checkbutton(
      root, var=self.enable_logging, text='Enable logging',
      command=self.enable_logging_changed)

    self.logfile_label = Label(root, text='Logfile name:')
    self.logfile_name = StringVar()
    self.logfile_name_entry = Entry(root, textvar=self.logfile_name)

    self.load()

  def write(self, string):
    if self.enable_logging.get() and self.logfile is None:
      if self.logfile_name.get() == '':
        self.logfile_name.set('logs/{}.log'.format(time.time()))
      self.logfile = io.open(self.logfile_name.get(), 'a')
      self.logfile.write(self.write_cache)
      self.write_cache = ''
    self.logfile.write(string)

  def cache(self, string):
    self.write_cache += string

  def enable_logging_changed(self):
    if not self.enable_logging.get():
      self.logfile_name_entry.configure(state=NORMAL)
      if self.logfile is not None:
        self.logfile.close()
        self.logfile = None
        self.logfile_name.set('')
    else:
      self.logfile_name_entry.configure(state=DISABLED)

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
          sys.stdout.write('Retrying')
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
    vessel = connection.space_center.active_vessel
    ref = vessel.orbit.body.reference_frame
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

    # Write the log file header.
    self.cache('time\t' + '\t'.join(floats) + '\t')
    s = '{}\t' + '\t'.join('{{}}[{}]'.format(x) for x in [0, 1, 2])
    self.cache('\t'.join(s.format(*(v for _ in [0, 1, 2, 3])) for v in vectors))
    self.cache('\n')
    log_sample_interval = 0.01
    next_log_sample = time.time()
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
        magnitude = value[0]
        padding = colon_pos_float - len(name) + 9
        format_string = '{{}}: {{:>{}.3f}}'.format(padding)
        strings.append(format_string.format(name, magnitude))
        values.append(magnitude)
        padding = colon_pos_vec - len(name) + 2
        format_string = '{{}}[{{}}]: {{:>{}.3f}}'.format(padding)
        for i in [0, 1, 2]:
          values.append(value[i])
          strings.append(format_string.format(name, i, value[i]))
      if self.enable_logging.get() and time.time() > next_log_sample:
        self.write('\t'.join(['{}'.format(v) for v in values]) + '\n')
        next_log_sample = time.time() + log_sample_interval
      self.list_string.set(tuple(strings))

  def run(self):
    try:
      self.establish_connection_and_run()
      self.logfile_name_entry.destroy()
      self.logfile_label.destroy()
      self.enable_logging_checkbox.destroy()
      self.listbox.destroy()
    except RuntimeError:
      # Should only happen when KeyboardInterrupt is thrown in the MainThread.
      pass
    if self.logfile is not None:
      self.logfile.close()

  @property
  def name(self):
    return 'Avionics Logger'

  def load(self):
    self.listbox.pack(side=LEFT, fill=BOTH)
    self.logfile_label.pack(side=LEFT, anchor=NW)
    self.logfile_name_entry.pack(side=LEFT, anchor=NE, fill=X, expand=True)
    self.enable_logging_checkbox.pack(side=LEFT, anchor=NW)
