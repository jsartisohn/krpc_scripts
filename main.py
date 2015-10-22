import time
import sys
import os
import math
import numpy as np
import threading
import operator
#import Tkinter as Tk
import importlib
from tkinter import *
from tkinter import ttk
import tkinter.font


import matplotlib

matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends.backend_tkagg import NavigationToolbar2TkAgg
from matplotlib.backend_bases import key_press_handler

terminate_event = threading.Event()
restart_event = threading.Event()
restart_event.clear()
error_event = threading.Event()
error_event.clear()
terminate = False


def reload_script():
  global restart_event
  global script
  restart_event.set()
  error_event.clear()
  if not script:
    threading.Thread(target=reload_thread).start()


f = Figure(figsize=(9, 4), dpi=100)

#import sp1
#import mjolnir2
#import staedler1
#import tenacity1
import tenacity2

script = None


def reload_thread():
  global script
  global restart_event
  global terminate
  #while not terminate and not error_event.isSet():
  #  importlib.reload(tenacity2)
  #  restart_event.clear()
  #  script = threading.Thread(target=tenacity2.update,
  #                            )#args=(var, restart_event, error_event, f))
  #  print('starting thread')
  #  script.start()
  #  script.join()
  #  print('finishing thread')
  script = None


KRCC_MODULE_DECLARATION = 'DECLARE_' + 'KRCC' + '_MODULE'
krcc_modules = []
for dirpath, _, filenames in os.walk(os.getcwd()):
  for filename in filenames:
    if not filename.endswith('.py'):
      continue
    with open(os.path.join(dirpath, filename), 'r') as f:
      for line in f.readlines():
        if KRCC_MODULE_DECLARATION in line:
          krcc_modules.append(filename[:-3])


def on_combobox_changed(event):
  loader.start_module(combobox.get())


def on_button_clicked(event):
  loader.reload_module()


tk = Tk()
tk.title('KRCC')
tk.geometry('1200x450+2180+550')
s = ttk.Style()
s.theme_use('clam')
s.configure('TButton', padding=(0, 1, 0, 1))
for font in tkinter.font.names():
  tkinter.font.nametofont(font).configure(family='Liberation Sans')
tkinter.font.nametofont('TkFixedFont').configure(family='Liberation Mono')


app = ttk.Frame(tk)
app.pack(side=TOP, fill=X)

button = ttk.Button(app)
button['text'] = "reload"
button['command'] = reload_script
button.pack(side=RIGHT)
button.bind('<Button-1>', on_button_clicked)

should_auto_reload = BooleanVar()
should_auto_reload.set(True)
auto_reload_checkbutton = ttk.Checkbutton(app, var=should_auto_reload)
auto_reload_checkbutton['text'] = 'Automatically reload'
auto_reload_checkbutton.pack(side=RIGHT)

combobox = ttk.Combobox(app)
combobox['state'] = 'readonly'
combobox['values'] = krcc_modules
combobox.set(krcc_modules[0])
combobox.pack(side=RIGHT)
combobox.bind('<<ComboboxSelected>>', on_combobox_changed)

module_frame = ttk.Frame(tk)
module_frame.pack(fill=BOTH, expand=1)


class KRCCModuleLoader:
  def __init__(self):
    self.krcc_module = None
    self.lock = threading.RLock()
    self.unloading = threading.Lock()
    self.loading = False

  def _execute_module(self, name):
    # Wait until unloading is finished.
    self.unloading.acquire()
    self.lock.acquire()
    self.krcc_module = importlib.import_module(name).load(module_frame)
    self.krcc_module.id = name
    print('Starting thread with module: %s' % self.krcc_module.name)
    while self.krcc_module is None or not self.krcc_module.terminate:
      t = threading.Thread(target=self.krcc_module.run)
      t.start()
      # Allow unloading from now on.
      #if not self.unloading.acquire(False):
      self.loading = False
      self.unloading.release()
      self.lock.release()
      t.join()
      print('yolo')
      self.lock.acquire()
    print('Module %s finished executing.' % self.krcc_module.name)
    self.krcc_module = None
    # Release lock from the actual stop_module() call.
    self.unloading.release()
    self.lock.release()

  def stop_module(self):
    with self.lock:
      if not self.loading and self.krcc_module is not None:
        self.unloading.acquire()
        self.krcc_module.terminate = True

  def start_module(self, name):
    with self.lock:
      if self.loading:
        return False
      if self.krcc_module is not None and self.krcc_module.id == name:
        return False
      self.stop_module()
      self.loading = True
      threading.Thread(target=KRCCModuleLoader._execute_module, args=(self, name)).start()

  def reload_module(self):
    with self.lock:
      if self.krcc_module is None or self.loading:
        return
      self.stop_module()
      threading.Thread(target=KRCCModuleLoader._reload_module, args=(self, self.krcc_module.id)).start()

  def _reload_module(self, name):
    with self.unloading:
      self.start_module(name)


loader = KRCCModuleLoader()
loader.start_module(krcc_modules[0])


#var = StringVar()
#var.set("")
#label = Label(app, textvariable=var, font=("Liberation Mono", 10),
#                 justify=LEFT)
#label.grid(sticky=W)
#
#canvas_frame = Frame(tk)
#canvas_frame.grid(sticky=E, column=1, row=0)
#
#canvas = FigureCanvasTkAgg(f, master=canvas_frame)
#canvas.show()
#canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=1)
#
#toolbar = NavigationToolbar2TkAgg(canvas, canvas_frame)
#toolbar.update()
#canvas._tkcanvas.pack(side=TOP, fill=BOTH, expand=1)


#def on_key_event(event):
#  print('you pressed %s' % event.key)
#  key_press_handler(event, canvas, toolbar)
#canvas.mpl_connect('key_press_event', on_key_event)
#import tenacity2
#tmp = tenacity2.Tenacity2(module)
#tmp.load()
#
reload_script()
tk.mainloop()
print('terminating')
terminate = True
restart_event.set()
