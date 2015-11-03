import time
import os
import threading
import importlib
from tkinter import *
from tkinter import ttk
import tkinter.font


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


def on_combobox_changed():
  loader.start_module(combobox.get())


def on_button_clicked():
  loader.reload_module()


class KRCCModuleLoader:
  def __init__(self, root):
    self._root = root
    self._lock = threading.Lock()
    self._py_module = None
    self._module = None
    self._module_name = None
    self._module_thread = None
    self._shutdown = False
    self._thread = threading.Thread(target=self._execute_module, name='_thread')
    self._thread.start()
    self._file_thread = threading.Thread(target=self._watch_file,
                                         name='_file_thread')
    self._file_thread.start()

  def _execute_module(self):
    error = None
    while True:
      try:
        self._lock.acquire()
        if self._module_name is None and self._py_module is None:
          continue
        if self._py_module is None:
          self._py_module = importlib.import_module(self._module_name)
        else:
          self._py_module = importlib.reload(self._py_module)
          self._module_name = self._py_module.__name__.split('/')[-1]
          self._module_name = self._module_name.split('.')[0]
        self._module = self._py_module.load(self._root)
        print('\nStarting thread with module: %s' % self._module.name)
        self._module_thread = threading.Thread(target=self._module.run,
                                               name='_module_thread')
        self._module_thread.start()
        error = None
        self._lock.release()
        self._module_thread.join()
        self._lock.acquire()
        print('\nModule %s finished executing.' % self._module.name)
        if self._shutdown:
          self._module_name = None
          return
      except Exception as e:
        if error != e.args[0]:
          error = e.args[0]
          print('\n')
          print(e)
          self._module_name = None
          sys.stdout.write('Retrying')
        sys.stdout.write('.')
        sys.stdout.flush()
        time.sleep(1)
      finally:
        self._lock.release()

  def _watch_file(self):
    watched_file = None
    while watched_file is None:
      with self._lock:
        if self._shutdown:
          return
        if self._module_name is not watched_file:
          watched_file = self._module_name
      if watched_file is None:
        continue
      stats = os.stat(watched_file + '.py')
      mtime = stats.st_mtime
      while True:
        time.sleep(3)
        new_stats = os.stat(watched_file + '.py')
        new_mtime = new_stats.st_mtime
        if new_mtime > mtime:
          self.reload_module()
          mtime = new_mtime
        with self._lock:
          if self._shutdown:
            return
          if self._module_name is None:
            watched_file = None
            break

  def shutdown(self):
    with self._lock:
      if self._shutdown:
        return
      self._shutdown = True
    self.stop_module()
    tk.update()
    while self._thread.isAlive():
      self._thread.join(0.01)
      tk.update()
    while self._file_thread.isAlive():
      self._file_thread.join(0.01)
      tk.update()

  def start_module(self, name):
    with self._lock:
      if self._shutdown:
        return
      if self._module_name is None:
        self._module_name = name

  def stop_module(self):
    with self._lock:
      if self._module_name is not None:
        self._module.terminate = True
        self._module_name = None

  def reload_module(self):
    with self._lock:
      if self._shutdown:
        return
      name = self._module_name
    self.stop_module()
    self.start_module(name)


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


loader = KRCCModuleLoader(module_frame)
loader.start_module(krcc_modules[0])
button['command'] = loader.reload_module


def on_shutdown():
  loader.shutdown()
  tk.quit()


tk.protocol("WM_DELETE_WINDOW", on_shutdown)
try:
  tk.mainloop()
except KeyboardInterrupt:
  loader.shutdown()
print('Shutdown complete! Have a nice day.')
