import time
import os
import sys
import threading
import importlib
# import mttkinter as tkinter
import traceback
from tkinter import ttk
import tkinter.font
from watchdog.observers import Observer
from watchdog.events import LoggingEventHandler, FileSystemEventHandler
import krpc

import krcc

KRCC_MODULE_DECLARATION = 'DECLARE_' + 'KRCC' + '_MODULE'
krcc_modules = []
def load_krcc_modules():
  for dirpath, _, filenames in os.walk(os.getcwd()):
    for filename in filenames:
      if not filename.endswith('.py'):
        continue
      with open(os.path.join(dirpath, filename), 'r') as f:
        for line in f.readlines():
          if KRCC_MODULE_DECLARATION in line:
            krcc_modules.append(filename[:-3])
load_krcc_modules()

def on_combobox_changed(event):
  loader.start_module(combobox.get())


def on_button_clicked(event):
  loader.reload_module()


class KRCCModuleLoader(FileSystemEventHandler):
  def __init__(self, root):
    self._root = root
    self._lock = threading.Lock()
    self._py_module = None
    self._module = None
    self._module_name = tkinter.StringVar()
    self._module_thread = None
    self._shutdown = False
    self._dirty = False
    self._run = None
    self._state = None
    self._connection = None
    self._thread = threading.Thread(target=self._execute_module,
                                    name='krcc_thread',
                                    daemon=True)
    self._thread.start()

  def on_modified(self, event):
    with self._lock:
      if event.src_path == './' + self._module_name + '.py':
        print('Reloading module "%s"...' % self._module_name)
        try:
          if self._py_module is None:
            self._py_module = importlib.import_module(self._module_name)
          else:
            importlib.reload(self._py_module)
          self._dirty = False
        except Exception:
          self._print_trace_and_wait_for_refresh()

  def request_module(self, module_name: str):
    with self._lock:
      self._module_name.set(module_name)

  def _print_trace_and_wait_for_refresh(self):
    sys.stdout.flush()
    traceback.print_exc()
    time.sleep(0.1)
    sys.stdout.flush()
    sys.stderr.flush()
    self._wait_for_reload()

  def _wait_for_reload(self):
    self._dirty = True
    self._run = self._run_wait_for_reload
    print('Waiting for change in %s.py...' % self._module_name, flush=True)
    return True

  def _run_wait_for_reload(self):
    with self._lock:
      if not self._dirty:
        self._run = self._run_verify_module
    return True

  def _run_load_new_module(self):
    self._py_module = importlib.import_module(self._module_name)
    if self._py_module is None:
      self._wait_for_reload()
    else:
      self._run = self._run_verify_module
    return True

  def _run_verify_module(self):
    with open(self._module_name + '.py') as f:
      #from pprint import pprint
      #pprint(f.readlines())
      if not any(line.startswith(krcc.Signature) for line in f.readlines()):
        print('No function with signature %s in "%s"'
              % (krcc.Signature, self._module_name))
        self._wait_for_reload()
        return True
    self._run = self._run_module_file
    return True

  def _run_module_file(self):
    self._state = self._py_module.execute(self._state, self._connection)
    return True

  def _main(self, connection: krpc.Connection):
    self._run = self._run_load_new_module
    self._connection = connection
    next_check = 0
    while time.time() < next_check or connection.krpc.get_status() is not None:
      if time.time() > next_check:
        next_check = time.time() + 1
      try:
        if not self._run():
          return False
      except Exception:
        self._print_trace_and_wait_for_refresh()
    print('Lost kRPC connection.')
    return True

  def _connect_to_krpc(self):
    with krpc.connect(name="KRCC") as connection:
      observer = Observer()
      observer.schedule(self, '.')
      observer.start()
      print('Connected to kRPC')
      try:
        #self._py_module = importlib.import_module(self._module_name)
        self._main(connection)
      finally:
        observer.stop()
      print('Disconnected from kRPC')
      return False

  def _execute_module(self):
    error = None
    dots = 0
    result = True
    while result:
      try:
        result = self._connect_to_krpc()
      except krpc.error.NetworkError as e:
        if error != e.args[0]:
          error = e.args[0]
          print('\n')
          print(e)
          msg = 'Could not connect to kRPC. Retrying'
          sys.stdout.write(msg)
          dots = len(msg)
        if dots > 80:
          dots = 0
          sys.stdout.write('\n')
        dots += 1
        sys.stdout.write('.')
        sys.stdout.flush()
        time.sleep(1)
      except KeyboardInterrupt:
        print(e)
        return
      except Exception as e:
        print(e)
        return


      #        self._lock.acquire()
      #        if self._module_name is None and self._py_module is None:
      #          continue
      #        if self._py_module is None:
      #          self._py_module = importlib.import_module(self._module_name)
      #        else:
      #          self._py_module = importlib.reload(self._py_module)
      #          self._module_name = self._py_module.__name__.split('/')[-1]
      #          self._module_name = self._module_name.split('.')[0]
      #        self._module = self._py_module.load(self._root)
      #        print('\nStarting thread with module: %s' % self._module.name)
      #        self._module_thread = threading.Thread(target=self._module.run,
      #                                               name='_module_thread')
      #        self._module_thread.start()
      #        error = None
      #        self._lock.release()
      #        self._module_thread.join()
      #        self._lock.acquire()
      #        print('\nModule %s finished executing.' % self._module.name)
      #        if self._shutdown:
      #          self._module_name = None
      #          return
      #    except Exception as e:
      #      print(e)
      #        if error != e.args[0]:
      #          error = e.args[0]
      #          print('\n')
      #          print(e)
      #          self._module_name = None
      #          sys.stdout.write('Retrying')
      #        if dots > 80:
      #          dots = 0
      #          sys.stdout.write('\n')
      #        sys.stdout.write('.')
      #        dots += 1
      #        sys.stdout.flush()
      #        time.sleep(1)
      #    finally:
      #      print("yolo")
      #        self._lock.release()

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
      if self._module_name != name:
        self._module_name = name
        if self._module is not None:
          self._module.terminate = True
          self._py_module = None

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
    # self.start_module(name)

  def _open_mudule(self):
    while True:
      try:
        pass
      finally:
        pass

  def is_alive(self):
    return True
    #self._thread.join(0.1)
    #return self._thread.is_alive()



tk = tkinter.Tk()
tk.title('KRCC')
tk.geometry('1200x450+2180+550')
s = ttk.Style()
s.theme_use('clam')
s.configure('TButton', padding=(0, 1, 0, 1))
for font in tkinter.font.names():
  tkinter.font.nametofont(font).configure(family='Liberation Sans')
tkinter.font.nametofont('TkFixedFont').configure(family='Liberation Mono')

app = ttk.Frame(tk)
app.pack(side=tkinter.TOP, fill=tkinter.X)

button = ttk.Button(app)
button['text'] = "reload"
button.pack(side=tkinter.RIGHT)
button.bind('<Button-1>', on_button_clicked)

should_auto_reload = tkinter.BooleanVar()
should_auto_reload.set(True)
auto_reload_checkbutton = ttk.Checkbutton(app, var=should_auto_reload)
auto_reload_checkbutton['text'] = 'Automatically reload'
auto_reload_checkbutton.pack(side=tkinter.RIGHT)

combobox = ttk.Combobox(app)
combobox['state'] = 'readonly'
combobox['values'] = krcc_modules
combobox.set(krcc_modules[0])
combobox.pack(side=tkinter.RIGHT)
combobox.bind('<<ComboboxSelected>>', on_combobox_changed)

module_frame = ttk.Frame(tk)
module_frame.pack(fill=tkinter.BOTH, expand=1)

loader = KRCCModuleLoader(module_frame)
loader.start_module(krcc_modules[0])
button['command'] = loader.reload_module


def check():
  if not loader.is_alive():
    tk.quit()
  tk.after(100, check)


tk.after(50, check)

try:
  tk.mainloop()
except KeyboardInterrupt:
  pass

print('\nShutdown complete! Have a nice day.')
