from tkinter import LEFT, BOTH, IntVar, DoubleVar, \
  Entry, Label, E, Frame, RIGHT, NW, TOP

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, rcParams
from matplotlib.figure import Figure

from krcc_module import KRCCModule


# DECLARE_KRCC_MODULE
def load(root):
  return StaticCalc(root)


g = 9.81


class StaticCalc(KRCCModule):
  def __init__(self, root):
    super().__init__()
    self.root = root

    self.left_frame = Frame(root)
    self.mass_t0_double = DoubleVar()
    self.mass_t0_double.set(100)
    self.mass_t0_double.trace('w', self.plot_values)
    self.mass_t0_label = Label(self.left_frame, text='mass_t0:')
    self.mass_t0_label.grid(row=0, column=0, sticky=E)
    self.mass_t0_entry = Entry(self.left_frame, textvar=self.mass_t0_double)
    self.mass_t0_entry.grid(row=0, column=1)

    self.mass_t1_double = DoubleVar()
    self.mass_t1_double.set(50)
    self.mass_t1_double.trace('w', self.plot_values)
    self.mass_t1_label = Label(self.left_frame, text='mass_t1:')
    self.mass_t1_label.grid(row=1, column=0, sticky=E)
    self.mass_t1_entry = Entry(self.left_frame, textvar=self.mass_t1_double)
    self.mass_t1_entry.grid(row=1, column=1)

    self.thrust_int = IntVar()
    self.thrust_int.set(2000)
    self.thrust_int.trace('w', self.plot_values)
    self.thrust_label = Label(self.left_frame, text='thrust:')
    self.thrust_label.grid(row=2, column=0, sticky=E)
    self.thrust_entry = Entry(self.left_frame, textvar=self.thrust_int)
    self.thrust_entry.grid(row=2, column=1)

    self.burn_duration_double = DoubleVar()
    self.burn_duration_double.set(30)
    self.burn_duration_double.trace('w', self.plot_values)
    self.burn_duration_label = Label(self.left_frame, text='burn_duration:')
    self.burn_duration_label.grid(row=3, column=0, sticky=E)
    self.burn_duration_entry = Entry(self.left_frame,
                                     textvar=self.burn_duration_double)
    self.burn_duration_entry.grid(row=3, column=1)

    self.diameter_double = DoubleVar()
    self.diameter_double.set(0.3)
    self.diameter_double.trace('w', self.plot_values)
    self.diameter_label = Label(self.left_frame, text='Rocket diameter:')
    self.diameter_label.grid(row=5, column=0, sticky=E)
    self.diameter_entry = Entry(self.left_frame, textvar=self.diameter_double)
    self.diameter_entry.grid(row=5, column=1)

    self.left_frame.pack(side=LEFT, anchor=NW)

    self.canvas_frame = Frame(root)
    self.canvas_frame.pack(side=RIGHT, fill=BOTH, expand=True)

    rcParams['font.family'] = 'Liberation Sans'
    rcParams['font.size'] = 10
    rcParams['text.hinting'] = 'native'
    rcParams['text.hinting_factor'] = 1
    rcParams['text.antialiased'] = False
    rcParams['figure.facecolor'] = '#FFFFFF'
    rcParams['axes.linewidth'] = 0.75
    rcParams['xtick.major.width'] = 0.75
    rcParams['xtick.minor.width'] = 0.75
    rcParams['ytick.major.width'] = 0.75
    rcParams['ytick.minor.width'] = 0.75
    figure = Figure(dpi=96)
    figure.subplots_adjust(left=0.05, right=0.99, top=0.96, bottom=0.06)
    self.canvas = FigureCanvasTkAgg(figure, master=self.canvas_frame)
    self.canvas.show()
    self.canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=1)
    self.plot = figure.add_subplot(111)
    # plot.plot(range(-10, 10), [x * x for x in range(-10, 10)])
    self.plot_values()

  def establish_connection_and_run(self):
    pass

  def plot_values(self, _=None, __=None, ___=None):
    if _ is None and __ is None and ___ is None:
      pass
    time = []
    acceleration = []
    velocity = []
    distance = []
    mass = []
    thrust = self.thrust_int.get()
    dry_mass = self.mass_t0_double.get()
    fuel_mass = self.mass_t0_double.get() - self.mass_t1_double.get()
    burn_duration = self.burn_duration_double.get()
    if fuel_mass >= dry_mass or burn_duration <= 0:
      return
    for t in range(5 * 60):
      time.append(t)
      fuel_consumed = min(1, t / burn_duration)
      thrust_accel = thrust / (mass[t - 1] if mass else dry_mass)
      drag_accel = 0
      accel_sum = drag_accel - g + (thrust_accel if fuel_consumed < 1 else 0)
      acceleration.append(accel_sum)
      velocity.append((velocity[t - 1] if velocity else 0) + acceleration[t])
      last_distance = distance[-1] if distance else 0
      dist = last_distance + (velocity[t] + velocity[t - 1]) / 2
      distance.append(max(0, dist))
      mass.append(dry_mass - fuel_consumed * fuel_mass)
    self.plot.clear()
    self.plot.plot(time, distance)
    self.canvas.draw()

  def run(self):
    try:
      while not self.terminate:
        pass
    except RuntimeError:
      # Should only happen when KeyboardInterrupt is thrown in the MainThread.
      pass
    finally:
      for child in self.root.winfo_children():
        child.destroy()

  @property
  def name(self):
    return 'Static Calc'
