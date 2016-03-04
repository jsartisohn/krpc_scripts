import math
import tkinter
#import mttkinter as tkinter

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, rcParams
from matplotlib.figure import Figure

from krcc_module import KRCCModule


# DECLARE_KRCC_MODULE
def load(root):
  return StaticCalc(root)


g = 9.81


def density(x):
  return -2.686e-24 * pow(x, 5) + 7.626e-19 * pow(x, 4) - 8.422e-14 * pow(x, 3) + 4.535e-9 * pow(x, 2) - 1.188e-4 * x + 1.217


class StaticCalc(KRCCModule):
  def __init__(self, root):
    super().__init__()
    self.root = root

    self.left_frame = tkinter.Frame(root)
    self.mass_t0_double = tkinter.DoubleVar()
    self.mass_t0_double.set(114.713)
    self.mass_t0_double.trace('w', self.plot_values)
    self.mass_t0_label = tkinter.Label(self.left_frame, text='mass_t0:')
    self.mass_t0_label.grid(row=0, column=0, sticky=tkinter.E)
    self.mass_t0_entry = tkinter.Entry(self.left_frame,
                                       textvar=self.mass_t0_double)
    self.mass_t0_entry.grid(row=0, column=1)

    self.mass_t1_double = tkinter.DoubleVar()
    self.mass_t1_double.set(14.997)
    self.mass_t1_double.trace('w', self.plot_values)
    self.mass_t1_label = tkinter.Label(self.left_frame, text='mass_t1:')
    self.mass_t1_label.grid(row=1, column=0, sticky=tkinter.E)
    self.mass_t1_entry = tkinter.Entry(self.left_frame,
                                       textvar=self.mass_t1_double)
    self.mass_t1_entry.grid(row=1, column=1)

    self.thrust_int = tkinter.IntVar()
    self.thrust_int.set(1334.5)
    self.thrust_int.trace('w', self.plot_values)
    self.thrust_label = tkinter.Label(self.left_frame, text='thrust:')
    self.thrust_label.grid(row=2, column=0, sticky=tkinter.E)
    self.thrust_entry = tkinter.Entry(self.left_frame, textvar=self.thrust_int)
    self.thrust_entry.grid(row=2, column=1)

    self.burn_duration_double = tkinter.DoubleVar()
    self.burn_duration_double.set(172.7)
    self.burn_duration_double.trace('w', self.plot_values)
    self.burn_duration_label = tkinter.Label(self.left_frame,
                                             text='burn_duration:')
    self.burn_duration_label.grid(row=3, column=0, sticky=tkinter.E)
    self.burn_duration_entry = tkinter.Entry(self.left_frame,
                                             textvar=self.burn_duration_double)
    self.burn_duration_entry.grid(row=3, column=1)

    self.diameter_double = tkinter.DoubleVar()
    self.diameter_double.set(0.3)
    self.diameter_double.trace('w', self.plot_values)
    self.diameter_label = tkinter.Label(self.left_frame,
                                        text='Rocket diameter:')
    self.diameter_label.grid(row=5, column=0, sticky=tkinter.E)
    self.diameter_entry = tkinter.Entry(self.left_frame,
                                        textvar=self.diameter_double)
    self.diameter_entry.grid(row=5, column=1)

    self.left_frame.pack(side=tkinter.LEFT, anchor=tkinter.NW)

    self.canvas_frame = tkinter.Frame(root)
    self.canvas_frame.pack(side=tkinter.RIGHT, fill=tkinter.BOTH, expand=True)

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
    self.canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH,
                                     expand=1)
    self.plot = figure.add_subplot(111)
    # plot.plot(range(-10, 10), [x * x for x in range(-10, 10)])
    self.plot_values()

  def establish_connection_and_run(self):
    pass

  def mass(self, t):
    wet_mass = self.mass_t0_double.get()
    dry_mass = self.mass_t1_double.get()
    burn_duration = self.burn_duration_double.get()
    return wet_mass - (wet_mass - dry_mass) * min(1, t / burn_duration)

  def plot_values(self, _=None, __=None, ___=None):
    if _ is None and __ is None and ___ is None:
      pass
    time = []
    value = []
    value2 = []
    value3 = []
    thrust_max = self.thrust_int.get()
    dry_mass = self.mass_t0_double.get()
    fuel_mass = self.mass_t0_double.get() - self.mass_t1_double.get()
    burn_duration = self.burn_duration_double.get()
    v_last = 0
    area = self.diameter_double.get() * self.diameter_double.get() / 2 * math.pi
    altitude_last = 0


    alpha = 90/360*2*math.pi
    h = 0
    x = 0
    v = 0
    D = 0
    radius_earth = 6371000
    init = False

    if fuel_mass >= dry_mass or burn_duration <= 0 or thrust_max <= 0:
      return
    try:
      for t in range(1 * 60):
        time.append(t)

        fuel_consumed = min(1, t / burn_duration)
        m = self.mass(t)
        d = density(altitude_last) * v_last * v_last * 0.27 * area / 2
        thrust = thrust_max if fuel_consumed < 1 else 0
        #a = thrust / m - g
        #v = v_last + a + (d if v_last < 0 else -d) / m
        #altitude = max(0, altitude_last + (v + v_last) / 2)

        T = thrust
        dx = v * math.cos(alpha)
        x = x + dx
        dh = v * math.sin(alpha)
        h = h + dh
        dv = (T - d - (m * g - (m * v * v) / (radius_earth + h)) * math.sin(alpha)) / m
        #dv = (T/m * math.sin(alpha)) - g
        v = v + dv
        dalpha = (-1 / v) * (g - v * v / (radius_earth + h)) * math.cos(alpha)
        #dalpha = math.cos(alpha) * g / v
        alpha = alpha + dalpha
        if h > 300 and not init:
          init = True
          alpha = 91/360*2*math.pi


        value.append(alpha/2/math.pi*360)
        value2.append(h/100 + 100)
        value3.append(v/2 + 100)
        #value.append(altitude)

        #altitude_last = altitude
        #v_last = v
    except Exception as e:
      print(e)
    self.plot.clear()
    self.plot.plot(time, value)
    self.plot.plot(time, value2)
    self.plot.plot(time, value3)
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
