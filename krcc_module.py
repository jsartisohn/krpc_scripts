from abc import ABCMeta, abstractproperty, abstractmethod
import socket
import threading
from tkinter import Listbox, Y, END, LEFT, Canvas, RIGHT, BOTH, E, N, S, W, \
  BooleanVar, StringVar
import krpc
import math
import time
import numpy as np
from numpy import array

class KRCCModule:
  __metaclass__ = ABCMeta

  def __init__(self):
    self._terminate = BooleanVar(False)
    self._id = StringVar(False)

  @property
  def terminate(self):
    return self._terminate.get()

  @terminate.setter
  def terminate(self, value):
    return self._terminate.set(value)

  @property
  def id(self):
    return self._id.get()

  @id.setter
  def id(self, value):
    return self._id.set(value)

  @abstractproperty
  def name(self):
    pass

  @abstractmethod
  def run(self):
    pass
