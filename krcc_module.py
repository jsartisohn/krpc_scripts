from abc import ABCMeta, abstractproperty, abstractmethod
from tkinter import BooleanVar, StringVar


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
    self._terminate.set(value)

  @property
  def id(self):
    return self._id.get()

  @id.setter
  def id(self, value):
    self._id.set(value)

  @abstractproperty
  def name(self):
    pass

  @abstractmethod
  def run(self):
    pass
