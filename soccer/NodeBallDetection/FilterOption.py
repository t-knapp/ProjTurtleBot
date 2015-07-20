# -*- coding: utf-8 -*-

class FilterOption(object):
  
  def __init__(self, name):
    self.name = name
    self.activated = 0
    self.minimum = 0.0
    self.maximum = 0.0
