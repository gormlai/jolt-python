import sys
from ctypes import *
import os

exportedFunctions = None

def init():
  global exportedFunctions
  currentDirectory = os.getcwd()
  buildDirectory = currentDirectory + "/build/"
  print("current directory is " + currentDirectory)
  soFile = buildDirectory + "libjolt-python.so"
  exportedFunctions = CDLL(soFile)
  exportedFunctions.init()

def start():
  global exportedFunctions
  exportedFunctions.start()

def update():
  global exportedFunctions
  exportedFunctions.update()

def shutdown():
  global exportedFunctions
  exportedFunctions.shutdown()

  