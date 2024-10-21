import sys
from ctypes import *
import os
import struct

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

def createBoxShape(xSize, ySize, zSize):
  global exportedFunctions
  x = c_float(xSize)
  y = c_float(ySize)
  z = c_float(zSize)
  return exportedFunctions.createBoxShape(x, y, z)

def createRigidBody(shape, pos, rotation, motionType, layer):
  cPos = struct.pack('fff',pos[0], pos[1], pos[2])
  cRot = struct.pack('ffff',rotation[0], rotation[1], rotation[2], rotation[3])
  return exportedFunctions.createRigidBody(shape, cPos, cRot, motionType, layer)
