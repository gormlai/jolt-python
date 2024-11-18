import sys
from ctypes import *
import os
import struct

exportedFunctions = None

def addRigidBody(rigidBodyHandle, activate):
  return exportedFunctions.jolt_addRigidBody(rigidBodyHandle, activate)

def createBoxShape(xSize, ySize, zSize):
  global exportedFunctions
  x = c_float(xSize)
  y = c_float(ySize)
  z = c_float(zSize)
  return exportedFunctions.jolt_createBoxShape(x, y, z)

def createSphereShape(radius):
  global exportedFunctions
  r = c_float(radius)
  return exportedFunctions.jolt_createSphereShape(r)

def createRigidBody(shape, pos, rotation, motionType, layer):
  cPos = (c_float * len(pos))(*pos)
  cRot = (c_float * len(rotation))(*rotation)
  return exportedFunctions.jolt_cCreateRigidBody(shape, cPos, cRot, motionType, layer)

def init():
  global exportedFunctions
  currentDirectory = os.getcwd()
  buildDirectory = currentDirectory
  print("current directory is " + currentDirectory)
  soFile = buildDirectory + "/libjolt-python.so"
  exportedFunctions = CDLL(soFile)
  exportedFunctions.jolt_init()

def setLinearVelocity(shape, velocity):
  cVelocity = (c_float * len(velocity))(*velocity)
  return exportedFunctions.jolt_cSetLinearVelocity(shape, cVelocity)

def start():
  global exportedFunctions
  exportedFunctions.jolt_start()


def shutdown():
  global exportedFunctions
  exportedFunctions.jolt_shutdown()

def update():
  global exportedFunctions
  exportedFunctions.jolt_update()

