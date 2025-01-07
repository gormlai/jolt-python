import sys
from ctypes import *
import os
import struct

exportedFunctions = None

def addRigidBody(rigidBodyHandle, activate):
  global exportedFunctions
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

def createConvexHullShape(vertices, indices = None):
  global exportedFunctions
  numVertices = len(vertices)
  if numVertices == 0 :
    return 0
  
  cVertices = None
  numVertexValuesFromPython = len(vertices[0])
  if indices is None:
    # we multiply by 4 here, as the num vertices are assumed to be a Vec3 
    cVertices = (c_float * (numVertices * 4))(0)
    for vertex in range(numVertices):
      for vIndex in range(numVertexValuesFromPython):
        cVertices[vertex*4 + vIndex] = vertices[vertex][vIndex]
  else:
    numIndices = len(indices)
    cVertices = (c_float * (numIndices * 4))(0)
    for index in range(numIndices):
      vertexIndex = indices[index]
      for vIndex in range(numVertexValuesFromPython):
        cVertices[vertex*4 + vIndex] = vertices[vertexIndex][vIndex]      

  return exportedFunctions.jolt_createConvexHullShape(cVertices, numVertices)

def createRigidBody(shape, pos, rotation, motionType, layer):
  global exportedFunctions
  cPos = (c_float * len(pos))(*pos)
  cRot = (c_float * len(rotation))(*rotation)
  return exportedFunctions.jolt_cCreateRigidBody(shape, cPos, cRot, motionType, layer)

def getCenterOfMassPosition(shape):
  global exportedFunctions
  cPos = (c_float * 3)(0)
  exportedFunctions.jolt_cGetCenterOfMassPosition(shape, cPos)
  pos = [cPos[i] for i in range(3)]
  return pos

def getLinearVelocity(shape):
  global exportedFunctions
  cVelocity = (c_float * 3)(0)
  exportedFunctions.jolt_cGetLinearVelocity(shape, cVelocity)
  velocity = [cVelocity[i] for i in range(3)]
  return velocity

def init():
  global exportedFunctions
  currentDirectory = os.getcwd()
  buildDirectory = currentDirectory
  print("current directory is " + currentDirectory)
  soFile = buildDirectory + "/libjolt-python.so"
  exportedFunctions = CDLL(soFile)
  exportedFunctions.jolt_init()

def isActive(shape):
  global exportedFunctions
  return exportedFunctions.jolt_isActive(shape)

def setLinearVelocity(shape, velocity):
  global exportedFunctions
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

