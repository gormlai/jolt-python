import jolt

if __name__ == "__main__":
  jolt.init()

  # add geometry here
  boxShape = jolt.createBoxShape(100, 1, 100)
  boxPos = [0.0, -1.0, 0.0]
  boxRot = [0.0, 0.0, 0.0, 1.0]
  boxMotionType = 0
  boxLayer = 0
  floorBody = jolt.createRigidBody(boxShape, boxPos, boxRot, boxMotionType, boxLayer)
  jolt.addRigidBody(floorBody, False)
  

  jolt.start()
  jolt.update()
  jolt.shutdown()
