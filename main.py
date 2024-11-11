import jolt

if __name__ == "__main__":
  jolt.init()

  # add geometry here
  boxShape = jolt.createBoxShape(100, 1, 100)
  pos = [0.0, -1.0, 0.0]
  rot = [0.0, 0.0, 0.0, 1.0]
  motionType = 0
  layer = 0
  floorBody = jolt.createRigidBody(boxShape, pos, rot, motionType, layer)
  jolt.addRigidBody(floorBody, False)

  jolt.start()
  jolt.update()
  jolt.shutdown()
