import jolt

if __name__ == "__main__":
  jolt.init()

  # add geometry here
  floorShape = jolt.createBoxShape(100, 1, 100)
  floorPos = [0.0, -1.0, 0.0]
  floorRot = [0.0, 0.0, 0.0, 1.0]
  floorMotionType = 0
  floorLayer = 0
  floorBody = jolt.createRigidBody(floorShape, floorPos, floorRot, floorMotionType, floorLayer)
  jolt.addRigidBody(floorBody, False)

  # add ball here
  sphereShape = jolt.createSphereShape(0.5)
  spherePos = [0.0, 2.0, 0.0]
  sphereRot = [0.0, 0.0, 0.0, 1.0]
  sphereMotionType = 2
  sphereLayer = 1
  sphereBody = jolt.createRigidBody(sphereShape, spherePos, sphereRot, sphereMotionType, sphereLayer)
  jolt.addRigidBody(sphereBody, True)

  # add box here 
  boxVertices = [
    [-1.0, -1.0, -1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, 1.0, 1.0],
    [-1.0, -1.0, 1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, -1.0],
    [1.0, 1.0, 1.0],
    [1.0, -1.0, 1.0]
  ]
  convexHullShape = jolt.createConvexHullShape(boxVertices)
  boxPos = [0.2, 5.0, 0.0]
  boxRot = [0.0, 0.0, 0.0, 1.0]
  boxMotionType = 2
  boxLayer = 1
  boxBody = jolt.createRigidBody(convexHullShape, boxPos, boxRot, boxMotionType, boxLayer)
  jolt.addRigidBody(boxBody, True)

  bodiesVelocity = [0.0, -5.0, 0.0]
  jolt.setLinearVelocity(sphereBody, bodiesVelocity)
  jolt.setLinearVelocity(boxBody, bodiesVelocity)

  jolt.start()



  while jolt.isActive(sphereBody) and jolt.isActive(boxBody):
    spherePosition = jolt.getCenterOfMassPosition(sphereBody)
    sphereVelocity = jolt.getLinearVelocity(sphereBody)
    print("Sphere position = (%s, %s, %s)" % (spherePosition[0], spherePosition[1], spherePosition[2]))
    print("Sphere velocity = (%s, %s, %s)" % (sphereVelocity[0], sphereVelocity[1], sphereVelocity[2]))

    boxPosition = jolt.getCenterOfMassPosition(boxBody)
    boxVelocity = jolt.getLinearVelocity(boxBody)
    print("Box position = (%s, %s, %s)" % (boxPosition[0], boxPosition[1], boxPosition[2]))
    print("Box velocity = (%s, %s, %s)" % (boxVelocity[0], boxVelocity[1], boxVelocity[2]))

    jolt.update()
    

  jolt.shutdown()
