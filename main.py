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

  # add ball here
  sphereShape = jolt.createSphereShape(0.5)
  spherePos = [0.0, 2.0, 0.0]
  sphereRot = [0.0, 0.0, 0.0, 1.0]
  sphereMotionType = 2
  sphereLayer = 1
  sphereBody = jolt.createRigidBody(sphereShape, spherePos, sphereRot, sphereMotionType, sphereLayer)
  jolt.addRigidBody(sphereBody, True)

  sphereVelocity = [0.0, -5.0, 0.0]
  jolt.setLinearVelocity(sphereBody, sphereVelocity)
  jolt.start()

  while jolt.isActive(sphereBody):
    spherePosition = jolt.getCenterOfMassPosition(sphereBody)
    sphereVelocity = jolt.getLinearVelocity(sphereBody)
    print("Sphere position = (%s, %s, %s)" % (spherePosition[0], spherePosition[1], spherePosition[2]))
    print("Sphere velocity = (%s, %s, %s)" % (sphereVelocity[0], sphereVelocity[1], sphereVelocity[2]))
    jolt.update()
    

  jolt.shutdown()
