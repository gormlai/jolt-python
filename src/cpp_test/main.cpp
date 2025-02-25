#include <stdio.h>
#include <iostream>
#include "jolt_lib.h"

int main(const int argc, const char **argv) {
  printf("Example of using Jolt Lib\n");

  jolt_init();

  // first create the ground
  uint64_t floorShape = jolt_createBoxShape(100.0f, 1.0f, 100.0f);
  JPH::RVec3 floorPos{0.0, -1.0, 0.0};
  JPH::Quat floorRotation{0.0, 0.0, 0.0, 1.0};
  JPH::EMotionType floorMotionType = JPH::EMotionType::Static;
  JPH::ObjectLayer floorLayer = 0;
  uint64_t floorBody = jolt_createRigidBody(floorShape, floorPos, floorRotation, floorMotionType, floorLayer);
  jolt_addRigidBody(floorBody, false); // do not activate, as the floor does not move

  // then a falling sphere
  uint64_t sphereShape = jolt_createSphereShape(0.5f);
  JPH::RVec3 spherePos{0.0, 2.0, 0.0};
  JPH::Quat sphereRotation{0.0, 0.0, 0.0, 1.0};
  JPH::EMotionType sphereMotionType = JPH::EMotionType::Dynamic;
  JPH::ObjectLayer sphereLayer = 1;
  uint64_t sphereBody = jolt_createRigidBody(sphereShape, spherePos, sphereRotation, sphereMotionType, sphereLayer);
  jolt_addRigidBody(sphereBody, true); // activate

  const JPH::Vec3 boxVertices[] = {
    {-1.0, -1.0, -1.0},
    {-1.0, 1.0, -1.0},
    {-1.0, 1.0, 1.0},
    {-1.0, -1.0, 1.0},
    {1.0, -1.0, -1.0},
    {1.0, 1.0, -1.0},
    {1.0, 1.0, 1.0},
    {1.0, -1.0, 1.0}
  };
  uint64_t boxShape = jolt_createConvexHullShape(boxVertices, sizeof(boxVertices) / sizeof(JPH::Vec3));
  JPH::RVec3 boxPos{0.2, 5.0, 0.0};
  JPH::Quat boxRotation{0.0, 0.0, 0.0, 1.0};
  JPH::EMotionType boxMotionType = JPH::EMotionType::Dynamic;
  JPH::ObjectLayer boxLayer = 1;
  uint64_t boxBody = jolt_createRigidBody(boxShape, boxPos, boxRotation, boxMotionType, boxLayer);
  jolt_addRigidBody(boxBody, true); // activate


  // start sphere moving
  JPH::RVec3 bodyVelocity{0.0f, -5.0f, 0.0f};
  jolt_setLinearVelocity(sphereBody, bodyVelocity);
  jolt_setLinearVelocity(boxBody, bodyVelocity);

  jolt_start();

  while(jolt_isActive(sphereBody) && jolt_isActive(boxBody)) {
    const JPH::RVec3 spherePosition = jolt_getCenterOfMassPosition(sphereBody);
	  const JPH::Vec3 sphereVelocity = jolt_getLinearVelocity(sphereBody);
	  std::cout << "SpherePosition = (" << spherePosition.GetX() << ", " << spherePosition.GetY() << ", " << spherePosition.GetZ() << "), Velocity = (" << sphereVelocity.GetX() << ", " << sphereVelocity.GetY() << ", " << sphereVelocity.GetZ() << ")" << std::endl;

    const JPH::RVec3 boxPosition = jolt_getCenterOfMassPosition(boxBody);
	  const JPH::Vec3 boxVelocity = jolt_getLinearVelocity(boxBody);
	  std::cout << "BoxPosition = (" << boxPosition.GetX() << ", " << boxPosition.GetY() << ", " << boxPosition.GetZ() << "), Velocity = (" << boxVelocity.GetX() << ", " << boxVelocity.GetY() << ", " << boxVelocity.GetZ() << ")" << std::endl;

    jolt_update();

  }
  jolt_shutdown();

  return 0;

}

