#include <stdio.h>
#include "jolt_lib.h"

int main(const int argc, const char **argv) {
  printf("Example of using Jolt Lib 3!\n");

  jolt_init();

  // first create the ground
  uint64_t boxShape = jolt_createBoxShape(100.0f, 1.0f, 100.0f);
  JPH::RVec3 boxPos{0.0, -1.0, 0.0};
  JPH::Quat boxRotation{0.0, 0.0, 0.0, 1.0};
  JPH::EMotionType boxMotionType = JPH::EMotionType::Static;
  JPH::ObjectLayer boxLayer = 0;
  uint64_t floorBody = jolt_createRigidBody(boxShape, boxPos, boxRotation, boxMotionType, boxLayer);
  jolt_addRigidBody(floorBody, false); // do not activate, as the floor does not move

  // then a falling sphere
  uint64_t sphereShape = jolt_createSphereShape(0.5f);
  JPH::RVec3 spherePos{0.0, 2.0, 0.0};
  JPH::Quat sphereRotation{0.0, 0.0, 0.0, 1.0};
  JPH::EMotionType sphereMotionType = JPH::EMotionType::Dynamic;
  JPH::ObjectLayer sphereLayer = 1;
  uint64_t sphereBody = jolt_createRigidBody(sphereShape, spherePos, sphereRotation, sphereMotionType, sphereLayer);
  jolt_addRigidBody(sphereBody, true); // activate

  // start sphere moving
  JPH::RVec3 sphereVelocity{0.0f, -5.0f, 0.0f};
  jolt_setLinearVelocity(sphereBody, sphereVelocity);

  jolt_start();
  jolt_update();
  jolt_shutdown();

  return 0;

}

