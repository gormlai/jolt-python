#include <stdio.h>
#include "jolt_lib.h"

int main(const int argc, const char **argv) {
  printf("Example of using Jolt Lib 3!\n");

  jolt_init();

  uint64_t boxShape = jolt_createBoxShape(100.0f, 1.0f, 100.0f);

  JPH::RVec3 pos{0.0, -1.0, 0.0};
  JPH::Quat rot{0.0, 0.0, 0.0, 1.0};
  JPH::EMotionType motionType = JPH::EMotionType::Static;
  JPH::ObjectLayer layer = 0;
  uint64_t floorBody = jolt_createRigidBody(boxShape, pos, rot, motionType, layer);
  jolt_addRigidBody(floorBody, false); // do not activate, as the floor does not move


  jolt_start();
  jolt_update();
  jolt_shutdown();

  return 0;

}

