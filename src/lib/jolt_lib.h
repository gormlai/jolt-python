#ifndef _JOLT_LIB_H_
#define _JOLT_LIB_H_

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>


#include <cstdint>

extern "C" {
  void jolt_init();
  void jolt_shutdown();
  void jolt_start();
  void jolt_update();
  uint64_t jolt_createBoxShape(float sizeX, float sizeY, float sizeZ);
  uint64_t jolt_createRigidBody(uint64_t shapeSettingsHandle, JPH::RVec3 position, JPH::Quat rotation, JPH::EMotionType motionType, JPH::ObjectLayer layer);
  void jolt_addRigidBody(uint64_t rigidBodyHandle, bool activate);
}

#endif
