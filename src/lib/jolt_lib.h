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
  void jolt_addRigidBody(uint64_t rigidBodyHandle, bool activate);
  bool jolt_checkBodiesOverlap(uint64_t rigidBodyHandle0, uint64_t rigidBodyHandle1);
  uint64_t jolt_createBoxShape(float sizeX, float sizeY, float sizeZ);
  uint64_t jolt_createConvexHullShape(const JPH::Vec3 * vertices, const int numVertices);
  uint64_t jolt_createRigidBody(uint64_t shapeSettingsHandle, JPH::RVec3 position, JPH::Quat rotation, JPH::EMotionType motionType, JPH::ObjectLayer layer);
  uint64_t jolt_cCreateRigidBody(uint64_t shapeSettingsHandle, float position[], float rotation[], JPH::EMotionType motionType, JPH::ObjectLayer layer);
  uint64_t jolt_createSphereShape(float radius);
  void jolt_destroyShape(uint64_t shapeHandle);
  void jolt_init();
  bool jolt_isActive(uint64_t rigidBodyHandle);
  JPH::RVec3 jolt_getCenterOfMassPosition(uint64_t rigidBodyHandle);
  void jolt_cGetCenterOfMassPosition(uint64_t rigidBodyHandle, float position[3]);
  JPH::Vec3 jolt_getLinearVelocity(uint64_t rigidBodyHandle);
  void jolt_cGetLinearVelocity(uint64_t rigidBodyHandle, float velocity[3]);
  void jolt_removeRigidBody(uint64_t rigidBodyHandle);
  void jolt_removeAndDestroyRigidBody(uint64_t rigidBodyHandle);
  void jolt_setLinearVelocity(uint64_t rigidBodyHandle, JPH::RVec3 velocity);
  void jolt_cSetLinearVelocity(uint64_t rigidBodyHandle, float velocity[]);
  void jolt_shutdown();
  void jolt_start();
  void jolt_update();
}

#endif
