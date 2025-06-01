#include <string>
#include <iterator>
#include <glm/glm.hpp>
// graphics includes
//#include "TinyEngine/TinyEngine.hpp"
//#include "TinyEngine/helpers/camera.hpp"

// physics includes
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>

#include "HandleManager.h"

#include <iostream>
#include <cstdarg>
#include <thread>

#include "jolt_lib.h"
#include <assert.h>

// Disable common warnings triggered by Jolt, you can use JPH_SUPPRESS_WARNING_PUSH / JPH_SUPPRESS_WARNING_POP to store and restore the warning state
JPH_SUPPRESS_WARNINGS

namespace {
    const std::string appName = "Physics Viewer";
}

namespace Layers
{
        static constexpr JPH::ObjectLayer NON_MOVING = 0;
        static constexpr JPH::ObjectLayer MOVING = 1;
        static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
};

namespace BroadPhaseLayers
{
        static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
        static constexpr JPH::BroadPhaseLayer MOVING(1);
        static constexpr uint NUM_LAYERS(2);
};

class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter
{
public:
        virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override
        {
                switch (inObject1)
                {
                case Layers::NON_MOVING:
                        return inObject2 == Layers::MOVING; // Non moving only collides with moving
                case Layers::MOVING:
                        return true; // Moving collides with everything
                default:
                        JPH_ASSERT(false);
                        return false;
                }
        }
};

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
public:
        BPLayerInterfaceImpl() {
            // Create a mapping table from object to broad phase layer
            mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
            mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
        }

        virtual uint GetNumBroadPhaseLayers() const override {
            return BroadPhaseLayers::NUM_LAYERS;
        }

        virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
            JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
            return mObjectToBroadPhase[inLayer];
        }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
        virtual const char * GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override {
            switch ((JPH::BroadPhaseLayer::Type)inLayer)
            {
                case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:       return "NON_MOVING";
                case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:           return "MOVING";
                default:                                                                                                        JPH_ASSERT(false); return "INVALID";
                }
        }
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
        JPH::BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

class ObjectVsBroadPhaseLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
        virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override
        {
                switch (inLayer1)
                {
                case Layers::NON_MOVING:
                        return inLayer2 == BroadPhaseLayers::MOVING;
                case Layers::MOVING:
                        return true;
                default:
                        JPH_ASSERT(false);
                        return false;
                }
        }
};

class MyContactListener : public JPH::ContactListener
{
public:
        // See: ContactListener
        virtual JPH::ValidateResult  OnContactValidate(const JPH::Body &inBody1, const JPH::Body &inBody2, JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult &inCollisionResult) override
        {
                std::cout << "Contact validate callback" << std::endl;

                // Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
                return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
        }

        virtual void OnContactAdded(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override
        {
                std::cout << "A contact was added" << std::endl;
        }

        virtual void OnContactPersisted(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override
        {
                std::cout << "A contact was persisted" << std::endl;
        }

        virtual void OnContactRemoved(const JPH::SubShapeIDPair &inSubShapePair) override
        {
                std::cout << "A contact was removed" << std::endl;
        }
};

class MyBodyActivationListener : public JPH::BodyActivationListener
{
public:
        virtual void OnBodyActivated(const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData) override
        {
                std::cout << "A body got activated" << std::endl;
        }

        virtual void OnBodyDeactivated(const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData) override
        {
                std::cout << "A body went to sleep" << std::endl;
        }
};


// physics setup
namespace JPH {
    using namespace std;

    // tracing
    static void TraceImpl(const char *argv, ...) {

    }

    static bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, uint inLine) {
        return true;
    }

}

namespace {
  // constants 
  constexpr uint cMaxBodies = 2 << 15;
  constexpr uint cNumBodyMutexes = 0;
  constexpr uint cMaxBodyPairs = 2 << 15;
  constexpr uint cMaxContactConstraints = 2 << 15;

  // variables
  BPLayerInterfaceImpl broadPhaseLayerInterface;
  ObjectVsBroadPhaseLayerFilterImpl objectVSBroadphaseLayerFilter;
  ObjectLayerPairFilterImpl objectVSObjectLayerFilter;
  JPH::PhysicsSystem physicsSystem;
  JPH::TempAllocatorImpl * tempAllocator = nullptr; // standard allocation from the example
  JPH::JobSystemThreadPool * jobSystem = nullptr;
  MyBodyActivationListener bodyActivationListener;
  MyContactListener contactListener;
  HandleManager<void*, uint64_t> g_handleManager;
  HandleManager<JPH::ShapeRefC, uint64_t> g_shapeHandleManager;

  int cCollisionSteps = 1;
  float cDeltaTime = 1.0f / 60.0f;

}

void jolt_addRigidBody(uint64_t rigidBodyHandle, bool activate) {
  JPH::BodyInterface &bodyInterface = physicsSystem.GetBodyInterface();
  JPH::Body * rigidBody = reinterpret_cast<JPH::Body*>(g_handleManager.lookup(rigidBodyHandle));
  const JPH::EActivation startsActive = 
    activate ? 
    JPH::EActivation::Activate :
    JPH::EActivation::DontActivate;

  bodyInterface.AddBody(rigidBody->GetID(), startsActive);
  
}

bool jolt_checkBodiesOverlap(uint64_t rigidBodyHandle0, uint64_t rigidBodyHandle1) {  
  JPH::Body * rigidBody0 = reinterpret_cast<JPH::Body*>(g_handleManager.lookup(rigidBodyHandle0));
  JPH::Body * rigidBody1 = reinterpret_cast<JPH::Body*>(g_handleManager.lookup(rigidBodyHandle1));

  if(rigidBody0==nullptr || rigidBody1==nullptr) {
    return false; // everything is true about the empty set
  }

  const JPH::NarrowPhaseQuery & narrowPhaseQuery = physicsSystem.GetNarrowPhaseQuery();
  //  narrowPhaseQuery.CollideShape()

  const JPH::Shape * shape0 = rigidBody0->GetShape();
  const JPH::RMat44 trans0 = rigidBody0->GetWorldTransform();
  const JPH::Shape * shape1 = rigidBody1->GetShape();
  const JPH::RMat44 trans1 = rigidBody1->GetWorldTransform();

  const JPH::Vec3Arg scale{1.0f, 1.0f, 1.0f};
  const JPH::CollideShapeSettings collideSettings{};
  JPH::CollideShapeResult result;

//  narrowPhaseQuery.CollideShape(shape, scale, centerOfMass, collideSettings, offset);
//  query.Are
  return true; 

}


uint64_t jolt_createBoxShape(float sizeX, float sizeY, float sizeZ) {
  JPH::BoxShapeSettings shapeSettings(JPH::Vec3(sizeX, sizeY, sizeZ));

  JPH::ShapeSettings::ShapeResult creationResult = shapeSettings.Create();
  JPH::ShapeRefC shapeRef = creationResult.Get();
  uint64_t shapeRefHandle = g_shapeHandleManager.create(shapeRef);
  return shapeRefHandle;
}

uint64_t jolt_createConvexHullShape(const JPH::Vec3 * vertices, const int numVertices) {
  JPH::ConvexHullShapeSettings shapeSettings(vertices, numVertices);
  JPH::ShapeSettings::ShapeResult creationResult = shapeSettings.Create();
  JPH::ShapeRefC shapeRef = creationResult.Get();
  uint64_t shapeRefHandle = g_shapeHandleManager.create(shapeRef);
  return shapeRefHandle;
}


uint64_t jolt_createRigidBody(uint64_t shapeSettingsHandle, JPH::RVec3 position, JPH::Quat rotation, JPH::EMotionType motionType, JPH::ObjectLayer layer) {
  uint64_t rigidBodyHandle = 0;
  JPH::ShapeRefC shapeRef = g_shapeHandleManager.lookup(shapeSettingsHandle);
  JPH::BodyCreationSettings creationSettings(shapeRef.GetPtr(), position, rotation, motionType, layer);
  JPH::BodyInterface &bodyInterface = physicsSystem.GetBodyInterface();
  JPH::Body * newBody = bodyInterface.CreateBody(creationSettings);
  assert(newBody);
  rigidBodyHandle = g_handleManager.create(newBody);
  return rigidBodyHandle;
}

uint64_t jolt_cCreateRigidBody(uint64_t shapeSettingsHandle, float position[], float rotation[], JPH::EMotionType motionType, JPH::ObjectLayer layer) {
   JPH::RVec3 pos{position[0], position[1], position[2]};
   JPH::Quat rot{rotation[0], rotation[1], rotation[2], rotation[3]};
   return jolt_createRigidBody(shapeSettingsHandle, pos, rot, motionType, layer);
}

uint64_t jolt_createSphereShape(float radius) {
  JPH::SphereShapeSettings shapeSettings(radius);

  JPH::ShapeSettings::ShapeResult creationResult = shapeSettings.Create();
  JPH::ShapeRefC shapeRef = creationResult.Get();
  uint64_t shapeRefHandle = g_shapeHandleManager.create(shapeRef);
  return shapeRefHandle;
}

void jolt_destroyShape(uint64_t shapeHandle) {
  JPH::ShapeRefC shapeRef = g_shapeHandleManager.lookup(shapeHandle);
  if(shapeRef) {
    g_shapeHandleManager.remove(shapeHandle);
  }
}


void jolt_init() {
  JPH::RegisterDefaultAllocator();
  JPH::Trace = JPH::TraceImpl;
  JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = JPH::AssertFailedImpl;)
  JPH::Factory::sInstance = new JPH::Factory();
  JPH::RegisterTypes();
  tempAllocator = new JPH::TempAllocatorImpl(100 * 1024 * 1024); // standard allocation from the example
  jobSystem = new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, std::thread::hardware_concurrency() - 1);

  physicsSystem.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, broadPhaseLayerInterface, objectVSBroadphaseLayerFilter, objectVSObjectLayerFilter);

  physicsSystem.SetBodyActivationListener(&bodyActivationListener);
  physicsSystem.SetContactListener(&contactListener);
  printf("simple physics setup done\n");
}

bool jolt_isActive(uint64_t rigidBodyHandle) {
  JPH::BodyInterface &bodyInterface = physicsSystem.GetBodyInterface();
  JPH::Body * rigidBody = reinterpret_cast<JPH::Body*>(g_handleManager.lookup(rigidBodyHandle));
  return bodyInterface.IsActive(rigidBody->GetID());
}

JPH::RVec3 jolt_getCenterOfMassPosition(uint64_t rigidBodyHandle) {
  JPH::BodyInterface &bodyInterface = physicsSystem.GetBodyInterface();
  JPH::Body * rigidBody = reinterpret_cast<JPH::Body*>(g_handleManager.lookup(rigidBodyHandle));
  return bodyInterface.GetCenterOfMassPosition(rigidBody->GetID());
}

void jolt_cGetCenterOfMassPosition(uint64_t rigidBodyHandle, float position[3]) {
  JPH::RVec3 vec = jolt_getCenterOfMassPosition(rigidBodyHandle);
  position[0] = vec[0];
  position[1] = vec[1];
  position[2] = vec[2];
}

JPH::Vec3 jolt_getLinearVelocity(uint64_t rigidBodyHandle) {
  JPH::BodyInterface &bodyInterface = physicsSystem.GetBodyInterface();
  JPH::Body * rigidBody = reinterpret_cast<JPH::Body*>(g_handleManager.lookup(rigidBodyHandle));
  return bodyInterface.GetLinearVelocity(rigidBody->GetID());
}

void jolt_cGetLinearVelocity(uint64_t rigidBodyHandle, float velocity[3]) {
  JPH::Vec3 vec = jolt_getLinearVelocity(rigidBodyHandle);
  velocity[0] = vec[0];
  velocity[1] = vec[1];
  velocity[2] = vec[2];
}

void jolt_removeRigidBody(uint64_t rigidBodyHandle) {
  JPH::BodyInterface &bodyInterface = physicsSystem.GetBodyInterface();
  JPH::Body * rigidBody = reinterpret_cast<JPH::Body*>(g_handleManager.lookup(rigidBodyHandle));
  if(rigidBody != nullptr) {
    const bool bodyRemoved = g_handleManager.remove(rigidBodyHandle);
    if(bodyRemoved) {
      bodyInterface.RemoveBody(rigidBody->GetID());
    }
  }
}

void jolt_removeAndDestroyRigidBody(uint64_t rigidBodyHandle) {
  JPH::BodyInterface &bodyInterface = physicsSystem.GetBodyInterface();
  JPH::Body * rigidBody = reinterpret_cast<JPH::Body*>(g_handleManager.lookup(rigidBodyHandle));
  if(rigidBody != nullptr) {
    const bool bodyRemoved = g_handleManager.remove(rigidBodyHandle);
    if(bodyRemoved) {
      bodyInterface.RemoveBody(rigidBody->GetID());
      bodyInterface.DestroyBody(rigidBody->GetID());
    }
  }

}

void jolt_setLinearVelocity(uint64_t rigidBodyHandle, JPH::RVec3 velocity) {
  JPH::BodyInterface &bodyInterface = physicsSystem.GetBodyInterface();
  JPH::Body * rigidBody = reinterpret_cast<JPH::Body*>(g_handleManager.lookup(rigidBodyHandle));
  bodyInterface.SetLinearVelocity(rigidBody->GetID(), velocity);
}

void jolt_cSetLinearVelocity(uint64_t rigidBodyHandle, float velocity[]) {
  JPH::RVec3 vVelocity{velocity[0], velocity[1], velocity[2]};
  jolt_setLinearVelocity(rigidBodyHandle, vVelocity);
}

void jolt_shutdown() {
  delete tempAllocator;
  delete jobSystem;
}

void jolt_start() {
  // nothing more to do -- optimise world
  physicsSystem.OptimizeBroadPhase();

}

void jolt_update() {
  // If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
  const int cCollisionSteps = 1;
  constexpr float cDeltaTime = 1.0f / 60.0f;

  // Step the world
  physicsSystem.Update(cDeltaTime, cCollisionSteps, tempAllocator, jobSystem);
  
}
