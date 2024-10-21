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
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

#include "HandleManager.h"

#include <iostream>
#include <cstdarg>
#include <thread>

#include <pybind11/pybind11.h>

// Disable common warnings triggered by Jolt, you can use JPH_SUPPRESS_WARNING_PUSH / JPH_SUPPRESS_WARNING_POP to store and restore the warning state
JPH_SUPPRESS_WARNINGS

namespace {
    const std::string appName = "Physics Viewer";
    constexpr unsigned int windowWidth = 1200;
    constexpr unsigned int WindowHeight = 800;
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

extern "C" {
  void init();
  void shutdown();
  void start();
  void update();
  uint64_t createBoxShape(float sizeX, float sizeY, float sizeZ);
  uint64_t createRigidBody(uint64_t shapeSettingsHandle, JPH::RVec3 position, JPH::Quat rotation, JPH::EMotionType motionType, JPH::ObjectLayer layer);
}

uint64_t createRigidBody(uint64_t shapeSettingsHandle, JPH::RVec3 position, JPH::Quat rotation, JPH::EMotionType motionType, JPH::ObjectLayer layer) {
  uint64_t rigidBodyHandle = 0;
  JPH::ShapeRefC shapeRef = g_shapeHandleManager.lookup(shapeSettingsHandle);
  JPH::BodyCreationSettings creationSettings(shapeRef.GetPtr(), position, rotation, motionType, layer);
  JPH::BodyInterface &bodyInterface = physicsSystem.GetBodyInterface();
  JPH::Body * newBody = bodyInterface.CreateBody(creationSettings);
  //rigidBodyHandle = g_handleManager.create(newBody);
  return rigidBodyHandle;
}

uint64_t createBoxShape(float sizeX, float sizeY, float sizeZ) {
  JPH::BoxShapeSettings shapeSettings(JPH::Vec3(sizeX, sizeY, sizeZ));
  shapeSettings.SetEmbedded();

  JPH::ShapeSettings::ShapeResult creationResult = shapeSettings.Create();
  JPH::ShapeRefC shapeRef = creationResult.Get();
  uint64_t shapeRefHandle = g_shapeHandleManager.create(shapeRef);
  return shapeRefHandle;
}

void init() {
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

void shutdown() {
  delete tempAllocator;
  delete jobSystem;
}

void start() {
  // nothing more to do -- optimise world
  physicsSystem.OptimizeBroadPhase();

}

void update() {
  // If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
  const int cCollisionSteps = 1;
  constexpr float cDeltaTime = 1.0f / 60.0f;

  // Step the world
  physicsSystem.Update(cDeltaTime, cCollisionSteps, tempAllocator, jobSystem);
  
}
