#pragma once

#include "PhysicsInterface.h"

#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/ContactListener.h>

#include <unordered_map>
#include <mutex>

// Jolt-specific shape wrapper
class JoltPhysicsShape : public IPhysicsShape
{
public:
    JoltPhysicsShape(JPH::Ref<JPH::Shape> shape, PhysicsShapeType type);
    virtual ~JoltPhysicsShape() = default;
    
    PhysicsShapeType GetType() const override { return m_type; }
    void* GetNativeHandle() const override { return const_cast<JPH::Shape*>(m_shape.GetPtr()); }
    
    JPH::Ref<JPH::Shape> GetJoltShape() const { return m_shape; }
    
private:
    JPH::Ref<JPH::Shape> m_shape;
    PhysicsShapeType m_type;
};

// Jolt broad phase layer interface
class JoltBPLayerInterface final : public JPH::BroadPhaseLayerInterface
{
public:
    JoltBPLayerInterface();
    
    virtual JPH::uint GetNumBroadPhaseLayers() const override;
    virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override;
    
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override;
#endif

private:
    JPH::BroadPhaseLayer m_objectToBroadPhase[static_cast<int>(PhysicsLayer::NumLayers)];
};

// Jolt object vs broad phase layer filter
class JoltObjectVsBPLayerFilter : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override;
};

// Jolt object layer pair filter
class JoltObjectLayerPairFilter : public JPH::ObjectLayerPairFilter
{
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override;
};

// Jolt contact listener
class JoltContactListener : public JPH::ContactListener
{
public:
    JoltContactListener(class JoltPhysicsWorld* world);
    
    virtual JPH::ValidateResult OnContactValidate(const JPH::Body& inBody1, const JPH::Body& inBody2, 
                                                   JPH::RVec3Arg inBaseOffset, 
                                                   const JPH::CollideShapeResult& inCollisionResult) override;
    virtual void OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2, 
                                const JPH::ContactManifold& inManifold, 
                                JPH::ContactSettings& ioSettings) override;
    virtual void OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2, 
                                    const JPH::ContactManifold& inManifold, 
                                    JPH::ContactSettings& ioSettings) override;
    virtual void OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair) override;
    
private:
    class JoltPhysicsWorld* m_world;
};

// Jolt physics world implementation
class JoltPhysicsWorld : public IPhysicsWorld
{
public:
    JoltPhysicsWorld();
    virtual ~JoltPhysicsWorld();
    
    // IPhysicsWorld implementation
    bool Initialize(const PhysicsWorldSettings& settings) override;
    void Shutdown() override;
    
    void SetGravity(Vector3 gravity) override;
    Vector3 GetGravity() const override;
    
    std::shared_ptr<IPhysicsShape> CreateBoxShape(const BoxShapeSettings& settings) override;
    std::shared_ptr<IPhysicsShape> CreateSphereShape(const SphereShapeSettings& settings) override;
    std::shared_ptr<IPhysicsShape> CreateCapsuleShape(const CapsuleShapeSettings& settings) override;
    std::shared_ptr<IPhysicsShape> CreateConvexHullShape(const ConvexHullShapeSettings& settings) override;
    std::shared_ptr<IPhysicsShape> CreateTriangleMeshShape(const TriangleMeshShapeSettings& settings) override;
    
    PhysicsBodyHandle CreateBody(std::shared_ptr<IPhysicsShape> shape, const PhysicsBodySettings& settings) override;
    void DestroyBody(PhysicsBodyHandle body) override;
    bool IsBodyValid(PhysicsBodyHandle body) const override;
    
    void SetBodyTransform(PhysicsBodyHandle body, const PhysicsTransform& transform) override;
    PhysicsTransform GetBodyTransform(PhysicsBodyHandle body) const override;
    
    void SetBodyVelocity(PhysicsBodyHandle body, const PhysicsVelocity& velocity) override;
    PhysicsVelocity GetBodyVelocity(PhysicsBodyHandle body) const override;
    
    void SetLinearVelocity(PhysicsBodyHandle body, Vector3 velocity) override;
    Vector3 GetLinearVelocity(PhysicsBodyHandle body) const override;
    
    void SetAngularVelocity(PhysicsBodyHandle body, Vector3 velocity) override;
    Vector3 GetAngularVelocity(PhysicsBodyHandle body) const override;
    
    void ActivateBody(PhysicsBodyHandle body) override;
    void DeactivateBody(PhysicsBodyHandle body) override;
    bool IsBodyActive(PhysicsBodyHandle body) const override;
    
    void ApplyForce(PhysicsBodyHandle body, Vector3 force) override;
    void ApplyForceAtPoint(PhysicsBodyHandle body, Vector3 force, Vector3 point) override;
    void ApplyImpulse(PhysicsBodyHandle body, Vector3 impulse) override;
    void ApplyImpulseAtPoint(PhysicsBodyHandle body, Vector3 impulse, Vector3 point) override;
    void ApplyTorque(PhysicsBodyHandle body, Vector3 torque) override;
    
    void Step(float deltaTime) override;
    
    PhysicsRaycastResult Raycast(Vector3 origin, Vector3 direction, float maxDistance) const override;
    std::vector<PhysicsBodyHandle> QuerySphere(Vector3 center, float radius) const override;
    std::vector<PhysicsBodyHandle> QueryBox(Vector3 center, Vector3 halfExtents) const override;
    
    void SetContactValidateCallback(ContactValidateCallback callback) override;
    void SetContactAddedCallback(ContactAddedCallback callback) override;
    void SetContactRemovedCallback(ContactRemovedCallback callback) override;
    
    void SetBodyUserData(PhysicsBodyHandle body, void* userData) override;
    void* GetBodyUserData(PhysicsBodyHandle body) const override;
    
    size_t GetBodyCount() const override;
    size_t GetActiveBodyCount() const override;
    
    void* GetNativeHandle() override { return m_physicsSystem.get(); }
    
    // Jolt-specific access
    JPH::PhysicsSystem* GetJoltSystem() { return m_physicsSystem.get(); }
    JPH::BodyInterface& GetBodyInterface();
    const JPH::BodyInterface& GetBodyInterface() const;
    
    // Contact callbacks (called by JoltContactListener)
    bool OnContactValidate(const JPH::Body& bodyA, const JPH::Body& bodyB);
    void OnContactAdded(const JPH::Body& bodyA, const JPH::Body& bodyB, 
                        const JPH::ContactManifold& manifold);
    void OnContactRemoved(JPH::BodyID bodyA, JPH::BodyID bodyB);
    
private:
    // Convert between Jolt and abstraction types
    static JPH::Vec3 ToJolt(Vector3 v) { return JPH::Vec3(v.x, v.y, v.z); }
    static JPH::RVec3 ToJoltR(Vector3 v) { return JPH::RVec3(v.x, v.y, v.z); }
    static JPH::Quat ToJolt(Vector4 q) { return JPH::Quat(q.x, q.y, q.z, q.w); }
    static Vector3 FromJolt(JPH::Vec3 v) { return { v.GetX(), v.GetY(), v.GetZ() }; }
    static Vector3 FromJoltR(JPH::RVec3 v) { return { (float)v.GetX(), (float)v.GetY(), (float)v.GetZ() }; }
    static Vector4 FromJolt(JPH::Quat q) { return { q.GetX(), q.GetY(), q.GetZ(), q.GetW() }; }
    
    static JPH::EMotionType ToJolt(PhysicsMotionType type);
    static JPH::ObjectLayer ToJoltLayer(PhysicsLayer layer);
    
    PhysicsBodyHandle ToHandle(JPH::BodyID id) const;
    JPH::BodyID FromHandle(PhysicsBodyHandle handle) const;
    
private:
    std::unique_ptr<JPH::PhysicsSystem> m_physicsSystem;
    std::unique_ptr<JPH::TempAllocatorImpl> m_tempAllocator;
    std::unique_ptr<JPH::JobSystemThreadPool> m_jobSystem;
    
    std::unique_ptr<JoltBPLayerInterface> m_bpLayerInterface;
    std::unique_ptr<JoltObjectVsBPLayerFilter> m_objectVsBPLayerFilter;
    std::unique_ptr<JoltObjectLayerPairFilter> m_objectLayerPairFilter;
    std::unique_ptr<JoltContactListener> m_contactListener;
    
    ContactValidateCallback m_contactValidateCallback;
    ContactAddedCallback m_contactAddedCallback;
    ContactRemovedCallback m_contactRemovedCallback;
    
    std::unordered_map<uint64_t, void*> m_bodyUserData;
    mutable std::mutex m_userDataMutex;
    
    bool m_initialized = false;
};
