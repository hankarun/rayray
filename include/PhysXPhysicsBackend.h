#pragma once

// PhysX Physics Backend - Stub for NVIDIA PhysX integration
// This file provides the interface for future PhysX implementation

#include "PhysicsInterface.h"

/*
 * To implement NVIDIA PhysX backend:
 * 
 * 1. Download and install PhysX SDK from NVIDIA
 *    - https://developer.nvidia.com/physx-sdk
 * 
 * 2. Add PhysX to your CMakeLists.txt:
 *    find_package(PhysX REQUIRED)
 *    target_link_libraries(${PROJECT_NAME} PhysX PhysXCooking PhysXCommon PhysXFoundation)
 * 
 * 3. Implement the following classes:
 *    - PhysXPhysicsShape : public IPhysicsShape
 *    - PhysXPhysicsWorld : public IPhysicsWorld
 * 
 * 4. Update CreatePhysicsWorld() in JoltPhysicsBackend.cpp to handle PhysicsBackend::PhysX
 */

#if defined(USE_PHYSX)

#include <PxPhysicsAPI.h>

// PhysX-specific shape wrapper
class PhysXPhysicsShape : public IPhysicsShape
{
public:
    PhysXPhysicsShape(physx::PxShape* shape, PhysicsShapeType type);
    virtual ~PhysXPhysicsShape();
    
    PhysicsShapeType GetType() const override { return m_type; }
    void* GetNativeHandle() const override { return m_shape; }
    
    physx::PxShape* GetPhysXShape() const { return m_shape; }
    
private:
    physx::PxShape* m_shape;
    PhysicsShapeType m_type;
};

// PhysX physics world implementation
class PhysXPhysicsWorld : public IPhysicsWorld
{
public:
    PhysXPhysicsWorld();
    virtual ~PhysXPhysicsWorld();
    
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
    
    void* GetNativeHandle() override { return m_scene; }
    
    // PhysX-specific access
    physx::PxScene* GetPhysXScene() { return m_scene; }
    physx::PxPhysics* GetPhysXPhysics() { return m_physics; }
    
private:
    // PhysX objects
    physx::PxFoundation* m_foundation = nullptr;
    physx::PxPhysics* m_physics = nullptr;
    physx::PxDefaultCpuDispatcher* m_dispatcher = nullptr;
    physx::PxScene* m_scene = nullptr;
    physx::PxMaterial* m_defaultMaterial = nullptr;
    
    // Callbacks
    ContactValidateCallback m_contactValidateCallback;
    ContactAddedCallback m_contactAddedCallback;
    ContactRemovedCallback m_contactRemovedCallback;
    
    // Helper conversion functions
    static physx::PxVec3 ToPhysX(Vector3 v) { return physx::PxVec3(v.x, v.y, v.z); }
    static physx::PxQuat ToPhysX(Vector4 q) { return physx::PxQuat(q.x, q.y, q.z, q.w); }
    static Vector3 FromPhysX(physx::PxVec3 v) { return { v.x, v.y, v.z }; }
    static Vector4 FromPhysX(physx::PxQuat q) { return { q.x, q.y, q.z, q.w }; }
    
    bool m_initialized = false;
};

#endif // USE_PHYSX

/*
 * Example PhysX implementation for CreateBody:
 * 
 * PhysicsBodyHandle PhysXPhysicsWorld::CreateBody(std::shared_ptr<IPhysicsShape> shape, const PhysicsBodySettings& settings)
 * {
 *     auto physxShape = std::static_pointer_cast<PhysXPhysicsShape>(shape);
 *     
 *     physx::PxTransform transform(
 *         ToPhysX(settings.transform.position),
 *         ToPhysX(settings.transform.rotation)
 *     );
 *     
 *     physx::PxRigidActor* actor = nullptr;
 *     
 *     switch (settings.motionType)
 *     {
 *     case PhysicsMotionType::Static:
 *         actor = m_physics->createRigidStatic(transform);
 *         break;
 *     
 *     case PhysicsMotionType::Dynamic:
 *     case PhysicsMotionType::Kinematic:
 *         {
 *             auto* dynamic = m_physics->createRigidDynamic(transform);
 *             if (settings.motionType == PhysicsMotionType::Kinematic)
 *                 dynamic->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, true);
 *             physx::PxRigidBodyExt::updateMassAndInertia(*dynamic, settings.mass);
 *             actor = dynamic;
 *         }
 *         break;
 *     }
 *     
 *     actor->attachShape(*physxShape->GetPhysXShape());
 *     m_scene->addActor(*actor);
 *     
 *     return { reinterpret_cast<uint64_t>(actor) };
 * }
 */
