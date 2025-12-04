#include "JoltPhysicsBackend.h"

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>

#include <algorithm>

// Static initialization tracking
static bool s_joltInitialized = false;

// Jolt memory allocation hooks
static void* JoltAlloc(size_t inSize) { return malloc(inSize); }
static void JoltFree(void* inBlock) { free(inBlock); }

#ifdef _WIN32
#include <malloc.h>
static void* JoltAlignedAlloc(size_t inSize, size_t inAlignment) { return _aligned_malloc(inSize, inAlignment); }
static void JoltAlignedFree(void* inBlock) { _aligned_free(inBlock); }
#else
static void* JoltAlignedAlloc(size_t inSize, size_t inAlignment) { return aligned_alloc(inAlignment, inSize); }
static void JoltAlignedFree(void* inBlock) { free(inBlock); }
#endif

// Initialize Jolt library (call once)
static void InitializeJolt()
{
    if (!s_joltInitialized)
    {
        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
        s_joltInitialized = true;
    }
}

// Cleanup Jolt library (call once at program end)
static void ShutdownJolt()
{
    if (s_joltInitialized)
    {
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
        s_joltInitialized = false;
    }
}

// JoltPhysicsShape implementation
JoltPhysicsShape::JoltPhysicsShape(JPH::Ref<JPH::Shape> shape, PhysicsShapeType type)
    : m_shape(shape), m_type(type)
{
}

// JoltBPLayerInterface implementation
JoltBPLayerInterface::JoltBPLayerInterface()
{
    m_objectToBroadPhase[static_cast<int>(PhysicsLayer::NonMoving)] = JPH::BroadPhaseLayer(0);
    m_objectToBroadPhase[static_cast<int>(PhysicsLayer::Moving)] = JPH::BroadPhaseLayer(1);
}

JPH::uint JoltBPLayerInterface::GetNumBroadPhaseLayers() const
{
    return static_cast<int>(PhysicsLayer::NumLayers);
}

JPH::BroadPhaseLayer JoltBPLayerInterface::GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const
{
    return m_objectToBroadPhase[inLayer];
}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
const char* JoltBPLayerInterface::GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const
{
    switch ((JPH::BroadPhaseLayer::Type)inLayer)
    {
    case 0: return "NON_MOVING";
    case 1: return "MOVING";
    default: return "INVALID";
    }
}
#endif

// JoltObjectVsBPLayerFilter implementation
bool JoltObjectVsBPLayerFilter::ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const
{
    switch (inLayer1)
    {
    case static_cast<JPH::ObjectLayer>(PhysicsLayer::NonMoving):
        return inLayer2 == JPH::BroadPhaseLayer(1);
    case static_cast<JPH::ObjectLayer>(PhysicsLayer::Moving):
        return true;
    default:
        return false;
    }
}

// JoltObjectLayerPairFilter implementation
bool JoltObjectLayerPairFilter::ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const
{
    switch (inObject1)
    {
    case static_cast<JPH::ObjectLayer>(PhysicsLayer::NonMoving):
        return inObject2 == static_cast<JPH::ObjectLayer>(PhysicsLayer::Moving);
    case static_cast<JPH::ObjectLayer>(PhysicsLayer::Moving):
        return true;
    default:
        return false;
    }
}

// JoltContactListener implementation
JoltContactListener::JoltContactListener(JoltPhysicsWorld* world)
    : m_world(world)
{
}

JPH::ValidateResult JoltContactListener::OnContactValidate(const JPH::Body& inBody1, const JPH::Body& inBody2,
                                                            JPH::RVec3Arg inBaseOffset,
                                                            const JPH::CollideShapeResult& inCollisionResult)
{
    if (m_world->OnContactValidate(inBody1, inBody2))
        return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
    return JPH::ValidateResult::RejectAllContactsForThisBodyPair;
}

void JoltContactListener::OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2,
                                          const JPH::ContactManifold& inManifold,
                                          JPH::ContactSettings& ioSettings)
{
    m_world->OnContactAdded(inBody1, inBody2, inManifold);
}

void JoltContactListener::OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2,
                                              const JPH::ContactManifold& inManifold,
                                              JPH::ContactSettings& ioSettings)
{
    // Could add a persisted callback if needed
}

void JoltContactListener::OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair)
{
    m_world->OnContactRemoved(inSubShapePair.GetBody1ID(), inSubShapePair.GetBody2ID());
}

// JoltPhysicsWorld implementation
JoltPhysicsWorld::JoltPhysicsWorld()
{
}

JoltPhysicsWorld::~JoltPhysicsWorld()
{
    Shutdown();
}

bool JoltPhysicsWorld::Initialize(const PhysicsWorldSettings& settings)
{
    if (m_initialized)
        return true;
    
    InitializeJolt();
    
    // Create allocators
    m_tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(10 * 1024 * 1024);
    
    uint32_t numThreads = settings.numThreads;
    if (numThreads == 0)
        numThreads = std::max(1u, std::thread::hardware_concurrency() - 1);
    
    m_jobSystem = std::make_unique<JPH::JobSystemThreadPool>(
        JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, numThreads);
    
    // Create layer interfaces
    m_bpLayerInterface = std::make_unique<JoltBPLayerInterface>();
    m_objectVsBPLayerFilter = std::make_unique<JoltObjectVsBPLayerFilter>();
    m_objectLayerPairFilter = std::make_unique<JoltObjectLayerPairFilter>();
    
    // Create physics system
    m_physicsSystem = std::make_unique<JPH::PhysicsSystem>();
    m_physicsSystem->Init(
        settings.maxBodies,
        0, // numBodyMutexes (0 = auto)
        settings.maxBodyPairs,
        settings.maxContactConstraints,
        *m_bpLayerInterface,
        *m_objectVsBPLayerFilter,
        *m_objectLayerPairFilter
    );
    
    m_physicsSystem->SetGravity(ToJolt(settings.gravity));
    
    // Create and register contact listener
    m_contactListener = std::make_unique<JoltContactListener>(this);
    m_physicsSystem->SetContactListener(m_contactListener.get());
    
    m_initialized = true;
    return true;
}

void JoltPhysicsWorld::Shutdown()
{
    if (!m_initialized)
        return;
    
    m_bodyUserData.clear();
    
    m_contactListener.reset();
    m_physicsSystem.reset();
    m_objectLayerPairFilter.reset();
    m_objectVsBPLayerFilter.reset();
    m_bpLayerInterface.reset();
    m_jobSystem.reset();
    m_tempAllocator.reset();
    
    m_initialized = false;
}

void JoltPhysicsWorld::SetGravity(Vector3 gravity)
{
    if (m_physicsSystem)
        m_physicsSystem->SetGravity(ToJolt(gravity));
}

Vector3 JoltPhysicsWorld::GetGravity() const
{
    if (m_physicsSystem)
        return FromJolt(m_physicsSystem->GetGravity());
    return { 0, -9.81f, 0 };
}

std::shared_ptr<IPhysicsShape> JoltPhysicsWorld::CreateBoxShape(const BoxShapeSettings& settings)
{
    JPH::BoxShapeSettings shapeSettings(ToJolt(settings.halfExtents));
    auto result = shapeSettings.Create();
    if (result.HasError())
        return nullptr;
    return std::make_shared<JoltPhysicsShape>(result.Get(), PhysicsShapeType::Box);
}

std::shared_ptr<IPhysicsShape> JoltPhysicsWorld::CreateSphereShape(const SphereShapeSettings& settings)
{
    JPH::SphereShapeSettings shapeSettings(settings.radius);
    auto result = shapeSettings.Create();
    if (result.HasError())
        return nullptr;
    return std::make_shared<JoltPhysicsShape>(result.Get(), PhysicsShapeType::Sphere);
}

std::shared_ptr<IPhysicsShape> JoltPhysicsWorld::CreateCapsuleShape(const CapsuleShapeSettings& settings)
{
    JPH::CapsuleShapeSettings shapeSettings(settings.halfHeight, settings.radius);
    auto result = shapeSettings.Create();
    if (result.HasError())
        return nullptr;
    return std::make_shared<JoltPhysicsShape>(result.Get(), PhysicsShapeType::Capsule);
}

std::shared_ptr<IPhysicsShape> JoltPhysicsWorld::CreateConvexHullShape(const ConvexHullShapeSettings& settings)
{
    std::vector<JPH::Vec3> points;
    points.reserve(settings.points.size());
    for (const auto& p : settings.points)
        points.push_back(ToJolt(p));
    
    JPH::ConvexHullShapeSettings shapeSettings(points.data(), static_cast<int>(points.size()));
    shapeSettings.mMaxConvexRadius = settings.maxConvexRadius;
    
    auto result = shapeSettings.Create();
    if (result.HasError())
    {
        // Fallback to a small box if convex hull fails
        JPH::BoxShapeSettings fallback(JPH::Vec3(0.1f, 0.1f, 0.1f));
        return std::make_shared<JoltPhysicsShape>(fallback.Create().Get(), PhysicsShapeType::Box);
    }
    return std::make_shared<JoltPhysicsShape>(result.Get(), PhysicsShapeType::ConvexHull);
}

std::shared_ptr<IPhysicsShape> JoltPhysicsWorld::CreateTriangleMeshShape(const TriangleMeshShapeSettings& settings)
{
    JPH::TriangleList triangles;
    
    for (size_t i = 0; i + 2 < settings.indices.size(); i += 3)
    {
        JPH::Triangle tri(
            ToJolt(settings.vertices[settings.indices[i]]),
            ToJolt(settings.vertices[settings.indices[i + 1]]),
            ToJolt(settings.vertices[settings.indices[i + 2]])
        );
        triangles.push_back(tri);
    }
    
    JPH::MeshShapeSettings shapeSettings(triangles);
    auto result = shapeSettings.Create();
    if (result.HasError())
        return nullptr;
    return std::make_shared<JoltPhysicsShape>(result.Get(), PhysicsShapeType::TriangleMesh);
}

PhysicsBodyHandle JoltPhysicsWorld::CreateBody(std::shared_ptr<IPhysicsShape> shape, const PhysicsBodySettings& settings)
{
    if (!m_physicsSystem || !shape)
        return PhysicsBodyHandle::Invalid();
    
    auto joltShape = std::static_pointer_cast<JoltPhysicsShape>(shape);
    
    JPH::BodyCreationSettings bodySettings(
        joltShape->GetJoltShape(),
        ToJoltR(settings.transform.position),
        ToJolt(settings.transform.rotation),
        ToJolt(settings.motionType),
        ToJoltLayer(settings.layer)
    );
    
    bodySettings.mFriction = settings.friction;
    bodySettings.mRestitution = settings.restitution;
    bodySettings.mIsSensor = settings.isSensor;
    
    JPH::BodyInterface& bodyInterface = m_physicsSystem->GetBodyInterface();
    JPH::BodyID bodyID = bodyInterface.CreateAndAddBody(bodySettings, JPH::EActivation::Activate);
    
    if (bodyID.IsInvalid())
        return PhysicsBodyHandle::Invalid();
    
    PhysicsBodyHandle handle = ToHandle(bodyID);
    
    // Store user data if provided
    if (settings.userData)
    {
        std::lock_guard<std::mutex> lock(m_userDataMutex);
        m_bodyUserData[handle.id] = settings.userData;
    }
    
    return handle;
}

void JoltPhysicsWorld::DestroyBody(PhysicsBodyHandle body)
{
    if (!m_physicsSystem || !body.IsValid())
        return;
    
    JPH::BodyID bodyID = FromHandle(body);
    JPH::BodyInterface& bodyInterface = m_physicsSystem->GetBodyInterface();
    
    bodyInterface.RemoveBody(bodyID);
    bodyInterface.DestroyBody(bodyID);
    
    // Remove user data
    {
        std::lock_guard<std::mutex> lock(m_userDataMutex);
        m_bodyUserData.erase(body.id);
    }
}

bool JoltPhysicsWorld::IsBodyValid(PhysicsBodyHandle body) const
{
    if (!m_physicsSystem || !body.IsValid())
        return false;
    
    JPH::BodyID bodyID = FromHandle(body);
    return m_physicsSystem->GetBodyInterface().IsAdded(bodyID);
}

void JoltPhysicsWorld::SetBodyTransform(PhysicsBodyHandle body, const PhysicsTransform& transform)
{
    if (!m_physicsSystem || !body.IsValid())
        return;
    
    JPH::BodyID bodyID = FromHandle(body);
    m_physicsSystem->GetBodyInterface().SetPositionAndRotation(
        bodyID,
        ToJoltR(transform.position),
        ToJolt(transform.rotation),
        JPH::EActivation::Activate
    );
}

PhysicsTransform JoltPhysicsWorld::GetBodyTransform(PhysicsBodyHandle body) const
{
    PhysicsTransform transform = PhysicsTransform::Identity();
    
    if (!m_physicsSystem || !body.IsValid())
        return transform;
    
    JPH::BodyID bodyID = FromHandle(body);
    JPH::RVec3 pos;
    JPH::Quat rot;
    m_physicsSystem->GetBodyInterface().GetPositionAndRotation(bodyID, pos, rot);
    
    transform.position = FromJoltR(pos);
    transform.rotation = FromJolt(rot);
    return transform;
}

void JoltPhysicsWorld::SetBodyVelocity(PhysicsBodyHandle body, const PhysicsVelocity& velocity)
{
    SetLinearVelocity(body, velocity.linear);
    SetAngularVelocity(body, velocity.angular);
}

PhysicsVelocity JoltPhysicsWorld::GetBodyVelocity(PhysicsBodyHandle body) const
{
    return { GetLinearVelocity(body), GetAngularVelocity(body) };
}

void JoltPhysicsWorld::SetLinearVelocity(PhysicsBodyHandle body, Vector3 velocity)
{
    if (!m_physicsSystem || !body.IsValid())
        return;
    
    JPH::BodyID bodyID = FromHandle(body);
    m_physicsSystem->GetBodyInterface().SetLinearVelocity(bodyID, ToJolt(velocity));
}

Vector3 JoltPhysicsWorld::GetLinearVelocity(PhysicsBodyHandle body) const
{
    if (!m_physicsSystem || !body.IsValid())
        return { 0, 0, 0 };
    
    JPH::BodyID bodyID = FromHandle(body);
    return FromJolt(m_physicsSystem->GetBodyInterface().GetLinearVelocity(bodyID));
}

void JoltPhysicsWorld::SetAngularVelocity(PhysicsBodyHandle body, Vector3 velocity)
{
    if (!m_physicsSystem || !body.IsValid())
        return;
    
    JPH::BodyID bodyID = FromHandle(body);
    m_physicsSystem->GetBodyInterface().SetAngularVelocity(bodyID, ToJolt(velocity));
}

Vector3 JoltPhysicsWorld::GetAngularVelocity(PhysicsBodyHandle body) const
{
    if (!m_physicsSystem || !body.IsValid())
        return { 0, 0, 0 };
    
    JPH::BodyID bodyID = FromHandle(body);
    return FromJolt(m_physicsSystem->GetBodyInterface().GetAngularVelocity(bodyID));
}

void JoltPhysicsWorld::ActivateBody(PhysicsBodyHandle body)
{
    if (!m_physicsSystem || !body.IsValid())
        return;
    
    JPH::BodyID bodyID = FromHandle(body);
    m_physicsSystem->GetBodyInterface().ActivateBody(bodyID);
}

void JoltPhysicsWorld::DeactivateBody(PhysicsBodyHandle body)
{
    if (!m_physicsSystem || !body.IsValid())
        return;
    
    JPH::BodyID bodyID = FromHandle(body);
    m_physicsSystem->GetBodyInterface().DeactivateBody(bodyID);
}

bool JoltPhysicsWorld::IsBodyActive(PhysicsBodyHandle body) const
{
    if (!m_physicsSystem || !body.IsValid())
        return false;
    
    JPH::BodyID bodyID = FromHandle(body);
    return m_physicsSystem->GetBodyInterface().IsActive(bodyID);
}

void JoltPhysicsWorld::ApplyForce(PhysicsBodyHandle body, Vector3 force)
{
    if (!m_physicsSystem || !body.IsValid())
        return;
    
    JPH::BodyID bodyID = FromHandle(body);
    m_physicsSystem->GetBodyInterface().AddForce(bodyID, ToJolt(force));
}

void JoltPhysicsWorld::ApplyForceAtPoint(PhysicsBodyHandle body, Vector3 force, Vector3 point)
{
    if (!m_physicsSystem || !body.IsValid())
        return;
    
    JPH::BodyID bodyID = FromHandle(body);
    m_physicsSystem->GetBodyInterface().AddForce(bodyID, ToJolt(force), ToJoltR(point));
}

void JoltPhysicsWorld::ApplyImpulse(PhysicsBodyHandle body, Vector3 impulse)
{
    if (!m_physicsSystem || !body.IsValid())
        return;
    
    JPH::BodyID bodyID = FromHandle(body);
    m_physicsSystem->GetBodyInterface().AddImpulse(bodyID, ToJolt(impulse));
}

void JoltPhysicsWorld::ApplyImpulseAtPoint(PhysicsBodyHandle body, Vector3 impulse, Vector3 point)
{
    if (!m_physicsSystem || !body.IsValid())
        return;
    
    JPH::BodyID bodyID = FromHandle(body);
    m_physicsSystem->GetBodyInterface().AddImpulse(bodyID, ToJolt(impulse), ToJoltR(point));
}

void JoltPhysicsWorld::ApplyTorque(PhysicsBodyHandle body, Vector3 torque)
{
    if (!m_physicsSystem || !body.IsValid())
        return;
    
    JPH::BodyID bodyID = FromHandle(body);
    m_physicsSystem->GetBodyInterface().AddTorque(bodyID, ToJolt(torque));
}

void JoltPhysicsWorld::Step(float deltaTime)
{
    if (!m_physicsSystem)
        return;
    
    // Jolt recommends a fixed time step
    const int collisionSteps = 1;
    m_physicsSystem->Update(deltaTime, collisionSteps, m_tempAllocator.get(), m_jobSystem.get());
}

PhysicsRaycastResult JoltPhysicsWorld::Raycast(Vector3 origin, Vector3 direction, float maxDistance) const
{
    PhysicsRaycastResult result;
    result.hit = false;
    
    if (!m_physicsSystem)
        return result;
    
    // Normalize direction
    float len = sqrtf(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
    if (len > 0.0001f)
    {
        direction.x /= len;
        direction.y /= len;
        direction.z /= len;
    }
    
    JPH::RRayCast ray(ToJoltR(origin), ToJolt(direction) * maxDistance);
    JPH::RayCastResult hitResult;
    
    if (m_physicsSystem->GetNarrowPhaseQuery().CastRay(ray, hitResult))
    {
        result.hit = true;
        result.body = ToHandle(hitResult.mBodyID);
        result.distance = hitResult.mFraction * maxDistance;
        result.hitPoint = {
            origin.x + direction.x * result.distance,
            origin.y + direction.y * result.distance,
            origin.z + direction.z * result.distance
        };
        // Note: Getting the normal requires additional work with SubShapeID
        result.hitNormal = { 0, 1, 0 }; // Placeholder
    }
    
    return result;
}

std::vector<PhysicsBodyHandle> JoltPhysicsWorld::QuerySphere(Vector3 center, float radius) const
{
    // TODO: Implement sphere query using Jolt's collision detection
    return {};
}

std::vector<PhysicsBodyHandle> JoltPhysicsWorld::QueryBox(Vector3 center, Vector3 halfExtents) const
{
    // TODO: Implement box query using Jolt's collision detection
    return {};
}

void JoltPhysicsWorld::SetContactValidateCallback(ContactValidateCallback callback)
{
    m_contactValidateCallback = callback;
}

void JoltPhysicsWorld::SetContactAddedCallback(ContactAddedCallback callback)
{
    m_contactAddedCallback = callback;
}

void JoltPhysicsWorld::SetContactRemovedCallback(ContactRemovedCallback callback)
{
    m_contactRemovedCallback = callback;
}

void JoltPhysicsWorld::SetBodyUserData(PhysicsBodyHandle body, void* userData)
{
    std::lock_guard<std::mutex> lock(m_userDataMutex);
    if (userData)
        m_bodyUserData[body.id] = userData;
    else
        m_bodyUserData.erase(body.id);
}

void* JoltPhysicsWorld::GetBodyUserData(PhysicsBodyHandle body) const
{
    std::lock_guard<std::mutex> lock(m_userDataMutex);
    auto it = m_bodyUserData.find(body.id);
    return (it != m_bodyUserData.end()) ? it->second : nullptr;
}

size_t JoltPhysicsWorld::GetBodyCount() const
{
    if (!m_physicsSystem)
        return 0;
    return m_physicsSystem->GetNumBodies();
}

size_t JoltPhysicsWorld::GetActiveBodyCount() const
{
    if (!m_physicsSystem)
        return 0;
    return m_physicsSystem->GetNumActiveBodies(JPH::EBodyType::RigidBody);
}

JPH::BodyInterface& JoltPhysicsWorld::GetBodyInterface()
{
    return m_physicsSystem->GetBodyInterface();
}

const JPH::BodyInterface& JoltPhysicsWorld::GetBodyInterface() const
{
    return m_physicsSystem->GetBodyInterface();
}

bool JoltPhysicsWorld::OnContactValidate(const JPH::Body& bodyA, const JPH::Body& bodyB)
{
    if (m_contactValidateCallback)
    {
        PhysicsContactInfo info;
        info.bodyA = ToHandle(bodyA.GetID());
        info.bodyB = ToHandle(bodyB.GetID());
        return m_contactValidateCallback(info);
    }
    return true;
}

void JoltPhysicsWorld::OnContactAdded(const JPH::Body& bodyA, const JPH::Body& bodyB,
                                       const JPH::ContactManifold& manifold)
{
    if (m_contactAddedCallback)
    {
        PhysicsContactInfo info;
        info.bodyA = ToHandle(bodyA.GetID());
        info.bodyB = ToHandle(bodyB.GetID());
        
        if (manifold.mRelativeContactPointsOn1.size() > 0)
        {
            JPH::Vec3 contactPoint = manifold.GetWorldSpaceContactPointOn1(0);
            info.contactPoint = FromJolt(contactPoint);
        }
        info.contactNormal = FromJolt(manifold.mWorldSpaceNormal);
        info.penetrationDepth = manifold.mPenetrationDepth;
        
        m_contactAddedCallback(info);
    }
}

void JoltPhysicsWorld::OnContactRemoved(JPH::BodyID bodyA, JPH::BodyID bodyB)
{
    if (m_contactRemovedCallback)
    {
        m_contactRemovedCallback(ToHandle(bodyA), ToHandle(bodyB));
    }
}

JPH::EMotionType JoltPhysicsWorld::ToJolt(PhysicsMotionType type)
{
    switch (type)
    {
    case PhysicsMotionType::Static:    return JPH::EMotionType::Static;
    case PhysicsMotionType::Kinematic: return JPH::EMotionType::Kinematic;
    case PhysicsMotionType::Dynamic:   return JPH::EMotionType::Dynamic;
    default:                           return JPH::EMotionType::Dynamic;
    }
}

JPH::ObjectLayer JoltPhysicsWorld::ToJoltLayer(PhysicsLayer layer)
{
    return static_cast<JPH::ObjectLayer>(layer);
}

PhysicsBodyHandle JoltPhysicsWorld::ToHandle(JPH::BodyID id) const
{
    return { static_cast<uint64_t>(id.GetIndexAndSequenceNumber()) };
}

JPH::BodyID JoltPhysicsWorld::FromHandle(PhysicsBodyHandle handle) const
{
    return JPH::BodyID(static_cast<uint32_t>(handle.id));
}

// Factory function implementation
std::unique_ptr<IPhysicsWorld> CreatePhysicsWorld(PhysicsBackend backend)
{
    switch (backend)
    {
    case PhysicsBackend::Jolt:
        return std::make_unique<JoltPhysicsWorld>();
    
    case PhysicsBackend::PhysX:
        // TODO: Implement PhysX backend
        // return std::make_unique<PhysXPhysicsWorld>();
        return nullptr;
    
    case PhysicsBackend::Bullet:
        // TODO: Implement Bullet backend
        // return std::make_unique<BulletPhysicsWorld>();
        return nullptr;
    
    default:
        return nullptr;
    }
}
