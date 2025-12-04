#pragma once

#include "raylib.h"
#include <cstdint>
#include <memory>
#include <vector>
#include <functional>

// Forward declare implementation types (will be different for each physics backend)
struct PhysicsBodyHandle
{
    uint64_t id;
    bool IsValid() const { return id != 0; }
    static PhysicsBodyHandle Invalid() { return { 0 }; }
    bool operator==(const PhysicsBodyHandle& other) const { return id == other.id; }
    bool operator!=(const PhysicsBodyHandle& other) const { return id != other.id; }
};

// Motion type enumeration (backend-agnostic)
enum class PhysicsMotionType
{
    Static,     // Cannot move, infinite mass
    Kinematic,  // Controlled by user, not affected by forces
    Dynamic     // Fully simulated
};

// Collision layer enumeration
enum class PhysicsLayer : uint8_t
{
    NonMoving = 0,
    Moving = 1,
    NumLayers = 2
};

// Shape types
enum class PhysicsShapeType
{
    Box,
    Sphere,
    Capsule,
    ConvexHull,
    TriangleMesh
};

// Transform structure (position + rotation)
struct PhysicsTransform
{
    Vector3 position;
    Vector4 rotation; // Quaternion (x, y, z, w)
    
    static PhysicsTransform Identity()
    {
        return { {0, 0, 0}, {0, 0, 0, 1} };
    }
};

// Velocity structure
struct PhysicsVelocity
{
    Vector3 linear;
    Vector3 angular;
};

// Body creation settings (backend-agnostic)
struct PhysicsBodySettings
{
    PhysicsMotionType motionType = PhysicsMotionType::Dynamic;
    PhysicsLayer layer = PhysicsLayer::Moving;
    PhysicsTransform transform = PhysicsTransform::Identity();
    float mass = 1.0f;
    float friction = 0.5f;
    float restitution = 0.3f;
    bool isSensor = false;
    void* userData = nullptr;
};

// Contact information
struct PhysicsContactInfo
{
    PhysicsBodyHandle bodyA;
    PhysicsBodyHandle bodyB;
    Vector3 contactPoint;
    Vector3 contactNormal;
    float penetrationDepth;
};

// Contact listener callback types
using ContactValidateCallback = std::function<bool(const PhysicsContactInfo&)>;
using ContactAddedCallback = std::function<void(const PhysicsContactInfo&)>;
using ContactRemovedCallback = std::function<void(PhysicsBodyHandle, PhysicsBodyHandle)>;

// Raycast result
struct PhysicsRaycastResult
{
    bool hit = false;
    PhysicsBodyHandle body;
    Vector3 hitPoint;
    Vector3 hitNormal;
    float distance;
};

// Abstract shape class
class IPhysicsShape
{
public:
    virtual ~IPhysicsShape() = default;
    virtual PhysicsShapeType GetType() const = 0;
    virtual void* GetNativeHandle() const = 0;
};

// Box shape
struct BoxShapeSettings
{
    Vector3 halfExtents;
};

// Sphere shape  
struct SphereShapeSettings
{
    float radius;
};

// Capsule shape
struct CapsuleShapeSettings
{
    float halfHeight;
    float radius;
};

// Convex hull shape (from points)
struct ConvexHullShapeSettings
{
    std::vector<Vector3> points;
    float maxConvexRadius = 0.01f;
};

// Triangle mesh shape
struct TriangleMeshShapeSettings
{
    std::vector<Vector3> vertices;
    std::vector<uint32_t> indices;
};

// Physics world settings
struct PhysicsWorldSettings
{
    Vector3 gravity = { 0, -9.81f, 0 };
    uint32_t maxBodies = 1024;
    uint32_t maxBodyPairs = 1024;
    uint32_t maxContactConstraints = 1024;
    uint32_t numThreads = 0; // 0 = auto-detect
};

// Abstract physics world interface
class IPhysicsWorld
{
public:
    virtual ~IPhysicsWorld() = default;
    
    // Initialization
    virtual bool Initialize(const PhysicsWorldSettings& settings) = 0;
    virtual void Shutdown() = 0;
    
    // World properties
    virtual void SetGravity(Vector3 gravity) = 0;
    virtual Vector3 GetGravity() const = 0;
    
    // Shape creation
    virtual std::shared_ptr<IPhysicsShape> CreateBoxShape(const BoxShapeSettings& settings) = 0;
    virtual std::shared_ptr<IPhysicsShape> CreateSphereShape(const SphereShapeSettings& settings) = 0;
    virtual std::shared_ptr<IPhysicsShape> CreateCapsuleShape(const CapsuleShapeSettings& settings) = 0;
    virtual std::shared_ptr<IPhysicsShape> CreateConvexHullShape(const ConvexHullShapeSettings& settings) = 0;
    virtual std::shared_ptr<IPhysicsShape> CreateTriangleMeshShape(const TriangleMeshShapeSettings& settings) = 0;
    
    // Body management
    virtual PhysicsBodyHandle CreateBody(std::shared_ptr<IPhysicsShape> shape, const PhysicsBodySettings& settings) = 0;
    virtual void DestroyBody(PhysicsBodyHandle body) = 0;
    virtual bool IsBodyValid(PhysicsBodyHandle body) const = 0;
    
    // Body properties
    virtual void SetBodyTransform(PhysicsBodyHandle body, const PhysicsTransform& transform) = 0;
    virtual PhysicsTransform GetBodyTransform(PhysicsBodyHandle body) const = 0;
    
    virtual void SetBodyVelocity(PhysicsBodyHandle body, const PhysicsVelocity& velocity) = 0;
    virtual PhysicsVelocity GetBodyVelocity(PhysicsBodyHandle body) const = 0;
    
    virtual void SetLinearVelocity(PhysicsBodyHandle body, Vector3 velocity) = 0;
    virtual Vector3 GetLinearVelocity(PhysicsBodyHandle body) const = 0;
    
    virtual void SetAngularVelocity(PhysicsBodyHandle body, Vector3 velocity) = 0;
    virtual Vector3 GetAngularVelocity(PhysicsBodyHandle body) const = 0;
    
    // Body activation
    virtual void ActivateBody(PhysicsBodyHandle body) = 0;
    virtual void DeactivateBody(PhysicsBodyHandle body) = 0;
    virtual bool IsBodyActive(PhysicsBodyHandle body) const = 0;
    
    // Forces and impulses
    virtual void ApplyForce(PhysicsBodyHandle body, Vector3 force) = 0;
    virtual void ApplyForceAtPoint(PhysicsBodyHandle body, Vector3 force, Vector3 point) = 0;
    virtual void ApplyImpulse(PhysicsBodyHandle body, Vector3 impulse) = 0;
    virtual void ApplyImpulseAtPoint(PhysicsBodyHandle body, Vector3 impulse, Vector3 point) = 0;
    virtual void ApplyTorque(PhysicsBodyHandle body, Vector3 torque) = 0;
    
    // Simulation
    virtual void Step(float deltaTime) = 0;
    
    // Queries
    virtual PhysicsRaycastResult Raycast(Vector3 origin, Vector3 direction, float maxDistance) const = 0;
    virtual std::vector<PhysicsBodyHandle> QuerySphere(Vector3 center, float radius) const = 0;
    virtual std::vector<PhysicsBodyHandle> QueryBox(Vector3 center, Vector3 halfExtents) const = 0;
    
    // Contact callbacks
    virtual void SetContactValidateCallback(ContactValidateCallback callback) = 0;
    virtual void SetContactAddedCallback(ContactAddedCallback callback) = 0;
    virtual void SetContactRemovedCallback(ContactRemovedCallback callback) = 0;
    
    // User data
    virtual void SetBodyUserData(PhysicsBodyHandle body, void* userData) = 0;
    virtual void* GetBodyUserData(PhysicsBodyHandle body) const = 0;
    
    // Debug
    virtual size_t GetBodyCount() const = 0;
    virtual size_t GetActiveBodyCount() const = 0;
    
    // Native access (for backend-specific features)
    virtual void* GetNativeHandle() = 0;
};

// Physics backend type enumeration
enum class PhysicsBackend
{
    Jolt,
    PhysX,  // Future: NVIDIA PhysX
    Bullet  // Future: Bullet Physics (optional)
};

// Factory function to create physics world
std::unique_ptr<IPhysicsWorld> CreatePhysicsWorld(PhysicsBackend backend);

// Helper functions for quaternion/transform conversions
namespace PhysicsHelpers
{
    // Create quaternion from axis-angle
    inline Vector4 QuaternionFromAxisAngle(Vector3 axis, float angle)
    {
        float halfAngle = angle * 0.5f;
        float s = sinf(halfAngle);
        return { axis.x * s, axis.y * s, axis.z * s, cosf(halfAngle) };
    }
    
    // Create identity quaternion
    inline Vector4 QuaternionIdentity()
    {
        return { 0, 0, 0, 1 };
    }
    
    // Convert quaternion to axis-angle
    inline void QuaternionToAxisAngle(Vector4 q, Vector3& axis, float& angle)
    {
        if (q.w > 1.0f) // Normalize if needed
        {
            float len = sqrtf(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
            q.x /= len; q.y /= len; q.z /= len; q.w /= len;
        }
        
        angle = 2.0f * acosf(q.w);
        float s = sqrtf(1.0f - q.w * q.w);
        
        if (s < 0.001f)
        {
            axis = { 1, 0, 0 };
        }
        else
        {
            axis = { q.x / s, q.y / s, q.z / s };
        }
    }
    
    // Multiply quaternions
    inline Vector4 QuaternionMultiply(Vector4 q1, Vector4 q2)
    {
        return {
            q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
            q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
            q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
            q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        };
    }
    
    // Rotate vector by quaternion
    inline Vector3 QuaternionRotateVector(Vector4 q, Vector3 v)
    {
        Vector3 qv = { q.x, q.y, q.z };
        Vector3 uv = {
            qv.y * v.z - qv.z * v.y,
            qv.z * v.x - qv.x * v.z,
            qv.x * v.y - qv.y * v.x
        };
        Vector3 uuv = {
            qv.y * uv.z - qv.z * uv.y,
            qv.z * uv.x - qv.x * uv.z,
            qv.x * uv.y - qv.y * uv.x
        };
        return {
            v.x + 2.0f * (q.w * uv.x + uuv.x),
            v.y + 2.0f * (q.w * uv.y + uuv.y),
            v.z + 2.0f * (q.w * uv.z + uuv.z)
        };
    }
    
    // Conjugate quaternion (inverse for unit quaternions)
    inline Vector4 QuaternionConjugate(Vector4 q)
    {
        return { -q.x, -q.y, -q.z, q.w };
    }
}
