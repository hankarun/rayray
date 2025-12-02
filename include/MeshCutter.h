#pragma once

#include "raylib.h"
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <vector>
#include <array>

// Plane represented by normal and distance from origin
struct CutPlane
{
    Vector3 normal;
    float distance;
    
    // Create plane from point and normal
    static CutPlane FromPointNormal(Vector3 point, Vector3 normal);
    
    // Get signed distance from point to plane
    float SignedDistance(Vector3 point) const;
    
    // Check which side of the plane a point is on
    // Returns: 1 = front, -1 = back, 0 = on plane
    int ClassifyPoint(Vector3 point, float epsilon = 0.0001f) const;
};

// Triangle structure for mesh operations
struct MeshTriangle
{
    Vector3 v0, v1, v2;
    Vector3 n0, n1, n2;  // Per-vertex normals
    ::Color color;       // Use global namespace to avoid conflict with JPH::Color
};

// Cuttable mesh with physics body
struct CuttableMesh
{
    JPH::BodyID bodyID;
    std::vector<MeshTriangle> triangles;
    Model model;
    ::Color color;       // Use global namespace
    bool markedForDestruction;
    Vector3 centerOfMass;
    
    // Create mesh data for rendering
    void UpdateModel();
    
    // Clean up resources
    void Unload();
};

// Result of cutting a mesh
struct CutResult
{
    std::vector<MeshTriangle> frontMesh;  // Triangles on positive side of plane
    std::vector<MeshTriangle> backMesh;   // Triangles on negative side of plane
    bool wasCut;  // True if mesh was actually divided
};

// Mesh cutting utilities
class MeshCutter
{
public:
    // Cut a mesh with a plane, returns two sets of triangles
    static CutResult CutMesh(const std::vector<MeshTriangle>& triangles, const CutPlane& plane);
    
    // Create a cube mesh
    static std::vector<MeshTriangle> CreateCubeMesh(float size, ::Color color);
    
    // Create a sphere mesh
    static std::vector<MeshTriangle> CreateSphereMesh(float radius, int rings, int slices, ::Color color);
    
    // Calculate center of mass for a mesh
    static Vector3 CalculateCenterOfMass(const std::vector<MeshTriangle>& triangles);
    
    // Create convex hull shape from mesh for Jolt Physics
    static JPH::Ref<JPH::Shape> CreateConvexHullShape(const std::vector<MeshTriangle>& triangles);
    
    // Create mesh shape from triangles for Jolt Physics
    static JPH::Ref<JPH::Shape> CreateMeshShape(const std::vector<MeshTriangle>& triangles);
    
private:
    // Clip a triangle against a plane
    static void ClipTriangle(const MeshTriangle& tri, const CutPlane& plane,
                            std::vector<MeshTriangle>& frontTris,
                            std::vector<MeshTriangle>& backTris,
                            std::vector<Vector3>& cutEdgePoints);
    
    // Interpolate between two vertices
    static Vector3 LerpVertex(Vector3 a, Vector3 b, float t);
    
    // Find intersection point of edge with plane
    static float EdgePlaneIntersection(Vector3 a, Vector3 b, const CutPlane& plane);
};

// Cuttable mesh manager - handles creation, cutting, and physics
class CuttableMeshManager
{
public:
    CuttableMeshManager(JPH::PhysicsSystem* physicsSystem);
    ~CuttableMeshManager();
    
    // Create a new cuttable cube
    CuttableMesh* CreateCube(Vector3 position, float size, ::Color color);
    
    // Create a new cuttable sphere
    CuttableMesh* CreateSphere(Vector3 position, float radius, ::Color color, int rings = 16, int slices = 16);
    
    // Cut all meshes that intersect with the plane
    void CutWithPlane(const CutPlane& plane, Vector3 planeWorldPos);
    
    // Update physics and remove destroyed meshes
    void Update(float deltaTime);
    
    // Draw all meshes
    void Draw();
    
    // Get mesh count
    size_t GetMeshCount() const { return meshes.size(); }
    
private:
    JPH::PhysicsSystem* physicsSystem;
    std::vector<CuttableMesh*> meshes;
    
    // Create physics body for mesh
    JPH::BodyID CreatePhysicsBody(const std::vector<MeshTriangle>& triangles, Vector3 position);
    
    // Remove a mesh and its physics body
    void RemoveMesh(size_t index);
};

// Helper to create a raylib Model from triangles
Model CreateModelFromTriangles(const std::vector<MeshTriangle>& triangles);
