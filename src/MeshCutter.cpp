// Include raylib first, before any other headers to avoid type conflicts
#include "raylib.h"
#include "raymath.h"

// Include our header
#include "MeshCutter.h"
#include "GeometryBuilder.h"
#include <algorithm>
#include <cmath>

// CutPlane implementation
CutPlane CutPlane::FromPointNormal(Vector3 point, Vector3 normal)
{
    CutPlane plane;
    // Normalize the normal
    float len = sqrtf(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    plane.normal = { normal.x / len, normal.y / len, normal.z / len };
    // Distance is dot product of point and normal
    plane.distance = plane.normal.x * point.x + plane.normal.y * point.y + plane.normal.z * point.z;
    return plane;
}

float CutPlane::SignedDistance(Vector3 point) const
{
    return normal.x * point.x + normal.y * point.y + normal.z * point.z - distance;
}

int CutPlane::ClassifyPoint(Vector3 point, float epsilon) const
{
    float dist = SignedDistance(point);
    if (dist > epsilon) return 1;   // Front
    if (dist < -epsilon) return -1; // Back
    return 0;                        // On plane
}

// MeshCutter implementation
Vector3 MeshCutter::LerpVertex(Vector3 a, Vector3 b, float t)
{
    return {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
}

float MeshCutter::EdgePlaneIntersection(Vector3 a, Vector3 b, const CutPlane& plane)
{
    float da = plane.SignedDistance(a);
    float db = plane.SignedDistance(b);
    return da / (da - db);
}

void MeshCutter::ClipTriangle(const MeshTriangle& tri, const CutPlane& plane,
                              std::vector<MeshTriangle>& frontTris,
                              std::vector<MeshTriangle>& backTris,
                              std::vector<Vector3>& cutEdgePoints)
{
    Vector3 verts[3] = { tri.v0, tri.v1, tri.v2 };
    Vector3 norms[3] = { tri.n0, tri.n1, tri.n2 };
    int sides[3];
    
    // Classify each vertex
    for (int i = 0; i < 3; i++)
    {
        sides[i] = plane.ClassifyPoint(verts[i]);
    }
    
    // Count vertices on each side
    int frontCount = 0, backCount = 0;
    for (int i = 0; i < 3; i++)
    {
        if (sides[i] >= 0) frontCount++;
        if (sides[i] <= 0) backCount++;
    }
    
    // All vertices on front side
    if (frontCount == 3)
    {
        frontTris.push_back(tri);
        return;
    }
    
    // All vertices on back side
    if (backCount == 3)
    {
        backTris.push_back(tri);
        return;
    }
    
    // Triangle crosses the plane - need to clip
    std::vector<Vector3> frontVerts, backVerts;
    std::vector<Vector3> frontNorms, backNorms;
    std::vector<Vector3> intersectionPoints;
    
    for (int i = 0; i < 3; i++)
    {
        int next = (i + 1) % 3;
        Vector3 currV = verts[i];
        Vector3 nextV = verts[next];
        Vector3 currN = norms[i];
        Vector3 nextN = norms[next];
        int currSide = sides[i];
        int nextSide = sides[next];
        
        // Add current vertex to appropriate list
        if (currSide >= 0)
        {
            frontVerts.push_back(currV);
            frontNorms.push_back(currN);
        }
        if (currSide <= 0)
        {
            backVerts.push_back(currV);
            backNorms.push_back(currN);
        }
        
        // Check if edge crosses plane
        if ((currSide > 0 && nextSide < 0) || (currSide < 0 && nextSide > 0))
        {
            float t = EdgePlaneIntersection(currV, nextV, plane);
            Vector3 intersect = LerpVertex(currV, nextV, t);
            Vector3 intersectNorm = LerpVertex(currN, nextN, t);
            
            frontVerts.push_back(intersect);
            frontNorms.push_back(intersectNorm);
            backVerts.push_back(intersect);
            backNorms.push_back(intersectNorm);
            
            // Collect intersection points for cap generation
            intersectionPoints.push_back(intersect);
        }
    }
    
    // Add intersection points to cut edge collection (for cap generation)
    for (const auto& pt : intersectionPoints)
    {
        cutEdgePoints.push_back(pt);
    }
    
    // Triangulate front polygon (keep same winding as original triangle)
    for (size_t i = 1; i + 1 < frontVerts.size(); i++)
    {
        MeshTriangle newTri;
        newTri.v0 = frontVerts[0];
        newTri.v1 = frontVerts[i];
        newTri.v2 = frontVerts[i + 1];
        newTri.n0 = frontNorms[0];
        newTri.n1 = frontNorms[i];
        newTri.n2 = frontNorms[i + 1];
        newTri.color = tri.color;
        frontTris.push_back(newTri);
    }
    
    // Triangulate back polygon (keep same winding as original triangle)
    for (size_t i = 1; i + 1 < backVerts.size(); i++)
    {
        MeshTriangle newTri;
        newTri.v0 = backVerts[0];
        newTri.v1 = backVerts[i];
        newTri.v2 = backVerts[i + 1];
        newTri.n0 = backNorms[0];
        newTri.n1 = backNorms[i];
        newTri.n2 = backNorms[i + 1];
        newTri.color = tri.color;
        backTris.push_back(newTri);
    }
}

// Helper to calculate centroid of points
static Vector3 CalculateCentroid(const std::vector<Vector3>& points)
{
    Vector3 centroid = { 0, 0, 0 };
    for (const auto& p : points)
    {
        centroid.x += p.x;
        centroid.y += p.y;
        centroid.z += p.z;
    }
    if (!points.empty())
    {
        centroid.x /= points.size();
        centroid.y /= points.size();
        centroid.z /= points.size();
    }
    return centroid;
}

// Helper to sort points around centroid for proper triangulation
static void SortPointsAroundCentroid(std::vector<Vector3>& points, const Vector3& centroid, const Vector3& normal)
{
    if (points.size() < 3) return;
    
    // Create a coordinate system on the plane
    Vector3 up = { 0, 1, 0 };
    if (fabsf(normal.y) > 0.9f)
    {
        up = { 1, 0, 0 };
    }
    
    // tangent = up x normal
    Vector3 tangent = {
        up.y * normal.z - up.z * normal.y,
        up.z * normal.x - up.x * normal.z,
        up.x * normal.y - up.y * normal.x
    };
    float tLen = sqrtf(tangent.x * tangent.x + tangent.y * tangent.y + tangent.z * tangent.z);
    if (tLen > 0.0001f)
    {
        tangent.x /= tLen;
        tangent.y /= tLen;
        tangent.z /= tLen;
    }
    
    // bitangent = normal x tangent
    Vector3 bitangent = {
        normal.y * tangent.z - normal.z * tangent.y,
        normal.z * tangent.x - normal.x * tangent.z,
        normal.x * tangent.y - normal.y * tangent.x
    };
    
    // Sort by angle around centroid
    std::sort(points.begin(), points.end(), [&](const Vector3& a, const Vector3& b) {
        Vector3 da = { a.x - centroid.x, a.y - centroid.y, a.z - centroid.z };
        Vector3 db = { b.x - centroid.x, b.y - centroid.y, b.z - centroid.z };
        
        float angleA = atan2f(
            da.x * bitangent.x + da.y * bitangent.y + da.z * bitangent.z,
            da.x * tangent.x + da.y * tangent.y + da.z * tangent.z
        );
        float angleB = atan2f(
            db.x * bitangent.x + db.y * bitangent.y + db.z * bitangent.z,
            db.x * tangent.x + db.y * tangent.y + db.z * tangent.z
        );
        
        return angleA < angleB;
    });
}

// Remove duplicate points within a threshold
static void RemoveDuplicatePoints(std::vector<Vector3>& points, float threshold = 0.001f)
{
    std::vector<Vector3> unique;
    for (const auto& p : points)
    {
        bool isDuplicate = false;
        for (const auto& u : unique)
        {
            float dx = p.x - u.x;
            float dy = p.y - u.y;
            float dz = p.z - u.z;
            if (dx * dx + dy * dy + dz * dz < threshold * threshold)
            {
                isDuplicate = true;
                break;
            }
        }
        if (!isDuplicate)
        {
            unique.push_back(p);
        }
    }
    points = unique;
}

// Generate cap triangles from cut edge points
static void GenerateCapTriangles(const std::vector<Vector3>& cutPoints, 
                                  const CutPlane& plane,
                                  std::vector<MeshTriangle>& meshTris,
                                  ::Color color,
                                  bool flipNormal)
{
    if (cutPoints.size() < 3) return;
    
    std::vector<Vector3> points = cutPoints;
    
    // Remove duplicates
    RemoveDuplicatePoints(points);
    
    if (points.size() < 3) return;
    
    // Calculate centroid
    Vector3 centroid = CalculateCentroid(points);
    
    // Get normal direction
    Vector3 normal = plane.normal;
    if (flipNormal)
    {
        normal.x = -normal.x;
        normal.y = -normal.y;
        normal.z = -normal.z;
    }
    
    // Sort points around centroid
    SortPointsAroundCentroid(points, centroid, normal);
    
    // Create fan triangulation from centroid
    for (size_t i = 0; i < points.size(); i++)
    {
        size_t next = (i + 1) % points.size();
        
        MeshTriangle tri;
        tri.v0 = centroid;
        tri.v1 = points[i];
        tri.v2 = points[next];
        tri.n0 = tri.n1 = tri.n2 = normal;
        tri.color = color;
        
        meshTris.push_back(tri);
    }
}

CutResult MeshCutter::CutMesh(const std::vector<MeshTriangle>& triangles, const CutPlane& plane)
{
    CutResult result;
    result.wasCut = false;
    
    std::vector<Vector3> cutEdgePoints;
    
    for (const auto& tri : triangles)
    {
        ClipTriangle(tri, plane, result.frontMesh, result.backMesh, cutEdgePoints);
    }
    
    // Check if we actually have geometry on both sides
    result.wasCut = !result.frontMesh.empty() && !result.backMesh.empty();
    
    // Generate cap faces to close the cut surfaces
    if (result.wasCut && cutEdgePoints.size() >= 3)
    {
        // Get the color from the original mesh
        ::Color capColor = triangles.empty() ? ::Color{128, 128, 128, 255} : triangles[0].color;
        
        // Front cap (normal points in positive direction of cut plane)
        GenerateCapTriangles(cutEdgePoints, plane, result.frontMesh, capColor, true);
        
        // Back cap (normal points in negative direction of cut plane)
        GenerateCapTriangles(cutEdgePoints, plane, result.backMesh, capColor, false);
    }
    
    return result;
}

Vector3 MeshCutter::CalculateCenterOfMass(const std::vector<MeshTriangle>& triangles)
{
    Vector3 center = { 0, 0, 0 };
    float totalArea = 0;
    
    for (const auto& tri : triangles)
    {
        // Calculate triangle centroid
        Vector3 centroid = {
            (tri.v0.x + tri.v1.x + tri.v2.x) / 3.0f,
            (tri.v0.y + tri.v1.y + tri.v2.y) / 3.0f,
            (tri.v0.z + tri.v1.z + tri.v2.z) / 3.0f
        };
        
        // Calculate triangle area (simplified - using magnitude of cross product / 2)
        Vector3 e1 = { tri.v1.x - tri.v0.x, tri.v1.y - tri.v0.y, tri.v1.z - tri.v0.z };
        Vector3 e2 = { tri.v2.x - tri.v0.x, tri.v2.y - tri.v0.y, tri.v2.z - tri.v0.z };
        Vector3 cross = {
            e1.y * e2.z - e1.z * e2.y,
            e1.z * e2.x - e1.x * e2.z,
            e1.x * e2.y - e1.y * e2.x
        };
        float area = sqrtf(cross.x * cross.x + cross.y * cross.y + cross.z * cross.z) / 2.0f;
        
        center.x += centroid.x * area;
        center.y += centroid.y * area;
        center.z += centroid.z * area;
        totalArea += area;
    }
    
    if (totalArea > 0.0001f)
    {
        center.x /= totalArea;
        center.y /= totalArea;
        center.z /= totalArea;
    }
    
    return center;
}

std::shared_ptr<IPhysicsShape> MeshCutter::CreateConvexHullShape(IPhysicsWorld* world, const std::vector<MeshTriangle>& triangles)
{
    // Collect all vertices
    ConvexHullShapeSettings settings;
    settings.points.reserve(triangles.size() * 3);
    settings.maxConvexRadius = 0.01f;
    
    for (const auto& tri : triangles)
    {
        settings.points.push_back(tri.v0);
        settings.points.push_back(tri.v1);
        settings.points.push_back(tri.v2);
    }
    
    return world->CreateConvexHullShape(settings);
}

// CuttableMesh implementation
void CuttableMesh::UpdateModel()
{
    if (model.meshCount > 0)
    {
        UnloadModel(model);
    }
    model = CreateModelFromTriangles(triangles);
}

void CuttableMesh::Unload()
{
    if (model.meshCount > 0)
    {
        UnloadModel(model);
        model.meshCount = 0;
    }
}

// CuttableMeshManager implementation
CuttableMeshManager::CuttableMeshManager(IPhysicsWorld* world)
    : physicsWorld(world)
{
}

CuttableMeshManager::~CuttableMeshManager()
{
    for (auto* mesh : meshes)
    {
        physicsWorld->DestroyBody(mesh->bodyHandle);
        mesh->Unload();
        delete mesh;
    }
    meshes.clear();
}

CuttableMesh* CuttableMeshManager::CreateCube(Vector3 position, float size, ::Color color)
{
    CuttableMesh* mesh = new CuttableMesh();
    mesh->triangles = GeometryBuilder::CreateCubeMesh(size, color);
    mesh->color = color;
    mesh->markedForDestruction = false;
    mesh->centerOfMass = MeshCutter::CalculateCenterOfMass(mesh->triangles);
    
    // Create physics body
    mesh->bodyHandle = CreatePhysicsBody(mesh->triangles, position);
    
    // Create visual model
    mesh->UpdateModel();
    
    meshes.push_back(mesh);
    return mesh;
}

CuttableMesh* CuttableMeshManager::CreateSphere(Vector3 position, float radius, ::Color color, int rings, int slices)
{
    CuttableMesh* mesh = new CuttableMesh();
    mesh->triangles = GeometryBuilder::CreateSphereMesh(radius, rings, slices, color);
    mesh->color = color;
    mesh->markedForDestruction = false;
    mesh->centerOfMass = MeshCutter::CalculateCenterOfMass(mesh->triangles);
    
    // Create physics body
    mesh->bodyHandle = CreatePhysicsBody(mesh->triangles, position);
    
    // Create visual model
    mesh->UpdateModel();
    
    meshes.push_back(mesh);
    return mesh;
}

PhysicsBodyHandle CuttableMeshManager::CreatePhysicsBody(const std::vector<MeshTriangle>& triangles, Vector3 position)
{
    // Create convex hull shape from mesh
    auto shape = MeshCutter::CreateConvexHullShape(physicsWorld, triangles);
    
    if (!shape)
        return PhysicsBodyHandle::Invalid();
    
    // Create body settings
    PhysicsBodySettings settings;
    settings.motionType = PhysicsMotionType::Dynamic;
    settings.layer = PhysicsLayer::Moving;
    settings.transform.position = position;
    settings.transform.rotation = PhysicsHelpers::QuaternionIdentity();
    settings.friction = 0.5f;
    settings.restitution = 0.3f;
    
    return physicsWorld->CreateBody(shape, settings);
}

void CuttableMeshManager::CutWithPlane(const CutPlane& plane, Vector3 planeWorldPos)
{
    std::vector<CuttableMesh*> newMeshes;
    
    for (size_t i = 0; i < meshes.size(); i++)
    {
        CuttableMesh* mesh = meshes[i];
        if (mesh->markedForDestruction) continue;
        
        // Get body transform
        PhysicsTransform bodyTransform = physicsWorld->GetBodyTransform(mesh->bodyHandle);
        Vector4 bodyRot = bodyTransform.rotation;
        Vector3 bodyPos = bodyTransform.position;
        
        // Get the inverse rotation (conjugate for unit quaternions)
        Vector4 invRot = PhysicsHelpers::QuaternionConjugate(bodyRot);
        
        // Transform plane normal to local space
        Vector3 worldNormal = plane.normal;
        Vector3 localNormal = PhysicsHelpers::QuaternionRotateVector(invRot, worldNormal);
        
        // Transform a point on the plane to local space
        Vector3 worldPoint = {
            plane.normal.x * plane.distance,
            plane.normal.y * plane.distance,
            plane.normal.z * plane.distance
        };
        
        // localPoint = invRot * (worldPoint - bodyPos)
        Vector3 relPoint = {
            worldPoint.x - bodyPos.x,
            worldPoint.y - bodyPos.y,
            worldPoint.z - bodyPos.z
        };
        Vector3 localPoint = PhysicsHelpers::QuaternionRotateVector(invRot, relPoint);
        
        CutPlane localPlane = CutPlane::FromPointNormal(localPoint, localNormal);
        
        // Perform the cut
        CutResult result = MeshCutter::CutMesh(mesh->triangles, localPlane);
        
        if (result.wasCut && result.frontMesh.size() >= 4 && result.backMesh.size() >= 4)
        {
            // Mark original for destruction
            mesh->markedForDestruction = true;
            
            // Get current velocity
            PhysicsVelocity velocity = physicsWorld->GetBodyVelocity(mesh->bodyHandle);
            
            // Create front mesh piece
            {
                CuttableMesh* frontMesh = new CuttableMesh();
                frontMesh->triangles = result.frontMesh;
                frontMesh->color = mesh->color;
                frontMesh->markedForDestruction = false;
                
                // Calculate local center of mass for the new piece
                Vector3 localCOM = MeshCutter::CalculateCenterOfMass(result.frontMesh);
                
                // Transform local COM to world space
                Vector3 rotatedCOM = PhysicsHelpers::QuaternionRotateVector(bodyRot, localCOM);
                Vector3 worldCOM = {
                    rotatedCOM.x + bodyPos.x,
                    rotatedCOM.y + bodyPos.y,
                    rotatedCOM.z + bodyPos.z
                };
                
                frontMesh->centerOfMass = worldCOM;
                
                // Center the mesh triangles around their center of mass
                for (auto& tri : frontMesh->triangles)
                {
                    tri.v0.x -= localCOM.x; tri.v0.y -= localCOM.y; tri.v0.z -= localCOM.z;
                    tri.v1.x -= localCOM.x; tri.v1.y -= localCOM.y; tri.v1.z -= localCOM.z;
                    tri.v2.x -= localCOM.x; tri.v2.y -= localCOM.y; tri.v2.z -= localCOM.z;
                }
                
                // Create physics body at the world COM position
                frontMesh->bodyHandle = CreatePhysicsBody(frontMesh->triangles, frontMesh->centerOfMass);
                
                // Apply original velocity plus a small impulse away from cut
                Vector3 impulse = { worldNormal.x * 0.5f, worldNormal.y * 0.5f, worldNormal.z * 0.5f };
                Vector3 newVel = {
                    velocity.linear.x + impulse.x,
                    velocity.linear.y + impulse.y,
                    velocity.linear.z + impulse.z
                };
                physicsWorld->SetLinearVelocity(frontMesh->bodyHandle, newVel);
                physicsWorld->SetAngularVelocity(frontMesh->bodyHandle, velocity.angular);
                
                frontMesh->UpdateModel();
                newMeshes.push_back(frontMesh);
            }
            
            // Create back mesh piece
            {
                CuttableMesh* backMesh = new CuttableMesh();
                backMesh->triangles = result.backMesh;
                backMesh->color = mesh->color;
                backMesh->markedForDestruction = false;
                
                // Calculate local center of mass for the new piece
                Vector3 localCOM = MeshCutter::CalculateCenterOfMass(result.backMesh);
                
                // Transform local COM to world space
                Vector3 rotatedCOM = PhysicsHelpers::QuaternionRotateVector(bodyRot, localCOM);
                Vector3 worldCOM = {
                    rotatedCOM.x + bodyPos.x,
                    rotatedCOM.y + bodyPos.y,
                    rotatedCOM.z + bodyPos.z
                };
                
                backMesh->centerOfMass = worldCOM;
                
                // Center the mesh triangles around their center of mass
                for (auto& tri : backMesh->triangles)
                {
                    tri.v0.x -= localCOM.x; tri.v0.y -= localCOM.y; tri.v0.z -= localCOM.z;
                    tri.v1.x -= localCOM.x; tri.v1.y -= localCOM.y; tri.v1.z -= localCOM.z;
                    tri.v2.x -= localCOM.x; tri.v2.y -= localCOM.y; tri.v2.z -= localCOM.z;
                }
                
                // Create physics body at the world COM position
                backMesh->bodyHandle = CreatePhysicsBody(backMesh->triangles, backMesh->centerOfMass);
                
                // Apply original velocity plus a small impulse away from cut (opposite direction)
                Vector3 impulse = { -worldNormal.x * 0.5f, -worldNormal.y * 0.5f, -worldNormal.z * 0.5f };
                Vector3 newVel = {
                    velocity.linear.x + impulse.x,
                    velocity.linear.y + impulse.y,
                    velocity.linear.z + impulse.z
                };
                physicsWorld->SetLinearVelocity(backMesh->bodyHandle, newVel);
                physicsWorld->SetAngularVelocity(backMesh->bodyHandle, velocity.angular);
                
                backMesh->UpdateModel();
                newMeshes.push_back(backMesh);
            }
        }
    }
    
    // Add new meshes
    for (auto* m : newMeshes)
    {
        meshes.push_back(m);
    }
}

void CuttableMeshManager::Update(float deltaTime)
{
    // Remove destroyed meshes
    for (size_t i = meshes.size(); i > 0; i--)
    {
        if (meshes[i - 1]->markedForDestruction)
        {
            RemoveMesh(i - 1);
        }
    }
}

void CuttableMeshManager::Draw()
{
    for (auto* mesh : meshes)
    {
        if (mesh->markedForDestruction) continue;
        
        // Get body transform
        PhysicsTransform transform = physicsWorld->GetBodyTransform(mesh->bodyHandle);
        
        // Convert quaternion to axis-angle for raylib
        Vector3 axis;
        float angle;
        PhysicsHelpers::QuaternionToAxisAngle(transform.rotation, axis, angle);
        
        // Create transform matrix
        ::Matrix modelTransform = MatrixIdentity();
        modelTransform = MatrixMultiply(modelTransform, MatrixRotate(axis, angle));
        modelTransform = MatrixMultiply(modelTransform, MatrixTranslate(
            transform.position.x, transform.position.y, transform.position.z));
        
        // Draw model with transform
        mesh->model.transform = modelTransform;
        DrawModel(mesh->model, { 0, 0, 0 }, 1.0f, mesh->color);
        
        // Draw wireframe for visibility
        ::Color wireColor = { 0, 0, 0, 255 };
        DrawModelWires(mesh->model, { 0, 0, 0 }, 1.0f, wireColor);
    }
}

void CuttableMeshManager::RemoveMesh(size_t index)
{
    if (index >= meshes.size()) return;
    
    CuttableMesh* mesh = meshes[index];
    
    // Remove physics body
    physicsWorld->DestroyBody(mesh->bodyHandle);
    
    // Unload mesh
    mesh->Unload();
    delete mesh;
    
    meshes.erase(meshes.begin() + index);
}

// Helper function to create raylib Model from triangles
Model CreateModelFromTriangles(const std::vector<MeshTriangle>& triangles)
{
    Model model = { 0 };
    
    if (triangles.empty())
    {
        return model;
    }
    
    // Create mesh
    ::Mesh mesh = { 0 };
    mesh.triangleCount = (int)triangles.size();
    mesh.vertexCount = mesh.triangleCount * 3;
    
    mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
    mesh.normals = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
    mesh.texcoords = (float*)MemAlloc(mesh.vertexCount * 2 * sizeof(float));
    
    int vi = 0;
    int ni = 0;
    int ti = 0;
    
    for (const auto& tri : triangles)
    {
        // Vertex 0
        mesh.vertices[vi++] = tri.v0.x;
        mesh.vertices[vi++] = tri.v0.y;
        mesh.vertices[vi++] = tri.v0.z;
        mesh.normals[ni++] = tri.n0.x;
        mesh.normals[ni++] = tri.n0.y;
        mesh.normals[ni++] = tri.n0.z;
        mesh.texcoords[ti++] = 0.0f;
        mesh.texcoords[ti++] = 0.0f;
        
        // Vertex 1
        mesh.vertices[vi++] = tri.v1.x;
        mesh.vertices[vi++] = tri.v1.y;
        mesh.vertices[vi++] = tri.v1.z;
        mesh.normals[ni++] = tri.n1.x;
        mesh.normals[ni++] = tri.n1.y;
        mesh.normals[ni++] = tri.n1.z;
        mesh.texcoords[ti++] = 1.0f;
        mesh.texcoords[ti++] = 0.0f;
        
        // Vertex 2
        mesh.vertices[vi++] = tri.v2.x;
        mesh.vertices[vi++] = tri.v2.y;
        mesh.vertices[vi++] = tri.v2.z;
        mesh.normals[ni++] = tri.n2.x;
        mesh.normals[ni++] = tri.n2.y;
        mesh.normals[ni++] = tri.n2.z;
        mesh.texcoords[ti++] = 0.5f;
        mesh.texcoords[ti++] = 1.0f;
    }
    
    // Upload mesh to GPU
    UploadMesh(&mesh, false);
    
    // Create model from mesh
    model.meshCount = 1;
    model.meshes = (::Mesh*)MemAlloc(sizeof(::Mesh));
    model.meshes[0] = mesh;
    
    model.materialCount = 1;
    model.materials = (Material*)MemAlloc(sizeof(Material));
    model.materials[0] = LoadMaterialDefault();
    
    model.meshMaterial = (int*)MemAlloc(sizeof(int));
    model.meshMaterial[0] = 0;
    
    model.transform = MatrixIdentity();
    
    return model;
}
