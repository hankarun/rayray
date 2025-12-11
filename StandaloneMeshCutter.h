/*
 * Standalone Mesh Cutting Algorithm
 * 
 * A dependency-free, single-file implementation of a mesh cutting algorithm.
 * This file can be integrated into any C++ project without external dependencies.
 * 
 * Features:
 * - Cut arbitrary triangle meshes with a plane
 * - Generate caps to close cut surfaces
 * - Calculate center of mass for mesh pieces
 * - No external dependencies (pure C++ with STL)
 * 
 * Usage:
 *   #include "StandaloneMeshCutter.h"
 *   
 *   // Define your mesh triangles
 *   std::vector<SMC::MeshTriangle> triangles = ...;
 *   
 *   // Define a cutting plane
 *   SMC::CutPlane plane = SMC::CutPlane::FromPointNormal(point, normal);
 *   
 *   // Perform the cut
 *   SMC::CutResult result = SMC::MeshCutter::CutMesh(triangles, plane);
 *   
 *   // Use result.frontMesh and result.backMesh
 */

#pragma once

#include <vector>
#include <algorithm>
#include <cmath>

// Standalone Mesh Cutter namespace to avoid conflicts
namespace SMC {

// Simple 3D vector structure
struct Vector3
{
    float x, y, z;
    
    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    // Vector operations
    Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
    Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
    Vector3 operator*(float s) const { return Vector3(x * s, y * s, z * s); }
    Vector3 operator/(float s) const { return Vector3(x / s, y / s, z / s); }
    
    // Dot product
    float Dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; }
    
    // Cross product
    Vector3 Cross(const Vector3& v) const {
        return Vector3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }
    
    // Length
    float Length() const { return sqrtf(x * x + y * y + z * z); }
    
    // Normalize
    Vector3 Normalized() const {
        float len = Length();
        if (len > 0.0001f) return *this / len;
        return Vector3(0, 0, 0);
    }
};

// Simple color structure (RGBA)
struct Color
{
    unsigned char r, g, b, a;
    
    Color() : r(0), g(0), b(0), a(255) {}
    Color(unsigned char r_, unsigned char g_, unsigned char b_, unsigned char a_ = 255)
        : r(r_), g(g_), b(b_), a(a_) {}
};

// Plane represented by normal and distance from origin
struct CutPlane
{
    Vector3 normal;
    float distance;
    
    // Create plane from point and normal
    static CutPlane FromPointNormal(Vector3 point, Vector3 normal)
    {
        CutPlane plane;
        // Normalize the normal
        plane.normal = normal.Normalized();
        // Distance is dot product of point and normal
        plane.distance = plane.normal.Dot(point);
        return plane;
    }
    
    // Get signed distance from point to plane
    float SignedDistance(Vector3 point) const
    {
        return normal.Dot(point) - distance;
    }
    
    // Check which side of the plane a point is on
    // Returns: 1 = front, -1 = back, 0 = on plane
    int ClassifyPoint(Vector3 point, float epsilon = 0.0001f) const
    {
        float dist = SignedDistance(point);
        if (dist > epsilon) return 1;   // Front
        if (dist < -epsilon) return -1; // Back
        return 0;                        // On plane
    }
};

// Triangle structure for mesh operations
struct MeshTriangle
{
    Vector3 v0, v1, v2;        // Vertex positions
    Vector3 n0, n1, n2;        // Per-vertex normals
    Color color;               // Triangle color
    
    MeshTriangle()
        : v0(), v1(), v2(), n0(), n1(), n2(), color() {}
};

// Result of cutting a mesh
struct CutResult
{
    std::vector<MeshTriangle> frontMesh;  // Triangles on positive side of plane
    std::vector<MeshTriangle> backMesh;   // Triangles on negative side of plane
    bool wasCut;  // True if mesh was actually divided
    
    CutResult() : wasCut(false) {}
};

// Mesh cutting utilities
class MeshCutter
{
public:
    // Cut a mesh with a plane, returns two sets of triangles
    static CutResult CutMesh(const std::vector<MeshTriangle>& triangles, const CutPlane& plane)
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
            Color capColor = triangles.empty() ? Color(128, 128, 128, 255) : triangles[0].color;
            
            // Front cap (normal points in positive direction of cut plane)
            GenerateCapTriangles(cutEdgePoints, plane, result.frontMesh, capColor, true);
            
            // Back cap (normal points in negative direction of cut plane)
            GenerateCapTriangles(cutEdgePoints, plane, result.backMesh, capColor, false);
        }
        
        return result;
    }
    
    // Calculate center of mass for a mesh
    static Vector3 CalculateCenterOfMass(const std::vector<MeshTriangle>& triangles)
    {
        Vector3 center(0, 0, 0);
        float totalArea = 0;
        
        for (const auto& tri : triangles)
        {
            // Calculate triangle centroid
            Vector3 centroid = (tri.v0 + tri.v1 + tri.v2) / 3.0f;
            
            // Calculate triangle area (using magnitude of cross product / 2)
            Vector3 e1 = tri.v1 - tri.v0;
            Vector3 e2 = tri.v2 - tri.v0;
            Vector3 cross = e1.Cross(e2);
            float area = cross.Length() / 2.0f;
            
            center = center + centroid * area;
            totalArea += area;
        }
        
        if (totalArea > 0.0001f)
        {
            center = center / totalArea;
        }
        
        return center;
    }

private:
    // Interpolate between two vertices
    static Vector3 LerpVertex(Vector3 a, Vector3 b, float t)
    {
        return a + (b - a) * t;
    }
    
    // Find intersection point of edge with plane
    static float EdgePlaneIntersection(Vector3 a, Vector3 b, const CutPlane& plane)
    {
        float da = plane.SignedDistance(a);
        float db = plane.SignedDistance(b);
        return da / (da - db);
    }
    
    // Clip a triangle against a plane
    static void ClipTriangle(const MeshTriangle& tri, const CutPlane& plane,
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
        Vector3 centroid(0, 0, 0);
        for (const auto& p : points)
        {
            centroid = centroid + p;
        }
        if (!points.empty())
        {
            centroid = centroid / static_cast<float>(points.size());
        }
        return centroid;
    }
    
    // Helper to sort points around centroid for proper triangulation
    static void SortPointsAroundCentroid(std::vector<Vector3>& points, const Vector3& centroid, const Vector3& normal)
    {
        if (points.size() < 3) return;
        
        // Create a coordinate system on the plane
        Vector3 up(0, 1, 0);
        if (fabsf(normal.y) > 0.9f)
        {
            up = Vector3(1, 0, 0);
        }
        
        // tangent = up x normal
        Vector3 tangent = up.Cross(normal).Normalized();
        
        // bitangent = normal x tangent
        Vector3 bitangent = normal.Cross(tangent);
        
        // Sort by angle around centroid
        std::sort(points.begin(), points.end(), [&](const Vector3& a, const Vector3& b) {
            Vector3 da = a - centroid;
            Vector3 db = b - centroid;
            
            float angleA = atan2f(da.Dot(bitangent), da.Dot(tangent));
            float angleB = atan2f(db.Dot(bitangent), db.Dot(tangent));
            
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
                Vector3 diff = p - u;
                if (diff.Dot(diff) < threshold * threshold)
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
                                      Color color,
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
            normal = normal * -1.0f;
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
};

} // namespace SMC
