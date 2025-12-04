#include "GeometryBuilder.h"
#include "MeshCutter.h"
#include <cmath>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

std::vector<MeshTriangle> GeometryBuilder::CreateCubeMesh(float size, ::Color color)
{
    std::vector<MeshTriangle> triangles;
    float h = size / 2.0f;
    
    // Define cube vertices
    Vector3 vertices[8] = {
        { -h, -h, -h }, // 0
        {  h, -h, -h }, // 1
        {  h,  h, -h }, // 2
        { -h,  h, -h }, // 3
        { -h, -h,  h }, // 4
        {  h, -h,  h }, // 5
        {  h,  h,  h }, // 6
        { -h,  h,  h }  // 7
    };
    
    // Define face normals
    Vector3 normals[6] = {
        {  0,  0, -1 }, // Front
        {  0,  0,  1 }, // Back
        { -1,  0,  0 }, // Left
        {  1,  0,  0 }, // Right
        {  0,  1,  0 }, // Top
        {  0, -1,  0 }  // Bottom
    };
    
    // Define faces (2 triangles per face)
    int faces[6][4] = {
        { 0, 1, 2, 3 }, // Front  (z = -h)
        { 5, 4, 7, 6 }, // Back   (z = +h)
        { 4, 0, 3, 7 }, // Left   (x = -h)
        { 1, 5, 6, 2 }, // Right  (x = +h)
        { 3, 2, 6, 7 }, // Top    (y = +h)
        { 4, 5, 1, 0 }  // Bottom (y = -h)
    };
    
    for (int f = 0; f < 6; f++)
    {
        MeshTriangle tri1, tri2;
        
        // First triangle (reversed winding order for correct front-facing)
        tri1.v0 = vertices[faces[f][0]];
        tri1.v1 = vertices[faces[f][2]];
        tri1.v2 = vertices[faces[f][1]];
        tri1.n0 = tri1.n1 = tri1.n2 = normals[f];
        tri1.color = color;
        
        // Second triangle (reversed winding order for correct front-facing)
        tri2.v0 = vertices[faces[f][0]];
        tri2.v1 = vertices[faces[f][3]];
        tri2.v2 = vertices[faces[f][2]];
        tri2.n0 = tri2.n1 = tri2.n2 = normals[f];
        tri2.color = color;
        
        triangles.push_back(tri1);
        triangles.push_back(tri2);
    }
    
    return triangles;
}

std::vector<MeshTriangle> GeometryBuilder::CreateSphereMesh(float radius, int rings, int slices, ::Color color)
{
    std::vector<MeshTriangle> triangles;
    
    for (int ring = 0; ring < rings; ring++)
    {
        float theta1 = (float)ring / rings * PI;
        float theta2 = (float)(ring + 1) / rings * PI;
        
        for (int slice = 0; slice < slices; slice++)
        {
            float phi1 = (float)slice / slices * 2.0f * PI;
            float phi2 = (float)(slice + 1) / slices * 2.0f * PI;
            
            // Calculate vertices for this quad
            Vector3 v1 = {
                radius * sinf(theta1) * cosf(phi1),
                radius * cosf(theta1),
                radius * sinf(theta1) * sinf(phi1)
            };
            Vector3 v2 = {
                radius * sinf(theta1) * cosf(phi2),
                radius * cosf(theta1),
                radius * sinf(theta1) * sinf(phi2)
            };
            Vector3 v3 = {
                radius * sinf(theta2) * cosf(phi2),
                radius * cosf(theta2),
                radius * sinf(theta2) * sinf(phi2)
            };
            Vector3 v4 = {
                radius * sinf(theta2) * cosf(phi1),
                radius * cosf(theta2),
                radius * sinf(theta2) * sinf(phi1)
            };
            
            // Normals are just normalized positions for a sphere centered at origin
            Vector3 n1 = { v1.x / radius, v1.y / radius, v1.z / radius };
            Vector3 n2 = { v2.x / radius, v2.y / radius, v2.z / radius };
            Vector3 n3 = { v3.x / radius, v3.y / radius, v3.z / radius };
            Vector3 n4 = { v4.x / radius, v4.y / radius, v4.z / radius };
            
            // First triangle (top cap has degenerate triangles at poles)
            if (ring != 0)
            {
                MeshTriangle tri1;
                tri1.v0 = v1;
                tri1.v1 = v2;
                tri1.v2 = v3;
                tri1.n0 = n1;
                tri1.n1 = n2;
                tri1.n2 = n3;
                tri1.color = color;
                triangles.push_back(tri1);
            }
            
            // Second triangle (bottom cap has degenerate triangles at poles)
            if (ring != rings - 1)
            {
                MeshTriangle tri2;
                tri2.v0 = v1;
                tri2.v1 = v3;
                tri2.v2 = v4;
                tri2.n0 = n1;
                tri2.n1 = n3;
                tri2.n2 = n4;
                tri2.color = color;
                triangles.push_back(tri2);
            }
        }
    }
    
    return triangles;
}
