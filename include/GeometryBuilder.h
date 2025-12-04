#pragma once

#include "raylib.h"
#include <vector>

// Forward declaration
struct MeshTriangle;

// Geometry builder utilities for creating primitive meshes
class GeometryBuilder
{
public:
    // Create a cube mesh centered at origin
    static std::vector<MeshTriangle> CreateCubeMesh(float size, ::Color color);
    
    // Create a sphere mesh centered at origin
    static std::vector<MeshTriangle> CreateSphereMesh(float radius, int rings, int slices, ::Color color);
};
