#pragma once

#include "PhysicsStructures.h"
#include "raylib.h"
#include <vector>

// 3D Voxel grid for tracking moving particles
class ParticleVoxelGrid
{
private:
    std::vector<ParticleVoxelCell> cells;
    int gridSizeX, gridSizeY, gridSizeZ;
    float cellSize;
    float worldMinX, worldMinY, worldMinZ;
    float worldMaxX, worldMaxY, worldMaxZ;
    
public:
    ParticleVoxelGrid(int sizeX, int sizeY, int sizeZ, float cellSz, 
                      float minX, float minY, float minZ,
                      float maxX, float maxY, float maxZ);
    
    void Clear();
    int GetIndex(int x, int y, int z) const;
    void WorldToGrid(float wx, float wy, float wz, int& gx, int& gy, int& gz) const;
    void AddParticle(float wx, float wy, float wz, float mass, Vector3 velocity);
    
    // Calculate force from particles against a blade (represented as a rotated box)
    // Returns force vector that particles exert on the blade
    Vector3 CalculateBladeForce(Vector3 bladePos, float bladeHalfX, float bladeHalfY, float bladeHalfZ,
                                float bladeRotationRad, float deltaTime) const;
    
    // Get statistics for display
    int GetTotalParticleCount() const;
    float GetTotalMomentum() const;
    
    int GetGridSizeX() const;
    int GetGridSizeY() const;
    int GetGridSizeZ() const;
    float GetCellSize() const;
    float GetWorldMinX() const;
    float GetWorldMinY() const;
    float GetWorldMinZ() const;
    
    // Draw debug visualization of occupied voxels
    void DrawDebug() const;
};
