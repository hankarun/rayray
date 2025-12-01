#include "ParticleVoxelGrid.h"
#include "raymath.h"
#include <cmath>

ParticleVoxelGrid::ParticleVoxelGrid(int sizeX, int sizeY, int sizeZ, float cellSz, 
                  float minX, float minY, float minZ,
                  float maxX, float maxY, float maxZ)
    : gridSizeX(sizeX), gridSizeY(sizeY), gridSizeZ(sizeZ), cellSize(cellSz),
      worldMinX(minX), worldMinY(minY), worldMinZ(minZ),
      worldMaxX(maxX), worldMaxY(maxY), worldMaxZ(maxZ)
{
    cells.resize(sizeX * sizeY * sizeZ);
    Clear();
}

void ParticleVoxelGrid::Clear() {
    for (auto& cell : cells) {
        cell.totalMass = 0.0f;
        cell.averageVelocity = { 0.0f, 0.0f, 0.0f };
        cell.particleCount = 0;
        cell.momentum = 0.0f;
    }
}

int ParticleVoxelGrid::GetIndex(int x, int y, int z) const {
    if (x < 0 || x >= gridSizeX || y < 0 || y >= gridSizeY || z < 0 || z >= gridSizeZ)
        return -1;
    return z * gridSizeX * gridSizeY + y * gridSizeX + x;
}

void ParticleVoxelGrid::WorldToGrid(float wx, float wy, float wz, int& gx, int& gy, int& gz) const {
    gx = (int)((wx - worldMinX) / cellSize);
    gy = (int)((wy - worldMinY) / cellSize);
    gz = (int)((wz - worldMinZ) / cellSize);
    gx = Clamp(gx, 0, gridSizeX - 1);
    gy = Clamp(gy, 0, gridSizeY - 1);
    gz = Clamp(gz, 0, gridSizeZ - 1);
}

void ParticleVoxelGrid::AddParticle(float wx, float wy, float wz, float mass, Vector3 velocity) {
    int gx, gy, gz;
    WorldToGrid(wx, wy, wz, gx, gy, gz);
    int idx = GetIndex(gx, gy, gz);
    if (idx >= 0) {
        ParticleVoxelCell& cell = cells[idx];
        // Accumulate velocity weighted by mass for later averaging
        float oldTotalMass = cell.totalMass;
        cell.totalMass += mass;
        if (cell.totalMass > 0.0f) {
            // Weighted average of velocities
            cell.averageVelocity.x = (cell.averageVelocity.x * oldTotalMass + velocity.x * mass) / cell.totalMass;
            cell.averageVelocity.y = (cell.averageVelocity.y * oldTotalMass + velocity.y * mass) / cell.totalMass;
            cell.averageVelocity.z = (cell.averageVelocity.z * oldTotalMass + velocity.z * mass) / cell.totalMass;
        }
        cell.particleCount++;
        // Calculate momentum magnitude
        float velMag = sqrtf(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
        cell.momentum += mass * velMag;
    }
}

// Calculate force from particles against a blade (represented as a rotated box)
// Returns force vector that particles exert on the blade
Vector3 ParticleVoxelGrid::CalculateBladeForce(Vector3 bladePos, float bladeHalfX, float bladeHalfY, float bladeHalfZ,
                            float bladeRotationRad, float deltaTime) const {
    Vector3 totalForce = { 0.0f, 0.0f, 0.0f };
    
    float cosR = cosf(bladeRotationRad);
    float sinR = sinf(bladeRotationRad);
    
    // Calculate blade AABB for quick rejection
    float bladeExtentX = fabs(bladeHalfX * cosR) + fabs(bladeHalfZ * sinR);
    float bladeExtentZ = fabs(bladeHalfX * sinR) + fabs(bladeHalfZ * cosR);
    
    float bladeMinX = bladePos.x - bladeExtentX;
    float bladeMaxX = bladePos.x + bladeExtentX;
    float bladeMinY = bladePos.y - bladeHalfY;
    float bladeMaxY = bladePos.y + bladeHalfY;
    float bladeMinZ = bladePos.z - bladeExtentZ;
    float bladeMaxZ = bladePos.z + bladeExtentZ;
    
    // Convert blade bounds to grid coordinates
    int gridMinX, gridMinY, gridMinZ, gridMaxX, gridMaxY, gridMaxZ;
    WorldToGrid(bladeMinX, bladeMinY, bladeMinZ, gridMinX, gridMinY, gridMinZ);
    WorldToGrid(bladeMaxX, bladeMaxY, bladeMaxZ, gridMaxX, gridMaxY, gridMaxZ);
    
    // Blade normal direction (perpendicular to blade face, in direction of travel)
    // For a blade moving in +X direction with rotation, the front face normal
    Vector3 bladeNormal = { cosR, 0.0f, sinR };
    
    // Check all voxel cells that might intersect the blade
    for (int gz = gridMinZ; gz <= gridMaxZ; gz++) {
        for (int gy = gridMinY; gy <= gridMaxY; gy++) {
            for (int gx = gridMinX; gx <= gridMaxX; gx++) {
                int idx = GetIndex(gx, gy, gz);
                if (idx < 0) continue;
                
                const ParticleVoxelCell& cell = cells[idx];
                if (cell.particleCount == 0 || cell.totalMass < 0.001f) continue;
                
                // Calculate cell center in world space
                float cellCenterX = worldMinX + (gx + 0.5f) * cellSize;
                float cellCenterY = worldMinY + (gy + 0.5f) * cellSize;
                float cellCenterZ = worldMinZ + (gz + 0.5f) * cellSize;
                
                // Transform cell center to blade local space
                float localX = (cellCenterX - bladePos.x) * cosR + (cellCenterZ - bladePos.z) * sinR;
                float localZ = -(cellCenterX - bladePos.x) * sinR + (cellCenterZ - bladePos.z) * cosR;
                float localY = cellCenterY - bladePos.y;
                
                // Check if cell is inside blade bounds (in local space)
                if (fabs(localX) <= bladeHalfX + cellSize * 0.5f &&
                    fabs(localY) <= bladeHalfY + cellSize * 0.5f &&
                    fabs(localZ) <= bladeHalfZ + cellSize * 0.5f) {
                    
                    // Calculate relative velocity (particle velocity relative to blade)
                    // Particles hitting the front of the blade exert force
                    Vector3 relVel = cell.averageVelocity;
                    
                    // Force = dp/dt = m * dv/dt ≈ m * v / dt (impulse force)
                    // We use momentum change as force estimate
                    // F = mass * velocity_component_normal_to_blade / deltaTime
                    
                    float velDotNormal = relVel.x * bladeNormal.x + relVel.y * bladeNormal.y + relVel.z * bladeNormal.z;
                    
                    // Only count particles moving against the blade (negative dot product means towards blade)
                    // Actually, we want particles that the blade is pushing - so blade velocity vs particle
                    // For simplicity, use momentum magnitude scaled by contact
                    float forceMagnitude = cell.momentum / fmaxf(deltaTime, 0.001f);
                    
                    // Apply force in direction opposite to blade normal (reaction force)
                    totalForce.x -= bladeNormal.x * forceMagnitude * 0.01f; // Scale factor for reasonable force
                    totalForce.y -= bladeNormal.y * forceMagnitude * 0.01f;
                    totalForce.z -= bladeNormal.z * forceMagnitude * 0.01f;
                }
            }
        }
    }
    
    return totalForce;
}

// Get statistics for display
int ParticleVoxelGrid::GetTotalParticleCount() const {
    int total = 0;
    for (const auto& cell : cells) {
        total += cell.particleCount;
    }
    return total;
}

float ParticleVoxelGrid::GetTotalMomentum() const {
    float total = 0.0f;
    for (const auto& cell : cells) {
        total += cell.momentum;
    }
    return total;
}

int ParticleVoxelGrid::GetGridSizeX() const { return gridSizeX; }
int ParticleVoxelGrid::GetGridSizeY() const { return gridSizeY; }
int ParticleVoxelGrid::GetGridSizeZ() const { return gridSizeZ; }
float ParticleVoxelGrid::GetCellSize() const { return cellSize; }
float ParticleVoxelGrid::GetWorldMinX() const { return worldMinX; }
float ParticleVoxelGrid::GetWorldMinY() const { return worldMinY; }
float ParticleVoxelGrid::GetWorldMinZ() const { return worldMinZ; }

// Draw debug visualization of occupied voxels
void ParticleVoxelGrid::DrawDebug() const {
    for (int gz = 0; gz < gridSizeZ; gz++) {
        for (int gy = 0; gy < gridSizeY; gy++) {
            for (int gx = 0; gx < gridSizeX; gx++) {
                int idx = GetIndex(gx, gy, gz);
                if (idx >= 0 && cells[idx].particleCount > 0) {
                    // Calculate world position of voxel center
                    float wx = worldMinX + (gx + 0.5f) * cellSize;
                    float wy = worldMinY + (gy + 0.5f) * cellSize;
                    float wz = worldMinZ + (gz + 0.5f) * cellSize;
                    
                    // Color based on momentum/mass
                    float intensity = fminf(cells[idx].momentum / 0.5f, 1.0f);
                    ::Color voxelColor = {
                        (unsigned char)(255 * intensity),
                        (unsigned char)(100 * (1.0f - intensity)),
                        (unsigned char)(50),
                        (unsigned char)(150)
                    };
                    
                    // Draw wireframe cube for each occupied voxel
                    DrawCubeWires(Vector3{wx, wy, wz}, cellSize * 0.9f, cellSize * 0.9f, cellSize * 0.9f, voxelColor);
                    
                    // Draw solid cube with transparency for high-momentum cells
                    if (cells[idx].momentum > 0.1f) {
                        DrawCube(Vector3{wx, wy, wz}, cellSize * 0.8f, cellSize * 0.8f, cellSize * 0.8f, 
                                Fade(voxelColor, 0.3f));
                    }
                }
            }
        }
    }
}
