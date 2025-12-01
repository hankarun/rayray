#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <cmath>
#include <vector>

// Jolt Physics includes
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

// Jolt namespace
using namespace JPH;

// Layer definitions
namespace Layers
{
    static constexpr ObjectLayer NON_MOVING = 0;
    static constexpr ObjectLayer MOVING = 1;
    static constexpr ObjectLayer NUM_LAYERS = 2;
};

// BroadPhaseLayerInterface implementation
class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface
{
public:
    BPLayerInterfaceImpl()
    {
        mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayer(0);
        mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayer(1);
    }

    virtual uint GetNumBroadPhaseLayers() const override
    {
        return 2;
    }

    virtual BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override
    {
        return mObjectToBroadPhase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    virtual const char* GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
    {
        switch ((BroadPhaseLayer::Type)inLayer)
        {
        case 0: return "NON_MOVING";
        case 1: return "MOVING";
        default: return "INVALID";
        }
    }
#endif

private:
    BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

// ObjectVsBroadPhaseLayerFilter implementation
class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter
{
public:
    virtual bool ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override
    {
        switch (inLayer1)
        {
        case Layers::NON_MOVING:
            return inLayer2 == BroadPhaseLayer(1);
        case Layers::MOVING:
            return true;
        default:
            return false;
        }
    }
};

// ObjectLayerPairFilter implementation
class ObjectLayerPairFilterImpl : public ObjectLayerPairFilter
{
public:
    virtual bool ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override
    {
        switch (inObject1)
        {
        case Layers::NON_MOVING:
            return inObject2 == Layers::MOVING;
        case Layers::MOVING:
            return true;
        default:
            return false;
        }
    }
};

// Contact listener (optional, for debugging)
class MyContactListener : public ContactListener
{
public:
    virtual ValidateResult OnContactValidate(const Body &inBody1, const Body &inBody2, RVec3Arg inBaseOffset, const CollideShapeResult &inCollisionResult) override
    {
        return ValidateResult::AcceptAllContactsForThisBodyPair;
    }

    virtual void OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
    {
    }

    virtual void OnContactPersisted(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
    {
    }

    virtual void OnContactRemoved(const SubShapeIDPair &inSubShapePair) override
    {
    }
};

// Structure to hold sphere data
struct PhysicsSphere
{
    BodyID bodyID;
    ::Color color;
    float stoppedTimer; // Timer to track how long the sphere has been stopped
    bool markedForDestruction;
    Vector3 lastPosition; // Track position for velocity check
    Vector3 velocity;     // Current velocity of the sphere
    float mass;           // Mass of the sphere
};

// Helper to check if a sphere has stopped moving
bool IsSphereStationary(const Vector3& currentPos, const Vector3& lastPos, float threshold = 0.01f) {
    float dx = currentPos.x - lastPos.x;
    float dy = currentPos.y - lastPos.y;
    float dz = currentPos.z - lastPos.z;
    return (dx * dx + dy * dy + dz * dz) < threshold * threshold;
}

// Cell structure for cellular automata
struct DensityCell
{
    float density;      // Material density at this cell
    float viscosity;    // Viscosity of material (resistance to flow)
};

// Cellular Automata for density simulation
class DensityAutomata
{
private:
    std::vector<DensityCell> cells;
    std::vector<DensityCell> nextCells;
    int gridSize;
    float cellSize;
    
public:
    DensityAutomata(int size, float cellSz) : gridSize(size), cellSize(cellSz) {
        cells.resize(size * size);
        nextCells.resize(size * size);
        
        // Initialize all cells
        for (int i = 0; i < size * size; i++) {
            cells[i].density = 0.0f;
            cells[i].viscosity = 0.5f; // Default viscosity
        }
    }
    
    void AddDensity(int x, int z, float amount, float visc = 0.5f) {
        if (x >= 0 && x < gridSize && z >= 0 && z < gridSize) {
            int idx = z * gridSize + x;
            cells[idx].density += amount;
            cells[idx].viscosity = visc;
        }
    }
    
    float GetDensity(int x, int z) const {
        if (x >= 0 && x < gridSize && z >= 0 && z < gridSize) {
            return cells[z * gridSize + x].density;
        }
        return 0.0f;
    }
    
    // Cellular automata update step
    void Update(float deltaTime, std::vector<float>& heightSamples, float heightScale) {
        // Copy current state to next state
        nextCells = cells;
        
        // Flow simulation based on density, viscosity, and height differences (like dirt)
        for (int z = 0; z < gridSize; z++) {
            for (int x = 0; x < gridSize; x++) {
                int idx = z * gridSize + x;
                DensityCell& cell = cells[idx];
                
                if (cell.density <= 0.01f) continue; // Skip empty cells
                
                // Calculate flow to neighbors based on density difference and viscosity
                float flowRate = 1.5f * (1.0f - cell.viscosity) * deltaTime; // Increased flow rate
                int neighbors[4][2] = {{-1,0}, {1,0}, {0,-1}, {0,1}};
                
                float currentHeight = heightSamples[idx];
                
                for (int n = 0; n < 4; n++) {
                    int nx = x + neighbors[n][0];
                    int nz = z + neighbors[n][1];
                    
                    if (nx >= 0 && nx < gridSize && nz >= 0 && nz < gridSize) {
                        int nidx = nz * gridSize + nx;
                        float neighborHeight = heightSamples[nidx];
                        
                        // Dirt flows based on both density and height differences (angle of repose)
                        float densityDiff = cell.density - cells[nidx].density;
                        float heightDiff = currentHeight - neighborHeight;
                        
                        // Simulate angle of repose - dirt flows down steep slopes
                        const float angleOfRepose = 0.3f; // About 35 degrees in height units
                        
                        if (densityDiff > 0.0f || heightDiff > angleOfRepose) {
                            float flow = (densityDiff * 0.3f + heightDiff * 0.7f) * flowRate * 0.25f;
                            flow = fmaxf(0.0f, flow); // Only positive flow
                            flow = fminf(flow, cell.density * 0.3f); // Don't flow more than 30% per neighbor
                            
                            nextCells[idx].density -= flow;
                            nextCells[nidx].density += flow;
                        }
                    }
                }
                
                // Apply density to height (higher density = higher terrain)
                if (nextCells[idx].density > 0.1f) {
                    float heightIncrease = nextCells[idx].density * 0.02f * deltaTime; // Faster settling
                    heightSamples[idx] += heightIncrease;
                    if (heightSamples[idx] > heightScale) {
                        heightSamples[idx] = heightScale;
                    }
                    
                    // Reduce density as it converts to height
                    nextCells[idx].density *= 0.95f; // Faster conversion
                }
            }
        }
        
        // Swap buffers
        cells = nextCells;
    }
    
    int GetGridSize() const { return gridSize; }
};

// Voxel cell for particle tracking
struct ParticleVoxelCell
{
    float totalMass;           // Total mass of particles in this cell
    Vector3 averageVelocity;   // Average velocity of particles
    int particleCount;         // Number of particles in this cell
    float momentum;            // Magnitude of momentum (for force calculation)
};

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
                      float maxX, float maxY, float maxZ)
        : gridSizeX(sizeX), gridSizeY(sizeY), gridSizeZ(sizeZ), cellSize(cellSz),
          worldMinX(minX), worldMinY(minY), worldMinZ(minZ),
          worldMaxX(maxX), worldMaxY(maxY), worldMaxZ(maxZ)
    {
        cells.resize(sizeX * sizeY * sizeZ);
        Clear();
    }
    
    void Clear() {
        for (auto& cell : cells) {
            cell.totalMass = 0.0f;
            cell.averageVelocity = { 0.0f, 0.0f, 0.0f };
            cell.particleCount = 0;
            cell.momentum = 0.0f;
        }
    }
    
    int GetIndex(int x, int y, int z) const {
        if (x < 0 || x >= gridSizeX || y < 0 || y >= gridSizeY || z < 0 || z >= gridSizeZ)
            return -1;
        return z * gridSizeX * gridSizeY + y * gridSizeX + x;
    }
    
    void WorldToGrid(float wx, float wy, float wz, int& gx, int& gy, int& gz) const {
        gx = (int)((wx - worldMinX) / cellSize);
        gy = (int)((wy - worldMinY) / cellSize);
        gz = (int)((wz - worldMinZ) / cellSize);
        gx = Clamp(gx, 0, gridSizeX - 1);
        gy = Clamp(gy, 0, gridSizeY - 1);
        gz = Clamp(gz, 0, gridSizeZ - 1);
    }
    
    void AddParticle(float wx, float wy, float wz, float mass, Vector3 velocity) {
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
    Vector3 CalculateBladeForce(Vector3 bladePos, float bladeHalfX, float bladeHalfY, float bladeHalfZ,
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
    int GetTotalParticleCount() const {
        int total = 0;
        for (const auto& cell : cells) {
            total += cell.particleCount;
        }
        return total;
    }
    
    float GetTotalMomentum() const {
        float total = 0.0f;
        for (const auto& cell : cells) {
            total += cell.momentum;
        }
        return total;
    }
    
    int GetGridSizeX() const { return gridSizeX; }
    int GetGridSizeY() const { return gridSizeY; }
    int GetGridSizeZ() const { return gridSizeZ; }
    float GetCellSize() const { return cellSize; }
    float GetWorldMinX() const { return worldMinX; }
    float GetWorldMinY() const { return worldMinY; }
    float GetWorldMinZ() const { return worldMinZ; }
    
    // Draw debug visualization of occupied voxels
    void DrawDebug() const {
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
};

// Perlin noise implementation
float PerlinFade(float t) {
    return t * t * t * (t * (t * 6 - 15) + 10);
}

float PerlinLerp(float t, float a, float b) {
    return a + t * (b - a);
}

float PerlinGrad(int hash, float x, float y) {
    int h = hash & 15;
    float u = h < 8 ? x : y;
    float v = h < 4 ? y : h == 12 || h == 14 ? x : 0;
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

class PerlinNoise {
private:
    std::vector<int> p;

public:
    PerlinNoise(unsigned int seed = 237) {
        p.resize(512);
        
        // Initialize permutation table
        for (int i = 0; i < 256; i++) {
            p[i] = i;
        }
        
        // Shuffle using seed
        for (int i = 255; i > 0; i--) {
            seed = seed * 1103515245 + 12345;
            int j = (seed / 65536) % (i + 1);
            std::swap(p[i], p[j]);
        }
        
        // Duplicate permutation table
        for (int i = 0; i < 256; i++) {
            p[256 + i] = p[i];
        }
    }
    
    float Noise(float x, float y) {
        int X = (int)floor(x) & 255;
        int Y = (int)floor(y) & 255;
        
        x -= floor(x);
        y -= floor(y);
        
        float u = PerlinFade(x);
        float v = PerlinFade(y);
        
        int A = p[X] + Y;
        int AA = p[A];
        int AB = p[A + 1];
        int B = p[X + 1] + Y;
        int BA = p[B];
        int BB = p[B + 1];
        
        return PerlinLerp(v, 
            PerlinLerp(u, PerlinGrad(p[AA], x, y), PerlinGrad(p[BA], x - 1, y)),
            PerlinLerp(u, PerlinGrad(p[AB], x, y - 1), PerlinGrad(p[BB], x - 1, y - 1))
        );
    }
    
    float OctaveNoise(float x, float y, int octaves, float persistence) {
        float total = 0;
        float frequency = 1;
        float amplitude = 1;
        float maxValue = 0;
        
        for (int i = 0; i < octaves; i++) {
            total += Noise(x * frequency, y * frequency) * amplitude;
            maxValue += amplitude;
            amplitude *= persistence;
            frequency *= 2;
        }
        
        return total / maxValue;
    }
};

void ModifyHeightmapWithDensity(DensityAutomata& densityGrid, float worldX, float worldZ, float radius, float densityAmount, float viscosity, float terrainScale, int heightmapSize) {
    // Convert world position to heightmap coordinates
    // Terrain is centered at origin, spans -10 to +10 in world space
    float hmX = (worldX + 10.0f) / (20.0f / heightmapSize);
    float hmZ = (worldZ + 10.0f) / (20.0f / heightmapSize);
    
    int centerX = (int)hmX;
    int centerZ = (int)hmZ;
    int radiusInSamples = (int)(radius / terrainScale);
    
    // Add density in a circular area
    for (int z = centerZ - radiusInSamples; z <= centerZ + radiusInSamples; z++) {
        for (int x = centerX - radiusInSamples; x <= centerX + radiusInSamples; x++) {
            if (x >= 0 && x < heightmapSize && z >= 0 && z < heightmapSize) {
                float dx = x - hmX;
                float dz = z - hmZ;
                float dist = sqrtf(dx * dx + dz * dz);
                
                if (dist <= radiusInSamples) {
                    // Smooth falloff based on distance
                    float falloff = 1.0f - (dist / radiusInSamples);
                    falloff = falloff * falloff; // Squared for smoother transition
                    
                    densityGrid.AddDensity(x, z, densityAmount * falloff, viscosity);
                }
            }
        }
    }
}

void ModifyHeightmap(std::vector<float>& heightSamples, int heightmapSize, float worldX, float worldZ, float radius, float heightIncrease, float terrainScale, float heightScale) {
    // Convert world position to heightmap coordinates
    // Terrain is centered at origin, spans -10 to +10 in world space
    float hmX = (worldX + 10.0f) / (20.0f / heightmapSize);
    float hmZ = (worldZ + 10.0f) / (20.0f / heightmapSize);
    
    int centerX = (int)hmX;
    int centerZ = (int)hmZ;
    int radiusInSamples = (int)(radius / terrainScale);
    
    // Modify heightmap in a circular area
    for (int z = centerZ - radiusInSamples; z <= centerZ + radiusInSamples; z++) {
        for (int x = centerX - radiusInSamples; x <= centerX + radiusInSamples; x++) {
            if (x >= 0 && x < heightmapSize && z >= 0 && z < heightmapSize) {
                float dx = x - hmX;
                float dz = z - hmZ;
                float dist = sqrtf(dx * dx + dz * dz);
                
                if (dist <= radiusInSamples) {
                    // Smooth falloff based on distance
                    float falloff = 1.0f - (dist / radiusInSamples);
                    falloff = falloff * falloff; // Squared for smoother transition
                    
                    heightSamples[z * heightmapSize + x] += heightIncrease * falloff;
                    // Clamp height to valid range
                    if (heightSamples[z * heightmapSize + x] > heightScale) {
                        heightSamples[z * heightmapSize + x] = heightScale;
                    }
                }
            }
        }
    }
}

Image GeneratePerlinNoiseHeightmap(int width, int height, float scale, int octaves, float persistence) {
    PerlinNoise perlin;
    Image image = GenImageColor(width, height, ::BLACK);
    ::Color* pixels = LoadImageColors(image);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float nx = (float)x / width * scale;
            float ny = (float)y / height * scale;
            
            float noiseValue = perlin.OctaveNoise(nx, ny, octaves, persistence);
            // Normalize to 0-1 range
            noiseValue = (noiseValue + 1.0f) / 2.0f;
            
            unsigned char value = (unsigned char)(noiseValue * 255);
            pixels[y * width + x] = ::Color{ value, value, value, 255 };
            pixels[y * width + x] = {0,0,0,255};
        }
    }
    
    // Update the image with the new pixel data
    unsigned char* imageData = (unsigned char*)image.data;
    for (int i = 0; i < width * height; i++) {
        imageData[i * 4 + 0] = pixels[i].r;
        imageData[i * 4 + 1] = pixels[i].g;
        imageData[i * 4 + 2] = pixels[i].b;
        imageData[i * 4 + 3] = pixels[i].a;
    }
    
    UnloadImageColors(pixels);
    
    return image;
}

void UpdateOrbitalCamera(Camera3D* camera, float* yaw, float* pitch, float* radius, Vector2* previousMousePos) {
    Vector2 mousePos = GetMousePosition();
    Vector2 mouseDelta = Vector2Subtract(mousePos, *previousMousePos);
    *previousMousePos = mousePos;

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        *yaw += mouseDelta.x * 0.005f;
        *pitch += mouseDelta.y * 0.005f;
        *pitch = Clamp(*pitch, -1.57f, 1.57f);
    }

    float mouseWheel = GetMouseWheelMove();
    if (mouseWheel != 0) {
        *radius -= mouseWheel * 0.5f;
        *radius = Clamp(*radius, 2.0f, 50.0f);
    }

    // Convert spherical coordinates to Cartesian
    camera->position.x = camera->target.x + (*radius) * cosf(*pitch) * cosf(*yaw);
    camera->position.z = camera->target.z + (*radius) * cosf(*pitch) * sinf(*yaw);
    camera->position.y = camera->target.y + (*radius) * sinf(*pitch);
}

int main() {
    const int screenWidth = 800;
    const int screenHeight = 450;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "Jolt Physics with Heightmap - raylib");

    // Initialize Jolt Physics
    RegisterDefaultAllocator();
    Factory::sInstance = new Factory();
    RegisterTypes();

    // Create temp allocator
    TempAllocatorImpl temp_allocator(10 * 1024 * 1024);

    // Create job system
    JobSystemThreadPool job_system(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);

    // Create physics system
    const uint cMaxBodies = 2048;
    const uint cNumBodyMutexes = 0;
    const uint cMaxBodyPairs = 4096;
    const uint cMaxContactConstraints = 4096;
    const int maxSphereCount = 200; // Limit sphere count to prevent physics overload

    BPLayerInterfaceImpl broad_phase_layer_interface;
    ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;
    ObjectLayerPairFilterImpl object_vs_object_layer_filter;

    PhysicsSystem physics_system;
    physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
        broad_phase_layer_interface, object_vs_broadphase_layer_filter, object_vs_object_layer_filter);

    // Contact listener
    MyContactListener contact_listener;
    physics_system.SetContactListener(&contact_listener);

    BodyInterface &body_interface = physics_system.GetBodyInterface();

    Camera3D camera = { 0 };
    camera.position = Vector3{ 10.0f, 10.0f, 10.0f };
    camera.target = Vector3{ 0.0f, 0.0f, 0.0f };
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    float cameraYaw = atan2f(camera.position.z - camera.target.z, camera.position.x - camera.target.x);
    float cameraRadius = Vector3Distance(camera.position, camera.target);
    float cameraPitch = asinf((camera.position.y - camera.target.y) / cameraRadius);

    Vector2 previousMousePos = GetMousePosition();

    // Load shader
    Shader shader = LoadShader("shaders/directional_light.vs", "shaders/directional_light.fs");
    
    // Load heightmap shader
    Shader heightmapShader = LoadShader("shaders/heightmap.vs", "shaders/heightmap.fs");
    
    // Get shader uniform locations for directional light shader
    int lightDirLoc = GetShaderLocation(shader, "lightDirection");
    int lightColorLoc = GetShaderLocation(shader, "lightColor");
    int ambientColorLoc = GetShaderLocation(shader, "ambientColor");
    int viewPosLoc = GetShaderLocation(shader, "viewPos");
    int ambientStrengthLoc = GetShaderLocation(shader, "ambientStrength");
    int specularStrengthLoc = GetShaderLocation(shader, "specularStrength");
    int shininessLoc = GetShaderLocation(shader, "shininess");
    
    // Get shader uniform locations for heightmap shader
    int hm_lightDirLoc = GetShaderLocation(heightmapShader, "lightDirection");
    int hm_lightColorLoc = GetShaderLocation(heightmapShader, "lightColor");
    int hm_ambientColorLoc = GetShaderLocation(heightmapShader, "ambientColor");
    int hm_viewPosLoc = GetShaderLocation(heightmapShader, "viewPos");
    int hm_ambientStrengthLoc = GetShaderLocation(heightmapShader, "ambientStrength");
    int hm_specularStrengthLoc = GetShaderLocation(heightmapShader, "specularStrength");
    int hm_shininessLoc = GetShaderLocation(heightmapShader, "shininess");
    int hm_heightScaleLoc = GetShaderLocation(heightmapShader, "heightScale");
    
    // Set up directional light properties
    Vector3 lightDirection = { -0.5f, -1.0f, -0.3f };
    lightDirection = Vector3Normalize(lightDirection);
    Vector3 lightColor = { 1.0f, 1.0f, 1.0f };
    Vector3 ambientColor = { 0.3f, 0.3f, 0.3f };
    float ambientStrength = 0.2f;
    float specularStrength = 0.5f;
    float shininess = 32.0f;
    float lightAngle = 0.0f;
    
    // Set shader values
    SetShaderValue(shader, lightDirLoc, &lightDirection, SHADER_UNIFORM_VEC3);
    SetShaderValue(shader, lightColorLoc, &lightColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(shader, ambientColorLoc, &ambientColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(shader, ambientStrengthLoc, &ambientStrength, SHADER_UNIFORM_FLOAT);
    SetShaderValue(shader, specularStrengthLoc, &specularStrength, SHADER_UNIFORM_FLOAT);
    SetShaderValue(shader, shininessLoc, &shininess, SHADER_UNIFORM_FLOAT);
    
    // Set heightmap shader values
    SetShaderValue(heightmapShader, hm_lightDirLoc, &lightDirection, SHADER_UNIFORM_VEC3);
    SetShaderValue(heightmapShader, hm_lightColorLoc, &lightColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(heightmapShader, hm_ambientColorLoc, &ambientColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(heightmapShader, hm_ambientStrengthLoc, &ambientStrength, SHADER_UNIFORM_FLOAT);
    SetShaderValue(heightmapShader, hm_specularStrengthLoc, &specularStrength, SHADER_UNIFORM_FLOAT);
    SetShaderValue(heightmapShader, hm_shininessLoc, &shininess, SHADER_UNIFORM_FLOAT);
    float heightScale = 2.0f;
    SetShaderValue(heightmapShader, hm_heightScaleLoc, &heightScale, SHADER_UNIFORM_FLOAT);
    
    // Generate Perlin noise heightmap
    int heightmapSize = 256;
    Image heightmapImage = GeneratePerlinNoiseHeightmap(heightmapSize, heightmapSize, 5.0f, 6, 0.5f);
    Texture2D heightmapTexture = LoadTextureFromImage(heightmapImage);
    SetTextureFilter(heightmapTexture, TEXTURE_FILTER_BILINEAR);
    
    // Create heightmap collision shape from the image
    std::vector<float> heightSamples;
    heightSamples.resize(heightmapSize * heightmapSize);
    ::Color* pixels = LoadImageColors(heightmapImage);
    
    for (int y = 0; y < heightmapSize; y++) {
        for (int x = 0; x < heightmapSize; x++) {
            // Convert grayscale to height (0-1 range scaled by heightScale)
            float height = (pixels[y * heightmapSize + x].r / 255.0f) * heightScale;
            heightSamples[y * heightmapSize + x] = height;
        }
    }
    UnloadImageColors(pixels);
    
    // Create Jolt HeightFieldShape
    float terrainScale = 20.0f / (float)heightmapSize; // Map to 20x20 world units
    
    // Initialize density cellular automata
    DensityAutomata densityGrid(heightmapSize, terrainScale);
    
    // Initialize particle voxel grid for force calculations
    // Grid covers -15 to 15 in X and Z, 0 to 5 in Y (where particles are active)
    int voxelGridSizeXZ = 60;  // 30m / 0.5m cell size
    int voxelGridSizeY = 10;   // 5m / 0.5m cell size
    float voxelCellSize = 0.5f;
    ParticleVoxelGrid particleVoxelGrid(voxelGridSizeXZ, voxelGridSizeY, voxelGridSizeXZ,
                                         voxelCellSize,
                                         -15.0f, 0.0f, -15.0f,   // min bounds
                                         15.0f, 5.0f, 15.0f);    // max bounds
    float lastParticleForce = 0.0f;  // For display purposes
    Vector3 lastParticleForceVec = { 0.0f, 0.0f, 0.0f }; // Force vector from particles
    
    HeightFieldShapeSettings heightfield_settings(heightSamples.data(), Vec3(0, 0, 0), 
        Vec3(terrainScale, 1.0f, terrainScale), heightmapSize);
    ShapeSettings::ShapeResult heightfield_shape_result = heightfield_settings.Create();
    ShapeRefC heightfield_shape = heightfield_shape_result.Get();
    
    // Create static body for heightmap
    BodyCreationSettings heightmap_body_settings(heightfield_shape, 
        RVec3(-10.0, 0.0, -10.0), // Position to center the heightmap
        Quat::sIdentity(), 
        EMotionType::Static, 
        Layers::NON_MOVING);
    heightmap_body_settings.mFriction = 50.0f; // High friction for terrain
    Body* heightmap_body = body_interface.CreateBody(heightmap_body_settings);
    body_interface.AddBody(heightmap_body->GetID(), EActivation::DontActivate);
    
    UnloadImage(heightmapImage);
    
    // Create plane mesh with high resolution for smooth heightmap displacement
    Mesh planeMesh = GenMeshPlane(20.0f, 20.0f, 128, 128);
    Model planeModel = LoadModelFromMesh(planeMesh);
    planeModel.materials[0].shader = heightmapShader;
    planeModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = heightmapTexture;
    
    // Create a sphere model for rendering
    float sphereRadius = 0.1f;
    float sphereDensity = 1500.0f; // kg/m³ (dirt/soil density)
    float sphereVolume = (4.0f / 3.0f) * 3.14159f * sphereRadius * sphereRadius * sphereRadius;
    float sphereMass = sphereDensity * sphereVolume; // Mass of each sphere
    Model sphereModel = LoadModelFromMesh(GenMeshSphere(sphereRadius, 32, 32));
    sphereModel.materials[0].shader = shader;
    
    // Create a cube model for the moving cube
    // Define cube half-extents for each axis
    float cubeHalfX = 1.0f;   // Half width (X)
    float cubeHalfY = 0.5f;   // Half height (Y)
    float cubeHalfZ = 0.1f;   // Half depth (Z)
    Model cubeModel = LoadModelFromMesh(GenMeshCube(cubeHalfX * 2.0f, cubeHalfY * 2.0f, cubeHalfZ * 2.0f));
    cubeModel.materials[0].shader = shader;
    
    // Create physics body for the cube (kinematic so it can push spheres)
    // BoxShapeSettings takes half-extents
    BoxShapeSettings cube_shape_settings(Vec3(cubeHalfX, cubeHalfY, cubeHalfZ));
    ShapeSettings::ShapeResult cube_shape_result = cube_shape_settings.Create();
    ShapeRefC cube_shape = cube_shape_result.Get();
    
    // 45 degree rotation around Y axis
    Quat cubeRotation = Quat::sRotation(Vec3(0, 1, 0), 3.14159f / 4.0f);
    
    BodyCreationSettings cube_body_settings(cube_shape,
        RVec3(-12.0f, 1.0f, 0.0f),
        cubeRotation,
        EMotionType::Kinematic,
        Layers::MOVING);
    cube_body_settings.mFriction = 0.5f;
    cube_body_settings.mRestitution = 0.1f;
    
    Body* cube_body = body_interface.CreateBody(cube_body_settings);
    BodyID cube_body_id = cube_body->GetID();
    body_interface.AddBody(cube_body_id, EActivation::Activate);
    
    // Moving cube properties
    Vector3 cubePosition = { -12.0f, 1.0f, 0.0f }; // Start from left side
    float cubeSpeed = 3.0f; // Units per second
    float cubeCurrentSpeed = 3.0f; // Actual current speed (affected by resistance)
    float cubeMinX = -12.0f;
    float cubeMaxX = 12.0f;
    float cubeDigDepth = 0.3f; // How deep the cube digs into terrain
    float sphereSpawnAccumulator = 0.0f; // Accumulate displaced volume for spawning spheres
    const float stoppedTimeThreshold = 0.5f; // Time in seconds before sphere converts to earth
    
    // Terrain resistance physics
    float cubeMass = 100.0f; // Mass of the cube in kg
    float cubeEnginePower = 500.0f; // Engine force in Newtons
    float terrainResistanceCoeff = 50.0f; // Resistance coefficient (force per unit volume displaced)
    float lastTerrainForce = 0.0f; // For display purposes
    
    // GUI settings for cube (applied on reset)
    float guiCubeStartX = -12.0f;
    float guiCubeStartZ = 0.0f;
    float guiCubeSpeed = 3.0f;
    float guiCubeHeight = 0.0f; // Height offset for cube Y position
    float guiEnginePower = 500.0f; // Engine power setting
    float guiResistanceCoeff = 50.0f; // Terrain resistance coefficient
    bool showGui = true;
    int activeSlider = -1; // Track which slider is being dragged
    
    // Circle mode settings
    bool circleMode = false; // false = linear, true = circle
    float guiCircleCenterX = 0.0f;
    float guiCircleCenterZ = 0.0f;
    float guiCircleRadius = 5.0f;
    float guiBladeRotation = 45.0f; // Rotation angle in degrees
    float circleAngle = 0.0f; // Current angle for circle movement
    
    // Manual drive mode settings
    bool manualMode = false; // Manual arrow key control mode
    int driveMode = 0; // 0 = linear, 1 = circle, 2 = manual
    float manualRotation = 0.0f; // Current rotation in manual mode (radians)
    Vector3 manualVelocity = { 0.0f, 0.0f, 0.0f }; // Current velocity in manual mode
    float manualAcceleration = 10.0f; // Acceleration when pressing arrow keys
    float manualDeceleration = 5.0f; // Deceleration when not pressing keys
    float manualTurnSpeed = 2.0f; // Rotation speed in radians per second
    
    // Debug visualization settings
    bool showVoxelDebug = false; // Toggle for voxel debug visualization

    // List to hold dynamic spheres
    std::vector<PhysicsSphere> dynamicSpheres;

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        UpdateOrbitalCamera(&camera, &cameraYaw, &cameraPitch, &cameraRadius, &previousMousePos);

        // Mouse click to spawn sphere at clicked terrain point, 5m above
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            // Get mouse ray for raycasting
            Ray ray = GetMouseRay(GetMousePosition(), camera);
            
            // Raycast against the terrain to find intersection point
            Vector3 spawnPosition = { 0.0f, 5.0f, 0.0f }; // Default position
            bool hitTerrain = false;
            
            // Sample along the ray to find terrain intersection
            float maxDistance = 100.0f;
            float step = 0.5f;
            for (float dist = 0.0f; dist < maxDistance; dist += step) {
                Vector3 testPoint = {
                    ray.position.x + ray.direction.x * dist,
                    ray.position.y + ray.direction.y * dist,
                    ray.position.z + ray.direction.z * dist
                };
                
                // Check if point is within terrain bounds
                if (testPoint.x >= -10.0f && testPoint.x <= 10.0f &&
                    testPoint.z >= -10.0f && testPoint.z <= 10.0f) {
                    
                    // Sample heightmap at this position
                    float hmX = (testPoint.x + 10.0f) / (20.0f / heightmapSize);
                    float hmZ = (testPoint.z + 10.0f) / (20.0f / heightmapSize);
                    int ix = (int)hmX;
                    int iz = (int)hmZ;
                    
                    if (ix >= 0 && ix < heightmapSize && iz >= 0 && iz < heightmapSize) {
                        float terrainHeight = heightSamples[iz * heightmapSize + ix];
                        
                        // Check if ray point is below terrain surface
                        if (testPoint.y <= terrainHeight) {
                            spawnPosition = { testPoint.x, terrainHeight + 5.0f, testPoint.z };
                            hitTerrain = true;
                            break;
                        }
                    }
                }
            }
            
            // Create 10 spheres in a cluster
            for (int i = 0; i < 10; i++) {
                // Random offset within a small radius
                float offsetX = ((float)GetRandomValue(-50, 50) / 100.0f);
                float offsetY = ((float)GetRandomValue(0, 100) / 100.0f);
                float offsetZ = ((float)GetRandomValue(-50, 50) / 100.0f);
                
                Vector3 sphereSpawnPos = {
                    spawnPosition.x + offsetX,
                    spawnPosition.y + offsetY,
                    spawnPosition.z + offsetZ
                };
                
                // Create sphere shape
                SphereShapeSettings sphere_shape_settings(sphereRadius); // 0.5m radius
                ShapeSettings::ShapeResult sphere_shape_result = sphere_shape_settings.Create();
                ShapeRefC sphere_shape = sphere_shape_result.Get();
                
                // Create dynamic body for sphere at clicked position, 5m above terrain
                BodyCreationSettings sphere_body_settings(sphere_shape,
                    RVec3(sphereSpawnPos.x, sphereSpawnPos.y, sphereSpawnPos.z),
                    Quat::sIdentity(),
                    EMotionType::Dynamic,
                    Layers::MOVING);
                
                // Set high friction and damping to reduce rolling (act like lumps)
                sphere_body_settings.mFriction = 50.0f; // Almost sticky friction
                sphere_body_settings.mRestitution = 0.05f; // Very low bounce
                sphere_body_settings.mLinearDamping = 2.0f; // Very high linear damping
                sphere_body_settings.mAngularDamping = 5.0f; // Extremely high angular damping to stop rolling
                
                Body* sphere_body = body_interface.CreateBody(sphere_body_settings);
                BodyID sphere_id = sphere_body->GetID();
                body_interface.AddBody(sphere_id, EActivation::Activate);
                
                // Random color for the sphere // Greenish for dirt
                ::Color sphereColor = {
                    (unsigned char)0,
                    (unsigned char)200,
                    (unsigned char)0,
                    255
                };
                
                Vector3 initialVelocity = { 0.0f, 0.0f, 0.0f };
                dynamicSpheres.push_back({sphere_id, sphereColor, 0.0f, false, spawnPosition, initialVelocity, sphereMass});
            }
        }

        // Update physics (60 Hz simulation)
        const float deltaTime = GetFrameTime();
        const int collisionSteps = 1;
        physics_system.Update(deltaTime, collisionSteps, &temp_allocator, &job_system);
        
        // Calculate terrain resistance force before moving
        // This is done by looking ahead at what terrain we'll encounter
        float terrainResistanceForce = 0.0f;
        float cubeBottomY_preview = guiCubeHeight; // Bottom of cube (cubeHalfY + guiCubeHeight - cubeHalfY = guiCubeHeight)
        
        // Sample terrain ahead of the cube to calculate resistance
        if (cubePosition.x >= -10.0f && cubePosition.x <= 10.0f &&
            cubePosition.z >= -10.0f && cubePosition.z <= 10.0f) {
            
            float rotationRad;
            if (driveMode == 2) {
                rotationRad = manualRotation + guiBladeRotation * 3.14159f / 180.0f;
            } else if (driveMode == 1) {
                rotationRad = circleAngle + 3.14159f / 2.0f + guiBladeRotation * 3.14159f / 180.0f;
            } else {
                rotationRad = guiBladeRotation * 3.14159f / 180.0f;
            }
            float cosR = cosf(rotationRad);
            float sinR = sinf(rotationRad);
            
            // Calculate the 4 corners of the rotated cube footprint
            float previewCorners[4][2] = {
                { cubeHalfX * cosR - cubeHalfZ * sinR,  cubeHalfX * sinR + cubeHalfZ * cosR},
                {-cubeHalfX * cosR - cubeHalfZ * sinR, -cubeHalfX * sinR + cubeHalfZ * cosR},
                { cubeHalfX * cosR + cubeHalfZ * sinR,  cubeHalfX * sinR - cubeHalfZ * cosR},
                {-cubeHalfX * cosR + cubeHalfZ * sinR, -cubeHalfX * sinR - cubeHalfZ * cosR}
            };
            
            float previewMinX = previewCorners[0][0], previewMaxX = previewCorners[0][0];
            float previewMinZ = previewCorners[0][1], previewMaxZ = previewCorners[0][1];
            for (int i = 1; i < 4; i++) {
                if (previewCorners[i][0] < previewMinX) previewMinX = previewCorners[i][0];
                if (previewCorners[i][0] > previewMaxX) previewMaxX = previewCorners[i][0];
                if (previewCorners[i][1] < previewMinZ) previewMinZ = previewCorners[i][1];
                if (previewCorners[i][1] > previewMaxZ) previewMaxZ = previewCorners[i][1];
            }
            
            int hmMinX_p = (int)((cubePosition.x + previewMinX + 10.0f) / (20.0f / heightmapSize));
            int hmMaxX_p = (int)((cubePosition.x + previewMaxX + 10.0f) / (20.0f / heightmapSize));
            int hmMinZ_p = (int)((cubePosition.z + previewMinZ + 10.0f) / (20.0f / heightmapSize));
            int hmMaxZ_p = (int)((cubePosition.z + previewMaxZ + 10.0f) / (20.0f / heightmapSize));
            
            // Calculate total penetration depth (represents displaced volume)
            float totalPenetration = 0.0f;
            int cellCount = 0;
            for (int z = hmMinZ_p; z <= hmMaxZ_p; z++) {
                for (int x = hmMinX_p; x <= hmMaxX_p; x++) {
                    if (x >= 0 && x < heightmapSize && z >= 0 && z < heightmapSize) {
                        // Convert heightmap cell back to world coordinates
                        float cellWorldX = (float)x * (20.0f / heightmapSize) - 10.0f;
                        float cellWorldZ = (float)z * (20.0f / heightmapSize) - 10.0f;
                        
                        // Transform cell position to cube's local space (rotate by -rotationRad)
                        float localX = (cellWorldX - cubePosition.x) * cosR + (cellWorldZ - cubePosition.z) * sinR;
                        float localZ = -(cellWorldX - cubePosition.x) * sinR + (cellWorldZ - cubePosition.z) * cosR;
                        
                        // Check if point is inside the cube's footprint (in local space, it's axis-aligned)
                        if (fabsf(localX) > cubeHalfX || fabsf(localZ) > cubeHalfZ) {
                            continue; // Point is outside the rotated cube footprint
                        }
                        
                        float terrainHeight = heightSamples[z * heightmapSize + x];
                        if (cubeBottomY_preview < terrainHeight) {
                            float penetration = terrainHeight - cubeBottomY_preview;
                            totalPenetration += penetration;
                            cellCount++;
                        }
                    }
                }
            }
            
            // Calculate resistance force: F = coefficient * penetration_area * speed^2
            // More penetration = more resistance, faster = more resistance (drag-like)
            float cellArea = terrainScale * terrainScale;
            float penetrationVolume = totalPenetration * cellArea;
            // Scale up the resistance significantly - multiply by large factor to make it meaningful
            // penetrationVolume is tiny because terrainScale is small (~0.078), so we need a big multiplier
            terrainResistanceForce = guiResistanceCoeff * penetrationVolume * 1000.0f * (1.0f + cubeCurrentSpeed * cubeCurrentSpeed);
        }
        
        lastTerrainForce = terrainResistanceForce;
        
        // Physics-based speed calculation
        // Net force = Engine power - Terrain resistance
        // Acceleration = Net force / Mass
        float netForce = guiEnginePower - terrainResistanceForce;
        float acceleration = netForce / cubeMass;
        
        // Update current speed based on acceleration
        cubeCurrentSpeed += acceleration * deltaTime;
        
        // Clamp speed to reasonable bounds (can't go negative, can't exceed max)
        float maxSpeed = guiCubeSpeed;
        cubeCurrentSpeed = Clamp(cubeCurrentSpeed, 0.01f, maxSpeed); // Minimum speed to prevent stalling completely
        
        // If resistance exceeds engine power, apply stronger braking
        if (terrainResistanceForce > guiEnginePower) {
            float overloadRatio = terrainResistanceForce / guiEnginePower;
            cubeCurrentSpeed *= (1.0f - 0.1f * fminf(overloadRatio - 1.0f, 5.0f) * deltaTime * 60.0f); // Scale braking with overload
        }
        
        // Update moving cube position based on mode
        if (driveMode == 2) {
            // Manual mode: arrow key control
            // Rotation with left/right arrows
            if (IsKeyDown(KEY_LEFT)) {
                manualRotation += manualTurnSpeed * deltaTime;
            }
            if (IsKeyDown(KEY_RIGHT)) {
                manualRotation -= manualTurnSpeed * deltaTime;
            }
            
            // Forward/backward acceleration with up/down arrows
            float forwardInput = 0.0f;
            if (IsKeyDown(KEY_UP)) {
                forwardInput = 1.0f;
            }
            if (IsKeyDown(KEY_DOWN)) {
                forwardInput = -0.5f; // Slower reverse
            }
            
            // Calculate forward direction based on rotation
            float forwardX = cosf(manualRotation);
            float forwardZ = -sinf(manualRotation);
            
            if (forwardInput != 0.0f) {
                // Accelerate in the forward direction
                manualVelocity.x += forwardX * forwardInput * manualAcceleration * deltaTime;
                manualVelocity.z += forwardZ * forwardInput * manualAcceleration * deltaTime;
            } else {
                // Decelerate when no input
                float speed = sqrtf(manualVelocity.x * manualVelocity.x + manualVelocity.z * manualVelocity.z);
                if (speed > 0.01f) {
                    float decel = manualDeceleration * deltaTime;
                    float newSpeed = fmaxf(0.0f, speed - decel);
                    manualVelocity.x *= newSpeed / speed;
                    manualVelocity.z *= newSpeed / speed;
                } else {
                    manualVelocity.x = 0.0f;
                    manualVelocity.z = 0.0f;
                }
            }
            
            // Apply terrain resistance to manual velocity
            float manualSpeed = sqrtf(manualVelocity.x * manualVelocity.x + manualVelocity.z * manualVelocity.z);
            if (manualSpeed > 0.01f && terrainResistanceForce > 0.0f) {
                float resistanceDecel = (terrainResistanceForce / cubeMass) * deltaTime;
                float newSpeed = fmaxf(0.0f, manualSpeed - resistanceDecel);
                manualVelocity.x *= newSpeed / manualSpeed;
                manualVelocity.z *= newSpeed / manualSpeed;
            }
            
            // Clamp max speed
            float maxManualSpeed = guiCubeSpeed;
            manualSpeed = sqrtf(manualVelocity.x * manualVelocity.x + manualVelocity.z * manualVelocity.z);
            if (manualSpeed > maxManualSpeed) {
                manualVelocity.x *= maxManualSpeed / manualSpeed;
                manualVelocity.z *= maxManualSpeed / manualSpeed;
            }
            
            // Update position
            cubePosition.x += manualVelocity.x * deltaTime;
            cubePosition.z += manualVelocity.z * deltaTime;
            
            // Update current speed for display
            cubeCurrentSpeed = sqrtf(manualVelocity.x * manualVelocity.x + manualVelocity.z * manualVelocity.z);
            
            // Update rotation
            cubeRotation = Quat::sRotation(Vec3(0, 1, 0), manualRotation + guiBladeRotation * 3.14159f / 180.0f);
        } else if (driveMode == 1) {
            // Circle mode: rotate around center point
            float angularSpeed = cubeCurrentSpeed / guiCircleRadius; // radians per second
            circleAngle += angularSpeed * deltaTime;
            if (circleAngle > 2.0f * 3.14159f) {
                circleAngle -= 2.0f * 3.14159f;
            }
            
            cubePosition.x = guiCircleCenterX + guiCircleRadius * cosf(circleAngle);
            cubePosition.z = guiCircleCenterZ + guiCircleRadius * sinf(circleAngle);
            
            // Update cube rotation to be tangent to circle + blade rotation
            float tangentAngle = circleAngle + 3.14159f / 2.0f; // Perpendicular to radius
            float totalRotation = tangentAngle + guiBladeRotation * 3.14159f / 180.0f;
            cubeRotation = Quat::sRotation(Vec3(0, 1, 0), totalRotation);
        } else {
            // Linear mode: move left to right
            cubePosition.x += cubeCurrentSpeed * deltaTime;
            if (cubePosition.x > cubeMaxX) {
                // Reset cube with GUI values
                cubePosition.x = guiCubeStartX;
                cubePosition.z = guiCubeStartZ;
                cubeCurrentSpeed = guiCubeSpeed; // Reset to base speed
                cubeMinX = guiCubeStartX;
            }
            // Update rotation from GUI
            cubeRotation = Quat::sRotation(Vec3(0, 1, 0), guiBladeRotation * 3.14159f / 180.0f);
        }
        
        // Set cube Y position from height slider only (no terrain following)
        cubePosition.y = cubeHalfY + guiCubeHeight;
        
        // Update cube physics body position and rotation (kinematic body)
        body_interface.SetPositionAndRotation(cube_body_id, 
            RVec3(cubePosition.x, cubePosition.y, cubePosition.z), 
            cubeRotation, 
            EActivation::Activate);
        // Set velocity so physics engine knows it's moving (helps with collision response)
        if (driveMode == 2) {
            // Manual mode velocity
            body_interface.SetLinearVelocity(cube_body_id, Vec3(manualVelocity.x, 0.0f, manualVelocity.z));
        } else if (driveMode == 1) {
            // Tangent velocity for circle motion
            float vx = -cubeCurrentSpeed * sinf(circleAngle);
            float vz = cubeCurrentSpeed * cosf(circleAngle);
            body_interface.SetLinearVelocity(cube_body_id, Vec3(vx, 0.0f, vz));
        } else {
            body_interface.SetLinearVelocity(cube_body_id, Vec3(cubeCurrentSpeed, 0.0f, 0.0f));
        }
        
        // Cube terrain deformation - dig into terrain and spawn spheres
        // Only deform if cube is within terrain bounds AND low enough to touch terrain
        float cubeBottomY = cubePosition.y - cubeHalfY; // Bottom of the cube
        if (cubePosition.x >= -10.0f && cubePosition.x <= 10.0f &&
            cubePosition.z >= -10.0f && cubePosition.z <= 10.0f &&
            cubeBottomY < heightScale) { // Only dig if cube bottom is below max terrain height
            
            // Calculate rotated cube footprint - account for blade rotation
            float rotationRad;
            if (driveMode == 2) {
                rotationRad = manualRotation + guiBladeRotation * 3.14159f / 180.0f;
            } else if (driveMode == 1) {
                rotationRad = circleAngle + 3.14159f / 2.0f + guiBladeRotation * 3.14159f / 180.0f;
            } else {
                rotationRad = guiBladeRotation * 3.14159f / 180.0f;
            }
            float cosR = cosf(rotationRad);
            float sinR = sinf(rotationRad);
            
            // Calculate the 4 corners of the rotated cube footprint
            float corners[4][2] = {
                { cubeHalfX * cosR - cubeHalfZ * sinR,  cubeHalfX * sinR + cubeHalfZ * cosR},
                {-cubeHalfX * cosR - cubeHalfZ * sinR, -cubeHalfX * sinR + cubeHalfZ * cosR},
                { cubeHalfX * cosR + cubeHalfZ * sinR,  cubeHalfX * sinR - cubeHalfZ * cosR},
                {-cubeHalfX * cosR + cubeHalfZ * sinR, -cubeHalfX * sinR - cubeHalfZ * cosR}
            };
            
            // Find axis-aligned bounding box of rotated footprint
            float minX = corners[0][0], maxX = corners[0][0];
            float minZ = corners[0][1], maxZ = corners[0][1];
            for (int i = 1; i < 4; i++) {
                if (corners[i][0] < minX) minX = corners[i][0];
                if (corners[i][0] > maxX) maxX = corners[i][0];
                if (corners[i][1] < minZ) minZ = corners[i][1];
                if (corners[i][1] > maxZ) maxZ = corners[i][1];
            }
            
            // Calculate cube footprint in heightmap coordinates using rotated bounds
            int hmMinX = (int)((cubePosition.x + minX + 10.0f) / (20.0f / heightmapSize));
            int hmMaxX = (int)((cubePosition.x + maxX + 10.0f) / (20.0f / heightmapSize));
            int hmMinZ = (int)((cubePosition.z + minZ + 10.0f) / (20.0f / heightmapSize));
            int hmMaxZ = (int)((cubePosition.z + maxZ + 10.0f) / (20.0f / heightmapSize));
            
            float totalDisplacedVolume = 0.0f;
            
            // Dig terrain under the cube - only if cube bottom is below terrain height at that point
            for (int z = hmMinZ; z <= hmMaxZ; z++) {
                for (int x = hmMinX; x <= hmMaxX; x++) {
                    if (x >= 0 && x < heightmapSize && z >= 0 && z < heightmapSize) {
                        // Convert heightmap cell back to world coordinates
                        float cellWorldX = (float)x * (20.0f / heightmapSize) - 10.0f;
                        float cellWorldZ = (float)z * (20.0f / heightmapSize) - 10.0f;
                        
                        // Transform cell position to cube's local space (rotate by -rotationRad)
                        float localX = (cellWorldX - cubePosition.x) * cosR + (cellWorldZ - cubePosition.z) * sinR;
                        float localZ = (cellWorldX - cubePosition.x) * sinR + (cellWorldZ - cubePosition.z) * cosR;
                        
                        // Check if point is inside the cube's footprint (in local space, it's axis-aligned)
                        if (fabsf(localX) > cubeHalfX || fabsf(localZ) > cubeHalfZ) {
                            continue; // Point is outside the rotated cube footprint
                        }
                        
                        int idx = z * heightmapSize + x;
                        float currentHeight = heightSamples[idx];
                        
                        // Only dig if cube bottom is below terrain at this point
                        if (cubeBottomY < currentHeight && currentHeight > 0.0f) {
                            float digAmount = cubeDigDepth * deltaTime * 5.0f; // Scale with frame time
                            float actualDig = fminf(digAmount, currentHeight - cubeBottomY);
                            actualDig = fminf(actualDig, currentHeight); // Don't go below 0
                            if (actualDig > 0.0f) {
                                heightSamples[idx] -= actualDig;
                                if (heightSamples[idx] < 0.0f) heightSamples[idx] = 0.0f;
                                
                                // Calculate displaced volume (per cell)
                                float cellArea = terrainScale * terrainScale;
                                totalDisplacedVolume += actualDig * cellArea;
                            }
                        }
                    }
                }
            }
            
            // Accumulate displaced volume and spawn spheres
            sphereSpawnAccumulator += totalDisplacedVolume;
            const float sphereVolume = (4.0f / 3.0f) * 3.14159f * sphereRadius * sphereRadius * sphereRadius;
            
            // Limit sphere spawning if we have too many
            while (sphereSpawnAccumulator >= sphereVolume && (int)dynamicSpheres.size() < maxSphereCount) {
                sphereSpawnAccumulator -= sphereVolume;
                
                // Spawn sphere IN FRONT of the cube so it gets pushed
                float spawnOffsetX = cubeHalfX + sphereRadius + 0.15f; // In front of cube
                float spawnOffsetZ = ((float)GetRandomValue(-50, 50) / 100.0f) * cubeHalfZ;
                float spawnY = cubePosition.y - cubeHalfY + sphereRadius;
                
                // Spawn in front of the cube (positive X direction since cube moves right)
                Vector3 sphereSpawnPos = {
                    cubePosition.x + spawnOffsetX, // Spawn IN FRONT of the cube
                    spawnY + 0.3f,
                    cubePosition.z + spawnOffsetZ
                };
                
                // Create sphere shape
                SphereShapeSettings sphere_shape_settings(sphereRadius);
                ShapeSettings::ShapeResult sphere_shape_result = sphere_shape_settings.Create();
                ShapeRefC sphere_shape = sphere_shape_result.Get();
                
                // Create dynamic body for sphere
                BodyCreationSettings sphere_body_settings(sphere_shape,
                    RVec3(sphereSpawnPos.x, sphereSpawnPos.y, sphereSpawnPos.z),
                    Quat::sIdentity(),
                    EMotionType::Dynamic,
                    Layers::MOVING);
                
                sphere_body_settings.mFriction = 50.0f;
                sphere_body_settings.mRestitution = 0.05f;
                sphere_body_settings.mLinearDamping = 2.0f;
                sphere_body_settings.mAngularDamping = 5.0f;
                
                Body* sphere_body = body_interface.CreateBody(sphere_body_settings);
                if (sphere_body != nullptr) {
                    BodyID sphere_id = sphere_body->GetID();
                    body_interface.AddBody(sphere_id, EActivation::Activate);
                    
                    // No initial velocity - cube will push the spheres
                    
                    ::Color sphereColor = {
                        (unsigned char)139,
                        (unsigned char)90,
                        (unsigned char)43,
                        255
                    }; // Brown dirt color
                    
                    Vector3 initialVelocity = { 0.0f, 0.0f, 0.0f };
                    dynamicSpheres.push_back({sphere_id, sphereColor, 0.0f, false, sphereSpawnPos, initialVelocity, sphereMass});
                }
            }
        }
        
        // Update density cellular automata
        densityGrid.Update(deltaTime, heightSamples, heightScale);
        
        // If too many spheres, force-convert the oldest stopped ones to terrain
        if ((int)dynamicSpheres.size() > maxSphereCount - 20) {
            // Find spheres that have been stopped for any amount of time and convert them
            int converted = 0;
            for (auto& sphere : dynamicSpheres) {
                if (sphere.markedForDestruction) continue;
                if (sphere.stoppedTimer > 0.1f && converted < 20) {
                    // Force convert to terrain
                    RVec3 pos = body_interface.GetPosition(sphere.bodyID);
                    float worldX = (float)pos.GetX();
                    float worldZ = (float)pos.GetZ();
                    
                    const float sphereVol = (4.0f / 3.0f) * 3.14159f * sphereRadius * sphereRadius * sphereRadius;
                    float densityAmt = sphereVol * 100.0f;
                    ModifyHeightmapWithDensity(densityGrid, worldX, worldZ, 
                                  2.0f, densityAmt, 0.15f, terrainScale, heightmapSize);
                    sphere.markedForDestruction = true;
                    converted++;
                }
            }
        }
        
        // Check for spheres touching terrain and track stopped time
        bool densityAdded = false;
        
        // Clear particle voxel grid for fresh update
        particleVoxelGrid.Clear();
        
        for (auto& sphere : dynamicSpheres) {
            if (sphere.markedForDestruction) continue;
            
            RVec3 position = body_interface.GetPosition(sphere.bodyID);
            Vec3 joltVel = body_interface.GetLinearVelocity(sphere.bodyID);
            float worldX = (float)position.GetX();
            float worldY = (float)position.GetY();
            float worldZ = (float)position.GetZ();
            Vector3 currentPos = { worldX, worldY, worldZ };
            
            // Update sphere velocity from physics engine
            sphere.velocity = { joltVel.GetX(), joltVel.GetY(), joltVel.GetZ() };
            
            // Add this sphere to the particle voxel grid
            particleVoxelGrid.AddParticle(worldX, worldY, worldZ, sphere.mass, sphere.velocity);
            
            // Check if sphere is touching or near the ground
            float terrainHeight = 0.0f;
            float hmX = (worldX + 10.0f) / (20.0f / heightmapSize);
            float hmZ = (worldZ + 10.0f) / (20.0f / heightmapSize);
            int ix = (int)hmX;
            int iz = (int)hmZ;
            
            if (ix >= 0 && ix < heightmapSize && iz >= 0 && iz < heightmapSize) {
                terrainHeight = heightSamples[iz * heightmapSize + ix];
            }
            
            // Check if sphere is on or near terrain
            bool onTerrain = (worldY - sphereRadius <= terrainHeight + 0.15f);
            
            // Check if sphere has stopped moving
            if (onTerrain && IsSphereStationary(currentPos, sphere.lastPosition, 0.02f)) {
                sphere.stoppedTimer += deltaTime;
            } else {
                sphere.stoppedTimer = 0.0f; // Reset timer if moving
            }
            
            // Update last position
            sphere.lastPosition = currentPos;
            
            // If sphere has been stopped long enough, convert to terrain
            if (sphere.stoppedTimer >= stoppedTimeThreshold) {
                // Add density to cellular automata grid
                const float sphereVolume = (4.0f / 3.0f) * 3.14159f * sphereRadius * sphereRadius * sphereRadius;
                
                // Convert sphere to terrain height
                float densityAmount = sphereVolume * 100.0f;
                ModifyHeightmapWithDensity(densityGrid, worldX, worldZ, 
                              2.0f, densityAmount, 0.15f, terrainScale, heightmapSize);
                densityAdded = true;
                sphere.markedForDestruction = true;
            }
            
            // Also destroy spheres that fall off the terrain
            if (worldY < -5.0f || worldX < -15.0f || worldX > 15.0f || 
                worldZ < -15.0f || worldZ > 15.0f) {
                sphere.markedForDestruction = true;
            }
        }
        
        // Calculate particle forces against the blade using voxel grid
        {
            float rotationRad;
            if (driveMode == 2) {
                rotationRad = manualRotation + guiBladeRotation * 3.14159f / 180.0f;
            } else if (driveMode == 1) {
                rotationRad = circleAngle + 3.14159f / 2.0f + guiBladeRotation * 3.14159f / 180.0f;
            } else {
                rotationRad = guiBladeRotation * 3.14159f / 180.0f;
            }
            
            lastParticleForceVec = particleVoxelGrid.CalculateBladeForce(
                cubePosition, cubeHalfX, cubeHalfY, cubeHalfZ,
                rotationRad, deltaTime);
            
            // Calculate magnitude of particle force
            lastParticleForce = sqrtf(lastParticleForceVec.x * lastParticleForceVec.x + 
                                      lastParticleForceVec.y * lastParticleForceVec.y + 
                                      lastParticleForceVec.z * lastParticleForceVec.z);
            
            // Add particle force to terrain resistance for speed calculation
            // This makes pushing particles slow down the blade
            terrainResistanceForce += lastParticleForce;
        }
        
        // Recreate physics body periodically to match CA updates (every few frames)
        static int frameCounter = 0;
        frameCounter++;
        bool shouldUpdatePhysics = densityAdded || (frameCounter % 10 == 0); // Update every 10 frames
        
        if (shouldUpdatePhysics) {
            // Remove old heightmap body
            body_interface.RemoveBody(heightmap_body->GetID());
            body_interface.DestroyBody(heightmap_body->GetID());
            
            // Create new heightfield shape with updated heights
            HeightFieldShapeSettings new_heightfield_settings(heightSamples.data(), Vec3(0, 0, 0), 
                Vec3(terrainScale, 1.0f, terrainScale), heightmapSize);
            ShapeSettings::ShapeResult new_heightfield_shape_result = new_heightfield_settings.Create();
            ShapeRefC new_heightfield_shape = new_heightfield_shape_result.Get();
            
            // Create new static body for heightmap
            BodyCreationSettings new_heightmap_body_settings(new_heightfield_shape, 
                RVec3(-10.0, 0.0, -10.0),
                Quat::sIdentity(), 
                EMotionType::Static, 
                Layers::NON_MOVING);
            new_heightmap_body_settings.mFriction = 5.0f; // High friction for terrain
            heightmap_body = body_interface.CreateBody(new_heightmap_body_settings);
            body_interface.AddBody(heightmap_body->GetID(), EActivation::DontActivate);
            
            // Update visual heightmap texture
            ::Color* newPixels = (::Color*)RL_MALLOC(heightmapSize * heightmapSize * sizeof(::Color));
            for (int y = 0; y < heightmapSize; y++) {
                for (int x = 0; x < heightmapSize; x++) {
                    unsigned char value = (unsigned char)((heightSamples[y * heightmapSize + x] / heightScale) * 255.0f);
                    newPixels[y * heightmapSize + x] = ::Color{ value, value, value, 255 };
                }
            }
            UpdateTexture(heightmapTexture, newPixels);
            RL_FREE(newPixels);
        }
        
        // Remove and destroy marked spheres
        for (auto it = dynamicSpheres.begin(); it != dynamicSpheres.end(); ) {
            if (it->markedForDestruction) {
                body_interface.RemoveBody(it->bodyID);
                body_interface.DestroyBody(it->bodyID);
                it = dynamicSpheres.erase(it);
            } else {
                ++it;
            }
        }

        // Rotate light direction around a circle
        float lightSpeed = 0.0f;
        lightAngle += lightSpeed * GetFrameTime();
        lightDirection.x = cosf(lightAngle);
        lightDirection.z = sinf(lightAngle);
        lightDirection.y = -0.5f;
        lightDirection = Vector3Normalize(lightDirection);
        SetShaderValue(shader, lightDirLoc, &lightDirection, SHADER_UNIFORM_VEC3);

        // Update camera position in shader
        SetShaderValue(shader, viewPosLoc, &camera.position, SHADER_UNIFORM_VEC3);
        
        // Update heightmap shader
        SetShaderValue(heightmapShader, hm_lightDirLoc, &lightDirection, SHADER_UNIFORM_VEC3);
        SetShaderValue(heightmapShader, hm_viewPosLoc, &camera.position, SHADER_UNIFORM_VEC3);

        BeginDrawing();
        ClearBackground(::GRAY);

        BeginMode3D(camera);
        //DrawGrid(20, 1.0f);
        
        // Draw the heightmap plane at the origin
        DrawModel(planeModel, Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, ::WHITE);
        
        // Draw all dynamic spheres
        for (const auto& sphere : dynamicSpheres) {
            RVec3 position = body_interface.GetPosition(sphere.bodyID);
            Vector3 spherePos = { (float)position.GetX(), (float)position.GetY(), (float)position.GetZ() };
            DrawModel(sphereModel, spherePos, 1.0f, sphere.color);
        }
        
        // Draw the moving cube with rotation from GUI
        float visualRotation;
        if (driveMode == 2) {
            visualRotation = manualRotation * 180.0f / 3.14159f + guiBladeRotation;
        } else if (driveMode == 1) {
            visualRotation = circleAngle * 180.0f / 3.14159f + 90.0f + guiBladeRotation;
        } else {
            visualRotation = guiBladeRotation;
        }
        DrawModelEx(cubeModel, cubePosition, Vector3{0.0f, 1.0f, 0.0f}, visualRotation, Vector3{1.0f, 1.0f, 1.0f}, ::BLUE);
        
        // Draw voxel debug visualization
        if (showVoxelDebug) {
            particleVoxelGrid.DrawDebug();
        }
        
        EndMode3D();

        DrawText("Right-click to drop 10 spheres from 5m high", 10, 10, 20, ::DARKGRAY);
        DrawText(TextFormat("Sphere Count: %d", (int)dynamicSpheres.size()), 10, 35, 16, ::DARKGRAY);
        DrawText(TextFormat("Sphere Radius: %.2fm", sphereRadius), 10, 55, 16, ::DARKGRAY);
        float speedPercent = (guiCubeSpeed > 0) ? (cubeCurrentSpeed / guiCubeSpeed * 100.0f) : 0.0f;
        DrawText(TextFormat("Speed: %.2f / %.2f m/s (%.0f%%)", cubeCurrentSpeed, guiCubeSpeed, speedPercent), 10, 75, 16, 
                 speedPercent < 50.0f ? ::RED : ::DARKGRAY);
        float totalForce = lastTerrainForce + lastParticleForce;
        DrawText(TextFormat("Total Force: %.0f N (Terrain: %.0f + Particle: %.0f)", totalForce, lastTerrainForce, lastParticleForce), 
                 10, 95, 16, totalForce > guiEnginePower ? ::RED : ::DARKGRAY);
        DrawText(TextFormat("Engine: %.0f N | Voxels: %d", guiEnginePower, particleVoxelGrid.GetTotalParticleCount()), 
                 10, 115, 16, ::DARKGRAY);
        DrawFPS(10, 135);
        
        // Toggle GUI with G key, toggle voxel debug with V key
        if (IsKeyPressed(KEY_G)) showGui = !showGui;
        if (IsKeyPressed(KEY_V)) showVoxelDebug = !showVoxelDebug;
        
        // Draw GUI panel for cube settings
        if (showGui) {
            int panelX = 10;
            int panelY = 160;
            int panelW = 220;
            int panelH = (driveMode == 1) ? 335 : ((driveMode == 2) ? 200 : 275);
            
            DrawRectangle(panelX, panelY, panelW, panelH, Fade(::LIGHTGRAY, 0.9f));
            DrawRectangleLines(panelX, panelY, panelW, panelH, ::DARKGRAY);
            DrawText("Cube Settings", panelX + 5, panelY + 5, 10, ::DARKGRAY);
            
            Vector2 mousePos = GetMousePosition();
            bool mouseDown = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
            bool mousePressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
            bool mouseReleased = IsMouseButtonReleased(MOUSE_BUTTON_LEFT);
            
            // Mode toggle button
            {
                int btnX = panelX + 5;
                int btnY = panelY + 22;
                int btnW = 100;
                int btnH = 18;
                Rectangle btnRect = { (float)btnX, (float)btnY, (float)btnW, (float)btnH };
                
                ::Color btnColor = (driveMode == 0) ? ::MAROON : ((driveMode == 1) ? ::GREEN : ::ORANGE);
                const char* modeText = (driveMode == 0) ? "Linear" : ((driveMode == 1) ? "Circle" : "Manual");
                DrawRectangle(btnX, btnY, btnW, btnH, btnColor);
                DrawRectangleLines(btnX, btnY, btnW, btnH, ::DARKGRAY);
                DrawText(modeText, btnX + 25, btnY + 4, 10, ::WHITE);
                
                if (CheckCollisionPointRec(mousePos, btnRect) && mousePressed) {
                    driveMode = (driveMode + 1) % 3; // Cycle through 0, 1, 2
                    if (driveMode == 1) {
                        // Initialize circle position
                        circleAngle = 0.0f;
                        cubePosition.x = guiCircleCenterX + guiCircleRadius;
                        cubePosition.z = guiCircleCenterZ;
                    } else if (driveMode == 2) {
                        // Initialize manual mode
                        manualVelocity = { 0.0f, 0.0f, 0.0f };
                        manualRotation = 0.0f;
                    }
                }
            }
            
            int sliderStartY = panelY + 45;
            
            if (driveMode == 0) {
                // Linear mode sliders
                // Slider 1: Start X
            {
                int sliderX = panelX + 70;
                int sliderY = sliderStartY;
                int sliderW = 120;
                int sliderH = 16;
                float minVal = -12.0f, maxVal = 12.0f;
                
                DrawText("Start X:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                
                float normalized = (guiCubeStartX - minVal) / (maxVal - minVal);
                int handleX = sliderX + (int)(normalized * (sliderW - 10));
                DrawRectangle(handleX, sliderY, 10, sliderH, ::BLUE);
                
                Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                if (CheckCollisionPointRec(mousePos, sliderRect)) {
                    if (mousePressed) activeSlider = 0;
                }
                if (activeSlider == 0 && mouseDown) {
                    float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                    newNorm = Clamp(newNorm, 0.0f, 1.0f);
                    guiCubeStartX = minVal + newNorm * (maxVal - minVal);
                }
                DrawText(TextFormat("%.1f", guiCubeStartX), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
            }
            
            // Slider 2: Start Z
            {
                int sliderX = panelX + 70;
                int sliderY = sliderStartY + 25;
                int sliderW = 120;
                int sliderH = 16;
                float minVal = -10.0f, maxVal = 10.0f;
                
                DrawText("Start Z:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                
                float normalized = (guiCubeStartZ - minVal) / (maxVal - minVal);
                int handleX = sliderX + (int)(normalized * (sliderW - 10));
                DrawRectangle(handleX, sliderY, 10, sliderH, ::BLUE);
                
                Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                if (CheckCollisionPointRec(mousePos, sliderRect)) {
                    if (mousePressed) activeSlider = 1;
                }
                if (activeSlider == 1 && mouseDown) {
                    float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                    newNorm = Clamp(newNorm, 0.0f, 1.0f);
                    guiCubeStartZ = minVal + newNorm * (maxVal - minVal);
                }
                DrawText(TextFormat("%.1f", guiCubeStartZ), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
            }
            
            // Slider 3: Speed
            {
                int sliderX = panelX + 70;
                int sliderY = sliderStartY + 50;
                int sliderW = 120;
                int sliderH = 16;
                float minVal = 0.5f, maxVal = 10.0f;
                
                DrawText("Speed:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                
                float normalized = (guiCubeSpeed - minVal) / (maxVal - minVal);
                int handleX = sliderX + (int)(normalized * (sliderW - 10));
                DrawRectangle(handleX, sliderY, 10, sliderH, ::BLUE);
                
                Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                if (CheckCollisionPointRec(mousePos, sliderRect)) {
                    if (mousePressed) activeSlider = 2;
                }
                if (activeSlider == 2 && mouseDown) {
                    float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                    newNorm = Clamp(newNorm, 0.0f, 1.0f);
                    guiCubeSpeed = minVal + newNorm * (maxVal - minVal);
                }
                DrawText(TextFormat("%.1f", guiCubeSpeed), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
            }
            
            // Slider 4: Blade Rotation
            {
                int sliderX = panelX + 70;
                int sliderY = sliderStartY + 75;
                int sliderW = 120;
                int sliderH = 16;
                float minVal = 0.0f, maxVal = 90.0f;
                
                DrawText("Rotation:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                
                float normalized = (guiBladeRotation - minVal) / (maxVal - minVal);
                int handleX = sliderX + (int)(normalized * (sliderW - 10));
                DrawRectangle(handleX, sliderY, 10, sliderH, ::BLUE);
                
                Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                if (CheckCollisionPointRec(mousePos, sliderRect)) {
                    if (mousePressed) activeSlider = 10;
                }
                if (activeSlider == 10 && mouseDown) {
                    float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                    newNorm = Clamp(newNorm, 0.0f, 1.0f);
                    guiBladeRotation = minVal + newNorm * (maxVal - minVal);
                }
                DrawText(TextFormat("%.0f°", guiBladeRotation), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
            }
            
            // Slider 5: Height
            {
                int sliderX = panelX + 70;
                int sliderY = sliderStartY + 100;
                int sliderW = 120;
                int sliderH = 16;
                float minVal = -2.0f, maxVal = 5.0f;
                
                DrawText("Height:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                
                float normalized = (guiCubeHeight - minVal) / (maxVal - minVal);
                int handleX = sliderX + (int)(normalized * (sliderW - 10));
                DrawRectangle(handleX, sliderY, 10, sliderH, ::BLUE);
                
                Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                if (CheckCollisionPointRec(mousePos, sliderRect)) {
                    if (mousePressed) activeSlider = 11;
                }
                if (activeSlider == 11 && mouseDown) {
                    float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                    newNorm = Clamp(newNorm, 0.0f, 1.0f);
                    guiCubeHeight = minVal + newNorm * (maxVal - minVal);
                }
                DrawText(TextFormat("%.1f", guiCubeHeight), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
            }
            
            // Slider 6: Engine Power
            {
                int sliderX = panelX + 70;
                int sliderY = sliderStartY + 125;
                int sliderW = 120;
                int sliderH = 16;
                float minVal = 100.0f, maxVal = 2000.0f;
                
                DrawText("Engine:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                
                float normalized = (guiEnginePower - minVal) / (maxVal - minVal);
                int handleX = sliderX + (int)(normalized * (sliderW - 10));
                DrawRectangle(handleX, sliderY, 10, sliderH, ::ORANGE);
                
                Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                if (CheckCollisionPointRec(mousePos, sliderRect)) {
                    if (mousePressed) activeSlider = 12;
                }
                if (activeSlider == 12 && mouseDown) {
                    float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                    newNorm = Clamp(newNorm, 0.0f, 1.0f);
                    guiEnginePower = minVal + newNorm * (maxVal - minVal);
                }
                DrawText(TextFormat("%.0fN", guiEnginePower), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
            }
            
            // Slider 7: Resistance Coefficient
            {
                int sliderX = panelX + 70;
                int sliderY = sliderStartY + 150;
                int sliderW = 120;
                int sliderH = 16;
                float minVal = 10.0f, maxVal = 200.0f;
                
                DrawText("Resist:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                
                float normalized = (guiResistanceCoeff - minVal) / (maxVal - minVal);
                int handleX = sliderX + (int)(normalized * (sliderW - 10));
                DrawRectangle(handleX, sliderY, 10, sliderH, ::MAROON);
                
                Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                if (CheckCollisionPointRec(mousePos, sliderRect)) {
                    if (mousePressed) activeSlider = 13;
                }
                if (activeSlider == 13 && mouseDown) {
                    float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                    newNorm = Clamp(newNorm, 0.0f, 1.0f);
                    guiResistanceCoeff = minVal + newNorm * (maxVal - minVal);
                }
                DrawText(TextFormat("%.0f", guiResistanceCoeff), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
            }
            } else if (driveMode == 1) {
                // Circle mode sliders
                // Center X
                {
                    int sliderX = panelX + 70;
                    int sliderY = sliderStartY;
                    int sliderW = 120;
                    int sliderH = 16;
                    float minVal = -8.0f, maxVal = 8.0f;
                    
                    DrawText("Center X:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                    DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                    
                    float normalized = (guiCircleCenterX - minVal) / (maxVal - minVal);
                    int handleX = sliderX + (int)(normalized * (sliderW - 10));
                    DrawRectangle(handleX, sliderY, 10, sliderH, ::GREEN);
                    
                    Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                    if (CheckCollisionPointRec(mousePos, sliderRect)) {
                        if (mousePressed) activeSlider = 3;
                    }
                    if (activeSlider == 3 && mouseDown) {
                        float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                        newNorm = Clamp(newNorm, 0.0f, 1.0f);
                        guiCircleCenterX = minVal + newNorm * (maxVal - minVal);
                    }
                    DrawText(TextFormat("%.1f", guiCircleCenterX), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
                }
                
                // Center Z
                {
                    int sliderX = panelX + 70;
                    int sliderY = sliderStartY + 25;
                    int sliderW = 120;
                    int sliderH = 16;
                    float minVal = -8.0f, maxVal = 8.0f;
                    
                    DrawText("Center Z:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                    DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                    
                    float normalized = (guiCircleCenterZ - minVal) / (maxVal - minVal);
                    int handleX = sliderX + (int)(normalized * (sliderW - 10));
                    DrawRectangle(handleX, sliderY, 10, sliderH, ::GREEN);
                    
                    Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                    if (CheckCollisionPointRec(mousePos, sliderRect)) {
                        if (mousePressed) activeSlider = 4;
                    }
                    if (activeSlider == 4 && mouseDown) {
                        float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                        newNorm = Clamp(newNorm, 0.0f, 1.0f);
                        guiCircleCenterZ = minVal + newNorm * (maxVal - minVal);
                    }
                    DrawText(TextFormat("%.1f", guiCircleCenterZ), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
                }
                
                // Radius
                {
                    int sliderX = panelX + 70;
                    int sliderY = sliderStartY + 50;
                    int sliderW = 120;
                    int sliderH = 16;
                    float minVal = 1.0f, maxVal = 9.0f;
                    
                    DrawText("Radius:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                    DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                    
                    float normalized = (guiCircleRadius - minVal) / (maxVal - minVal);
                    int handleX = sliderX + (int)(normalized * (sliderW - 10));
                    DrawRectangle(handleX, sliderY, 10, sliderH, ::GREEN);
                    
                    Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                    if (CheckCollisionPointRec(mousePos, sliderRect)) {
                        if (mousePressed) activeSlider = 5;
                    }
                    if (activeSlider == 5 && mouseDown) {
                        float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                        newNorm = Clamp(newNorm, 0.0f, 1.0f);
                        guiCircleRadius = minVal + newNorm * (maxVal - minVal);
                    }
                    DrawText(TextFormat("%.1f", guiCircleRadius), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
                }
                
                // Speed
                {
                    int sliderX = panelX + 70;
                    int sliderY = sliderStartY + 75;
                    int sliderW = 120;
                    int sliderH = 16;
                    float minVal = 0.5f, maxVal = 10.0f;
                    
                    DrawText("Speed:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                    DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                    
                    float normalized = (guiCubeSpeed - minVal) / (maxVal - minVal);
                    int handleX = sliderX + (int)(normalized * (sliderW - 10));
                    DrawRectangle(handleX, sliderY, 10, sliderH, ::GREEN);
                    
                    Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                    if (CheckCollisionPointRec(mousePos, sliderRect)) {
                        if (mousePressed) activeSlider = 6;
                    }
                    if (activeSlider == 6 && mouseDown) {
                        float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                        newNorm = Clamp(newNorm, 0.0f, 1.0f);
                        guiCubeSpeed = minVal + newNorm * (maxVal - minVal);
                    }
                    DrawText(TextFormat("%.1f", guiCubeSpeed), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
                }
                
                // Blade Rotation
                {
                    int sliderX = panelX + 70;
                    int sliderY = sliderStartY + 100;
                    int sliderW = 120;
                    int sliderH = 16;
                    float minVal = 0.0f, maxVal = 90.0f;
                    
                    DrawText("Rotation:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                    DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                    
                    float normalized = (guiBladeRotation - minVal) / (maxVal - minVal);
                    int handleX = sliderX + (int)(normalized * (sliderW - 10));
                    DrawRectangle(handleX, sliderY, 10, sliderH, ::GREEN);
                    
                    Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                    if (CheckCollisionPointRec(mousePos, sliderRect)) {
                        if (mousePressed) activeSlider = 7;
                    }
                    if (activeSlider == 7 && mouseDown) {
                        float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                        newNorm = Clamp(newNorm, 0.0f, 1.0f);
                        guiBladeRotation = minVal + newNorm * (maxVal - minVal);
                    }
                    DrawText(TextFormat("%.0f°", guiBladeRotation), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
                }
                
                // Height
                {
                    int sliderX = panelX + 70;
                    int sliderY = sliderStartY + 125;
                    int sliderW = 120;
                    int sliderH = 16;
                    float minVal = -2.0f, maxVal = 5.0f;
                    
                    DrawText("Height:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                    DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                    
                    float normalized = (guiCubeHeight - minVal) / (maxVal - minVal);
                    int handleX = sliderX + (int)(normalized * (sliderW - 10));
                    DrawRectangle(handleX, sliderY, 10, sliderH, ::GREEN);
                    
                    Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                    if (CheckCollisionPointRec(mousePos, sliderRect)) {
                        if (mousePressed) activeSlider = 8;
                    }
                    if (activeSlider == 8 && mouseDown) {
                        float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                        newNorm = Clamp(newNorm, 0.0f, 1.0f);
                        guiCubeHeight = minVal + newNorm * (maxVal - minVal);
                    }
                    DrawText(TextFormat("%.1f", guiCubeHeight), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
                }
                
                // Engine Power (circle mode)
                {
                    int sliderX = panelX + 70;
                    int sliderY = sliderStartY + 150;
                    int sliderW = 120;
                    int sliderH = 16;
                    float minVal = 100.0f, maxVal = 2000.0f;
                    
                    DrawText("Engine:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                    DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                    
                    float normalized = (guiEnginePower - minVal) / (maxVal - minVal);
                    int handleX = sliderX + (int)(normalized * (sliderW - 10));
                    DrawRectangle(handleX, sliderY, 10, sliderH, ::ORANGE);
                    
                    Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                    if (CheckCollisionPointRec(mousePos, sliderRect)) {
                        if (mousePressed) activeSlider = 14;
                    }
                    if (activeSlider == 14 && mouseDown) {
                        float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                        newNorm = Clamp(newNorm, 0.0f, 1.0f);
                        guiEnginePower = minVal + newNorm * (maxVal - minVal);
                    }
                    DrawText(TextFormat("%.0fN", guiEnginePower), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
                }
                
                // Resistance Coefficient (circle mode)
                {
                    int sliderX = panelX + 70;
                    int sliderY = sliderStartY + 175;
                    int sliderW = 120;
                    int sliderH = 16;
                    float minVal = 10.0f, maxVal = 200.0f;
                    
                    DrawText("Resist:", panelX + 5, sliderY + 2, 10, ::DARKGRAY);
                    DrawRectangle(sliderX, sliderY, sliderW, sliderH, ::DARKGRAY);
                    
                    float normalized = (guiResistanceCoeff - minVal) / (maxVal - minVal);
                    int handleX = sliderX + (int)(normalized * (sliderW - 10));
                    DrawRectangle(handleX, sliderY, 10, sliderH, ::MAROON);
                    
                    Rectangle sliderRect = { (float)sliderX, (float)sliderY, (float)sliderW, (float)sliderH };
                    if (CheckCollisionPointRec(mousePos, sliderRect)) {
                        if (mousePressed) activeSlider = 15;
                    }
                    if (activeSlider == 15 && mouseDown) {
                        float newNorm = (mousePos.x - sliderX) / (float)sliderW;
                        newNorm = Clamp(newNorm, 0.0f, 1.0f);
                        guiResistanceCoeff = minVal + newNorm * (maxVal - minVal);
                    }
                    DrawText(TextFormat("%.0f", guiResistanceCoeff), sliderX + sliderW + 5, sliderY + 2, 10, ::BLACK);
                }
            } else {
                // Manual mode - show controls info
                DrawText("Arrow Keys:", panelX + 5, sliderStartY, 10, ::DARKGRAY);
                DrawText("UP = Forward", panelX + 10, sliderStartY + 15, 10, ::ORANGE);
                DrawText("DOWN = Reverse", panelX + 10, sliderStartY + 30, 10, ::ORANGE);
                DrawText("LEFT = Turn Left", panelX + 10, sliderStartY + 45, 10, ::ORANGE);
                DrawText("RIGHT = Turn Right", panelX + 10, sliderStartY + 60, 10, ::ORANGE);
                
                // Speed display (current)
                float manualSpeed = sqrtf(manualVelocity.x * manualVelocity.x + manualVelocity.z * manualVelocity.z);
                DrawText(TextFormat("Speed: %.2f m/s", manualSpeed), panelX + 5, sliderStartY + 80, 10, 
                         manualSpeed > 0.1f ? ::BLUE : ::GRAY);
            }
            
            if (mouseReleased) activeSlider = -1;
            
            // Current position display
            int footerY = (driveMode == 1) ? panelY + 270 : ((driveMode == 2) ? panelY + 135 : panelY + 210);
            DrawText(TextFormat("Current: (%.1f, %.1f, %.1f)", cubePosition.x, cubePosition.y, cubePosition.z), 
                     panelX + 5, footerY, 10, ::BLUE);
            
            DrawText("Press G to toggle GUI", panelX + 5, footerY + 20, 10, ::GRAY);
            DrawText(TextFormat("Press V to toggle voxels %s", showVoxelDebug ? "(ON)" : "(OFF)"), panelX + 5, footerY + 33, 10, 
                     showVoxelDebug ? ::GREEN : ::GRAY);
            const char* modeInfo = (driveMode == 0) ? "Linear mode (resets at edge)" : 
                                   ((driveMode == 1) ? "Circle mode active" : "Manual: Arrows to drive");
            DrawText(modeInfo, panelX + 5, footerY + 46, 10, ::GRAY);
        }

        EndDrawing();
    }

    // Cleanup physics
    for (const auto& sphere : dynamicSpheres) {
        body_interface.RemoveBody(sphere.bodyID);
        body_interface.DestroyBody(sphere.bodyID);
    }
    body_interface.RemoveBody(cube_body_id);
    body_interface.DestroyBody(cube_body_id);
    body_interface.RemoveBody(heightmap_body->GetID());
    body_interface.DestroyBody(heightmap_body->GetID());

    // Unload shader and models
    UnloadShader(shader);
    UnloadShader(heightmapShader);
    UnloadTexture(heightmapTexture);
    UnloadModel(sphereModel);
    UnloadModel(cubeModel);
    UnloadModel(planeModel);

    CloseWindow();

    return 0;
}
