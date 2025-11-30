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
    const uint cMaxBodies = 1024;
    const uint cNumBodyMutexes = 0;
    const uint cMaxBodyPairs = 1024;
    const uint cMaxContactConstraints = 1024;

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
    Model sphereModel = LoadModelFromMesh(GenMeshSphere(sphereRadius, 32, 32));
    sphereModel.materials[0].shader = shader;
    
    // Create a cube model for the moving cube
    float cubeSize = 1.0f;
    Model cubeModel = LoadModelFromMesh(GenMeshCube(cubeSize * 2.0f, cubeSize, cubeSize * 0.1f));
    cubeModel.materials[0].shader = shader;
    
    // Create physics body for the cube (kinematic so it can push spheres)
    BoxShapeSettings cube_shape_settings(Vec3(cubeSize, cubeSize * 0.5f, cubeSize * 0.1f));
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
    float cubeMinX = -12.0f;
    float cubeMaxX = 12.0f;
    float cubeDigDepth = 0.3f; // How deep the cube digs into terrain
    float sphereSpawnAccumulator = 0.0f; // Accumulate displaced volume for spawning spheres
    const float stoppedTimeThreshold = 0.5f; // Time in seconds before sphere converts to earth
    
    // GUI settings for cube (applied on reset)
    float guiCubeStartX = -12.0f;
    float guiCubeStartZ = 0.0f;
    float guiCubeSpeed = 3.0f;
    bool showGui = true;
    int activeSlider = -1; // Track which slider is being dragged

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
                
                dynamicSpheres.push_back({sphere_id, sphereColor, 0.0f, false, spawnPosition});
            }
        }

        // Update physics (60 Hz simulation)
        const float deltaTime = GetFrameTime();
        const int collisionSteps = 1;
        physics_system.Update(deltaTime, collisionSteps, &temp_allocator, &job_system);
        
        // Update moving cube position (left to right)
        cubePosition.x += cubeSpeed * deltaTime;
        if (cubePosition.x > cubeMaxX) {
            // Reset cube with GUI values
            cubePosition.x = guiCubeStartX;
            cubePosition.z = guiCubeStartZ;
            cubeSpeed = guiCubeSpeed;
            cubeMinX = guiCubeStartX;
        }
        
        // Sample terrain height at cube position to make it follow terrain
        float cubeHmX = (cubePosition.x + 10.0f) / (20.0f / heightmapSize);
        float cubeHmZ = (cubePosition.z + 10.0f) / (20.0f / heightmapSize);
        int cubeIx = (int)cubeHmX;
        int cubeIz = (int)cubeHmZ;
        if (cubeIx >= 0 && cubeIx < heightmapSize && cubeIz >= 0 && cubeIz < heightmapSize) {
            float terrainHeightAtCube = heightSamples[cubeIz * heightmapSize + cubeIx];
            cubePosition.y = terrainHeightAtCube + cubeSize * 0.5f; // Place cube on top of terrain
        }
        
        // Update cube physics body position and rotation (kinematic body)
        body_interface.SetPositionAndRotation(cube_body_id, 
            RVec3(cubePosition.x, cubePosition.y, cubePosition.z), 
            cubeRotation, 
            EActivation::Activate);
        // Set velocity so physics engine knows it's moving (helps with collision response)
        body_interface.SetLinearVelocity(cube_body_id, Vec3(cubeSpeed, 0.0f, 0.0f));
        
        // Cube terrain deformation - dig into terrain and spawn spheres
        // Only deform if cube is within terrain bounds
        if (cubePosition.x >= -10.0f && cubePosition.x <= 10.0f &&
            cubePosition.z >= -10.0f && cubePosition.z <= 10.0f) {
            
            // Calculate cube footprint in heightmap coordinates
            float cubeHalfSize = cubeSize * 0.5f;
            int hmMinX = (int)((cubePosition.x - cubeHalfSize + 10.0f) / (20.0f / heightmapSize));
            int hmMaxX = (int)((cubePosition.x + cubeHalfSize + 10.0f) / (20.0f / heightmapSize));
            int hmMinZ = (int)((cubePosition.z - cubeHalfSize + 10.0f) / (20.0f / heightmapSize));
            int hmMaxZ = (int)((cubePosition.z + cubeHalfSize + 10.0f) / (20.0f / heightmapSize));
            
            float totalDisplacedVolume = 0.0f;
            
            // Dig terrain under the cube
            for (int z = hmMinZ; z <= hmMaxZ; z++) {
                for (int x = hmMinX; x <= hmMaxX; x++) {
                    if (x >= 0 && x < heightmapSize && z >= 0 && z < heightmapSize) {
                        int idx = z * heightmapSize + x;
                        float currentHeight = heightSamples[idx];
                        float digAmount = cubeDigDepth * deltaTime * 5.0f; // Scale with frame time
                        
                        if (currentHeight > 0.0f) {
                            float actualDig = fminf(digAmount, currentHeight);
                            heightSamples[idx] -= actualDig;
                            if (heightSamples[idx] < 0.0f) heightSamples[idx] = 0.0f;
                            
                            // Calculate displaced volume (per cell)
                            float cellArea = terrainScale * terrainScale;
                            totalDisplacedVolume += actualDig * cellArea;
                        }
                    }
                }
            }
            
            // Accumulate displaced volume and spawn spheres
            sphereSpawnAccumulator += totalDisplacedVolume;
            const float sphereVolume = (4.0f / 3.0f) * 3.14159f * sphereRadius * sphereRadius * sphereRadius;
            
            while (sphereSpawnAccumulator >= sphereVolume) {
                sphereSpawnAccumulator -= sphereVolume;
                
                // Spawn sphere IN FRONT of the cube so it gets pushed
                float spawnOffsetX = cubeHalfSize + sphereRadius + 0.15f; // In front of cube
                float spawnOffsetZ = ((float)GetRandomValue(-50, 50) / 100.0f) * cubeHalfSize;
                float spawnY = cubePosition.y - cubeHalfSize + sphereRadius;
                
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
                    
                    dynamicSpheres.push_back({sphere_id, sphereColor, 0.0f, false, sphereSpawnPos});
                }
            }
        }
        
        // Update density cellular automata
        densityGrid.Update(deltaTime, heightSamples, heightScale);
        
        // Check for spheres touching terrain and track stopped time
        bool densityAdded = false;
        
        for (auto& sphere : dynamicSpheres) {
            if (sphere.markedForDestruction) continue;
            
            RVec3 position = body_interface.GetPosition(sphere.bodyID);
            float worldX = (float)position.GetX();
            float worldY = (float)position.GetY();
            float worldZ = (float)position.GetZ();
            Vector3 currentPos = { worldX, worldY, worldZ };
            
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
        DrawGrid(20, 1.0f);
        
        // Draw the heightmap plane at the origin
        DrawModel(planeModel, Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, ::WHITE);
        
        // Draw all dynamic spheres
        for (const auto& sphere : dynamicSpheres) {
            RVec3 position = body_interface.GetPosition(sphere.bodyID);
            Vector3 spherePos = { (float)position.GetX(), (float)position.GetY(), (float)position.GetZ() };
            DrawModel(sphereModel, spherePos, 1.0f, sphere.color);
        }
        
        // Draw the moving cube with 45 degree rotation
        DrawModelEx(cubeModel, cubePosition, Vector3{0.0f, 1.0f, 0.0f}, 45.0f, Vector3{1.0f, 1.0f, 1.0f}, ::BLUE);
        
        EndMode3D();

        DrawText("Right-click to drop 10 spheres from 5m high", 10, 10, 20, ::DARKGRAY);
        DrawText(TextFormat("Sphere Count: %d", (int)dynamicSpheres.size()), 10, 35, 16, ::DARKGRAY);
        DrawText(TextFormat("Sphere Radius: %.2fm", sphereRadius), 10, 55, 16, ::DARKGRAY);
        DrawFPS(10, 75);
        
        // Toggle GUI with G key
        if (IsKeyPressed(KEY_G)) showGui = !showGui;
        
        // Draw GUI panel for cube settings
        if (showGui) {
            int panelX = screenWidth - 230;
            int panelY = 10;
            int panelW = 220;
            int panelH = 160;
            
            DrawRectangle(panelX, panelY, panelW, panelH, Fade(::LIGHTGRAY, 0.9f));
            DrawRectangleLines(panelX, panelY, panelW, panelH, ::DARKGRAY);
            DrawText("Cube Settings (next reset)", panelX + 5, panelY + 5, 10, ::DARKGRAY);
            
            Vector2 mousePos = GetMousePosition();
            bool mouseDown = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
            bool mousePressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
            bool mouseReleased = IsMouseButtonReleased(MOUSE_BUTTON_LEFT);
            
            // Custom slider helper lambda-like logic
            // Slider 1: Start X
            {
                int sliderX = panelX + 70;
                int sliderY = panelY + 30;
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
                int sliderY = panelY + 55;
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
                int sliderY = panelY + 80;
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
            
            if (mouseReleased) activeSlider = -1;
            
            // Current position display
            DrawText(TextFormat("Current: (%.1f, %.1f, %.1f)", cubePosition.x, cubePosition.y, cubePosition.z), 
                     panelX + 5, panelY + 110, 10, ::BLUE);
            
            DrawText("Press G to toggle GUI", panelX + 5, panelY + 130, 10, ::GRAY);
            DrawText("Settings apply on next reset", panelX + 5, panelY + 143, 10, ::GRAY);
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
