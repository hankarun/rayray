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
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

// Custom includes
#include "PhysicsLayers.h"
#include "PhysicsStructures.h"
#include "DensityAutomata.h"
#include "ParticleVoxelGrid.h"
#include "PerlinNoise.h"
#include "Helpers.h"
#include "Blade.h"
#include "ExcavatorBucket.h"

// Jolt namespace
using namespace JPH;

// Bucket scoop state machine
enum class ScoopState {
    IDLE,           // Waiting at start position
    MOVING_TO_DIG,  // Moving towards dig position
    DIGGING,        // Lowering and scooping terrain
    LIFTING,        // Lifting bucket with material
    MOVING_TO_DROP, // Moving towards drop position
    DROPPING,       // Tilting bucket to drop material
    RETURNING       // Returning to start position
};

const char* GetStateName(ScoopState state) {
    switch (state) {
        case ScoopState::IDLE: return "IDLE";
        case ScoopState::MOVING_TO_DIG: return "MOVING TO DIG";
        case ScoopState::DIGGING: return "DIGGING";
        case ScoopState::LIFTING: return "LIFTING";
        case ScoopState::MOVING_TO_DROP: return "MOVING TO DROP";
        case ScoopState::DROPPING: return "DROPPING";
        case ScoopState::RETURNING: return "RETURNING";
        default: return "UNKNOWN";
    }
}

// Helper to create a bucket/scoop compound shape from multiple boxes
ShapeRefC CreateBucketShape(float bucketWidth, float bucketDepth, float bucketHeight, float wallThickness) {
    StaticCompoundShapeSettings compound_settings;
    
    // Use a small convex radius that's less than the smallest half-extent (wallThickness/2)
    float convexRadius = wallThickness * 0.25f;  // 25% of wall thickness
    
    // Bottom plate
    BoxShapeSettings bottom_settings(Vec3(bucketWidth / 2.0f, wallThickness / 2.0f, bucketDepth / 2.0f), convexRadius);
    ShapeRefC bottom_shape = bottom_settings.Create().Get();
    compound_settings.AddShape(Vec3(0, wallThickness / 2.0f, 0), Quat::sIdentity(), bottom_shape);
    
    // Left wall
    BoxShapeSettings left_settings(Vec3(wallThickness / 2.0f, bucketHeight / 2.0f, bucketDepth / 2.0f), convexRadius);
    ShapeRefC left_shape = left_settings.Create().Get();
    compound_settings.AddShape(Vec3(-bucketWidth / 2.0f - wallThickness / 2.0f, bucketHeight / 2.0f + wallThickness, 0), 
                               Quat::sIdentity(), left_shape);
    
    // Right wall
    BoxShapeSettings right_settings(Vec3(wallThickness / 2.0f, bucketHeight / 2.0f, bucketDepth / 2.0f), convexRadius);
    ShapeRefC right_shape = right_settings.Create().Get();
    compound_settings.AddShape(Vec3(bucketWidth / 2.0f + wallThickness / 2.0f, bucketHeight / 2.0f + wallThickness, 0), 
                               Quat::sIdentity(), right_shape);
    
    // Back wall
    BoxShapeSettings back_settings(Vec3(bucketWidth / 2.0f + wallThickness, bucketHeight / 2.0f, wallThickness / 2.0f), convexRadius);
    ShapeRefC back_shape = back_settings.Create().Get();
    compound_settings.AddShape(Vec3(0, bucketHeight / 2.0f + wallThickness, -bucketDepth / 2.0f - wallThickness / 2.0f), 
                               Quat::sIdentity(), back_shape);
    
    // Front lip
    BoxShapeSettings lip_settings(Vec3(bucketWidth / 2.0f + wallThickness, wallThickness / 2.0f, wallThickness / 2.0f), convexRadius);
    ShapeRefC lip_shape = lip_settings.Create().Get();
    compound_settings.AddShape(Vec3(0, wallThickness / 2.0f, bucketDepth / 2.0f + wallThickness / 2.0f), 
                               Quat::sIdentity(), lip_shape);
    
    return compound_settings.Create().Get();
}

// Helper to create raylib mesh for bucket visualization
Model CreateBucketModel(float bucketWidth, float bucketDepth, float bucketHeight, float wallThickness, Shader shader) {
    Mesh bucketMesh = { 0 };
    int triangleCount = 5 * 12;
    
    bucketMesh.vertexCount = triangleCount * 3;
    bucketMesh.triangleCount = triangleCount;
    bucketMesh.vertices = (float*)RL_MALLOC(bucketMesh.vertexCount * 3 * sizeof(float));
    bucketMesh.normals = (float*)RL_MALLOC(bucketMesh.vertexCount * 3 * sizeof(float));
    
    int vi = 0;
    
    auto addBox = [&](float cx, float cy, float cz, float hw, float hh, float hd) {
        float vertices[8][3] = {
            {cx - hw, cy - hh, cz + hd}, {cx + hw, cy - hh, cz + hd},
            {cx + hw, cy + hh, cz + hd}, {cx - hw, cy + hh, cz + hd},
            {cx - hw, cy - hh, cz - hd}, {cx + hw, cy - hh, cz - hd},
            {cx + hw, cy + hh, cz - hd}, {cx - hw, cy + hh, cz - hd}
        };
        int faces[6][4] = {{0,1,2,3}, {5,4,7,6}, {4,0,3,7}, {1,5,6,2}, {3,2,6,7}, {4,5,1,0}};
        float normals[6][3] = {{0,0,1}, {0,0,-1}, {-1,0,0}, {1,0,0}, {0,1,0}, {0,-1,0}};
        
        for (int f = 0; f < 6; f++) {
            for (int i = 0; i < 3; i++) {
                int idx = (i == 0) ? 0 : (i == 1) ? 1 : 2;
                bucketMesh.vertices[vi*3] = vertices[faces[f][idx]][0];
                bucketMesh.vertices[vi*3+1] = vertices[faces[f][idx]][1];
                bucketMesh.vertices[vi*3+2] = vertices[faces[f][idx]][2];
                bucketMesh.normals[vi*3] = normals[f][0];
                bucketMesh.normals[vi*3+1] = normals[f][1];
                bucketMesh.normals[vi*3+2] = normals[f][2];
                vi++;
            }
            for (int i = 0; i < 3; i++) {
                int idx = (i == 0) ? 0 : (i == 1) ? 2 : 3;
                bucketMesh.vertices[vi*3] = vertices[faces[f][idx]][0];
                bucketMesh.vertices[vi*3+1] = vertices[faces[f][idx]][1];
                bucketMesh.vertices[vi*3+2] = vertices[faces[f][idx]][2];
                bucketMesh.normals[vi*3] = normals[f][0];
                bucketMesh.normals[vi*3+1] = normals[f][1];
                bucketMesh.normals[vi*3+2] = normals[f][2];
                vi++;
            }
        }
    };
    
    float wt = wallThickness, bw = bucketWidth, bd = bucketDepth, bh = bucketHeight;
    addBox(0, wt/2.0f, 0, bw/2.0f, wt/2.0f, bd/2.0f);
    addBox(-bw/2.0f - wt/2.0f, bh/2.0f + wt, 0, wt/2.0f, bh/2.0f, bd/2.0f);
    addBox(bw/2.0f + wt/2.0f, bh/2.0f + wt, 0, wt/2.0f, bh/2.0f, bd/2.0f);
    addBox(0, bh/2.0f + wt, -bd/2.0f - wt/2.0f, bw/2.0f + wt, bh/2.0f, wt/2.0f);
    addBox(0, wt/2.0f, bd/2.0f + wt/2.0f, bw/2.0f + wt, wt/2.0f, wt/2.0f);
    
    UploadMesh(&bucketMesh, false);
    Model model = LoadModelFromMesh(bucketMesh);
    model.materials[0].shader = shader;
    return model;
}

int main() {
    const int screenWidth = 1200;
    const int screenHeight = 800;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "Terrain Scoop Simulation - Jolt Physics + raylib");

    // Initialize Jolt Physics - MUST be done before creating any shapes
    RegisterDefaultAllocator();
    Factory::sInstance = new Factory();
    RegisterTypes();
    
    printf("DEBUG: Jolt Physics initialized\n"); fflush(stdout);

    TempAllocatorImpl temp_allocator(10 * 1024 * 1024);
    JobSystemThreadPool job_system(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);
    
    printf("DEBUG: Temp allocator and job system created\n"); fflush(stdout);

    const uint cMaxBodies = 4096;
    const uint cNumBodyMutexes = 0;
    const uint cMaxBodyPairs = 8192;
    const uint cMaxContactConstraints = 8192;
    const int maxSphereCount = 500;

    BPLayerInterfaceImpl broad_phase_layer_interface;
    ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;
    ObjectLayerPairFilterImpl object_vs_object_layer_filter;

    PhysicsSystem physics_system;
    physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
        broad_phase_layer_interface, object_vs_broadphase_layer_filter, object_vs_object_layer_filter);

    MyContactListener contact_listener;
    physics_system.SetContactListener(&contact_listener);

    BodyInterface &body_interface = physics_system.GetBodyInterface();
    
    printf("DEBUG: Physics system initialized\n"); fflush(stdout);

    // Camera setup
    Camera3D camera = { 0 };
    camera.position = Vector3{ 8.0f, 6.0f, 8.0f };
    camera.target = Vector3{ 0.0f, 0.0f, 0.0f };
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    float cameraYaw = atan2f(camera.position.z - camera.target.z, camera.position.x - camera.target.x);
    float cameraRadius = Vector3Distance(camera.position, camera.target);
    float cameraPitch = asinf((camera.position.y - camera.target.y) / cameraRadius);
    Vector2 previousMousePos = GetMousePosition();

    // Load shaders
    Shader shader = LoadShader("shaders/directional_light.vs", "shaders/directional_light.fs");
    Shader heightmapShader = LoadShader("shaders/heightmap.vs", "shaders/heightmap.fs");
    
    int lightDirLoc = GetShaderLocation(shader, "lightDirection");
    int lightColorLoc = GetShaderLocation(shader, "lightColor");
    int ambientColorLoc = GetShaderLocation(shader, "ambientColor");
    int viewPosLoc = GetShaderLocation(shader, "viewPos");
    int ambientStrengthLoc = GetShaderLocation(shader, "ambientStrength");
    int specularStrengthLoc = GetShaderLocation(shader, "specularStrength");
    int shininessLoc = GetShaderLocation(shader, "shininess");
    
    int hm_lightDirLoc = GetShaderLocation(heightmapShader, "lightDirection");
    int hm_lightColorLoc = GetShaderLocation(heightmapShader, "lightColor");
    int hm_ambientColorLoc = GetShaderLocation(heightmapShader, "ambientColor");
    int hm_viewPosLoc = GetShaderLocation(heightmapShader, "viewPos");
    int hm_ambientStrengthLoc = GetShaderLocation(heightmapShader, "ambientStrength");
    int hm_specularStrengthLoc = GetShaderLocation(heightmapShader, "specularStrength");
    int hm_shininessLoc = GetShaderLocation(heightmapShader, "shininess");
    int hm_heightScaleLoc = GetShaderLocation(heightmapShader, "heightScale");
    
    Vector3 lightDirection = Vector3Normalize(Vector3{ -0.5f, -1.0f, -0.3f });
    Vector3 lightColor = { 1.0f, 1.0f, 1.0f };
    Vector3 ambientColor = { 0.3f, 0.3f, 0.3f };
    float ambientStrength = 0.3f, specularStrength = 0.5f, shininess = 32.0f;
    
    SetShaderValue(shader, lightDirLoc, &lightDirection, SHADER_UNIFORM_VEC3);
    SetShaderValue(shader, lightColorLoc, &lightColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(shader, ambientColorLoc, &ambientColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(shader, ambientStrengthLoc, &ambientStrength, SHADER_UNIFORM_FLOAT);
    SetShaderValue(shader, specularStrengthLoc, &specularStrength, SHADER_UNIFORM_FLOAT);
    SetShaderValue(shader, shininessLoc, &shininess, SHADER_UNIFORM_FLOAT);
    
    float heightScale = 1.0f;
    SetShaderValue(heightmapShader, hm_lightDirLoc, &lightDirection, SHADER_UNIFORM_VEC3);
    SetShaderValue(heightmapShader, hm_lightColorLoc, &lightColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(heightmapShader, hm_ambientColorLoc, &ambientColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(heightmapShader, hm_ambientStrengthLoc, &ambientStrength, SHADER_UNIFORM_FLOAT);
    SetShaderValue(heightmapShader, hm_specularStrengthLoc, &specularStrength, SHADER_UNIFORM_FLOAT);
    SetShaderValue(heightmapShader, hm_shininessLoc, &shininess, SHADER_UNIFORM_FLOAT);
    SetShaderValue(heightmapShader, hm_heightScaleLoc, &heightScale, SHADER_UNIFORM_FLOAT);
    
    // FLAT TERRAIN WITH 20 CM INITIAL HEIGHT
    int heightmapSize = 128;
    float initialTerrainHeight = 0.2f; // 20 cm
    float terrainWorldSize = 10.0f;
    float terrainScale = terrainWorldSize / (float)heightmapSize;
    
    Image heightmapImage = GenImageColor(heightmapSize, heightmapSize, ::WHITE);
    unsigned char hv = (unsigned char)((initialTerrainHeight / heightScale) * 255.0f);
    ImageDrawRectangle(&heightmapImage, 0, 0, heightmapSize, heightmapSize, {hv, hv, hv, 255});
    Texture2D heightmapTexture = LoadTextureFromImage(heightmapImage);
    SetTextureFilter(heightmapTexture, TEXTURE_FILTER_BILINEAR);
    
    std::vector<float> heightSamples(heightmapSize * heightmapSize, initialTerrainHeight);
    
    printf("DEBUG: Creating HeightFieldShape\n"); fflush(stdout);
    HeightFieldShapeSettings hf_settings(heightSamples.data(), Vec3(0,0,0), 
        Vec3(terrainScale, 1.0f, terrainScale), heightmapSize);
    ShapeRefC hf_shape = hf_settings.Create().Get();
    printf("DEBUG: HeightFieldShape created\n"); fflush(stdout);
    
    float terrainOffset = -terrainWorldSize / 2.0f;
    BodyCreationSettings hm_body_settings(hf_shape, RVec3(terrainOffset, 0.0, terrainOffset),
        Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);
    hm_body_settings.mFriction = 0.8f;
    Body* heightmap_body = body_interface.CreateBody(hm_body_settings);
    body_interface.AddBody(heightmap_body->GetID(), EActivation::DontActivate);
    UnloadImage(heightmapImage);
    
    Mesh planeMesh = GenMeshPlane(terrainWorldSize, terrainWorldSize, 64, 64);
    Model planeModel = LoadModelFromMesh(planeMesh);
    planeModel.materials[0].shader = heightmapShader;
    planeModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = heightmapTexture;
    
    DensityAutomata densityGrid(heightmapSize, terrainScale);
    
    // Sphere model for dirt particles
    float sphereRadius = 0.03f;
    float sphereDensity = 1800.0f;
    float sphereVolume = (4.0f/3.0f) * 3.14159f * sphereRadius * sphereRadius * sphereRadius;
    float sphereMass = sphereDensity * sphereVolume;
    Model sphereModel = LoadModelFromMesh(GenMeshSphere(sphereRadius, 12, 12));
    sphereModel.materials[0].shader = shader;
    
    // BUCKET SCOOP
    printf("DEBUG: Creating bucket shape\n"); fflush(stdout);
    float bucketWidth = 0.6f, bucketDepth = 0.3f, bucketHeight = 0.2f, bucketWallThickness = 0.03f;
    ShapeRefC bucket_shape = CreateBucketShape(bucketWidth, bucketDepth, bucketHeight, bucketWallThickness);
    printf("DEBUG: Bucket shape created\n"); fflush(stdout);
    Model bucketModel = CreateBucketModel(bucketWidth, bucketDepth, bucketHeight, bucketWallThickness, shader);
    printf("DEBUG: Bucket model created\n"); fflush(stdout);
    
    Vector3 bucketPosition = { -3.0f, 1.0f, 0.0f };
    float bucketPitch = 0.0f, bucketYaw = 0.0f;
    Quat bucketRotation = Quat::sRotation(Vec3(0,1,0), bucketYaw) * Quat::sRotation(Vec3(1,0,0), bucketPitch);
    
    BodyCreationSettings bucket_body_settings(bucket_shape, RVec3(bucketPosition.x, bucketPosition.y, bucketPosition.z),
        bucketRotation, EMotionType::Kinematic, Layers::MOVING);
    bucket_body_settings.mFriction = 0.8f;
    bucket_body_settings.mRestitution = 0.1f;
    Body* bucket_body = body_interface.CreateBody(bucket_body_settings);
    BodyID bucket_body_id = bucket_body->GetID();
    body_interface.AddBody(bucket_body_id, EActivation::Activate);
    
    // ========================
    // NEW: CLASS-BASED BLADE (for pushing/bulldozing terrain)
    // ========================
    printf("DEBUG: Creating Blade class\n"); fflush(stdout);
    Blade blade(0.8f, 0.3f, 0.08f);  // Width, height, thickness
    blade.SetPosition(Vector3{3.0f, 0.5f, 0.0f});
    blade.SetShader(&shader);
    blade.SetDriveMode(Blade::DriveMode::Manual);
    blade.SetMaxSpeed(2.0f);
    blade.SetEnginePower(300.0f);
    blade.SetBladeRotation(30.0f);
    blade.SetHeightOffset(0.05f);
    printf("DEBUG: Initializing Blade physics\n"); fflush(stdout);
    blade.Initialize(body_interface);
    printf("DEBUG: Blade initialized\n"); fflush(stdout);
    
    bool bladeActive = false;  // Toggle with 'B' key
    
    // SCOOP STATE MACHINE
    ScoopState currentState = ScoopState::IDLE;
    float stateTimer = 0.0f;
    
    Vector3 startPosition = { -3.0f, 1.0f, 0.0f };
    Vector3 digPosition = { -1.5f, 0.0f, 0.0f };
    Vector3 dropPosition = { 2.5f, 0.0f, 0.0f };
    float liftHeight = 0.8f, digDepth = 0.08f;
    float moveSpeed = 1.5f, liftSpeed = 0.8f, pitchSpeed = 1.2f;
    
    bool autoScoop = false;
    float scoopedVolume = 0.0f, targetPitch = 0.0f;
    Vector3 targetPosition = startPosition;
    
    bool showGui = true;
    int activeSlider = -1;
    float guiDigX = -1.5f, guiDigZ = 0.0f, guiDropX = 2.5f, guiDropZ = 0.0f;
    float guiMoveSpeed = 1.5f, guiDigDepth = 0.08f;
    float guiParticleDamping = 0.8f; // Higher = less rolling
    
    std::vector<PhysicsSphere> dynamicSpheres;
    const float stoppedTimeThreshold = 1.0f;
    
    Model markerModel = LoadModelFromMesh(GenMeshCylinder(0.1f, 0.02f, 8));
    
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        UpdateOrbitalCamera(&camera, &cameraYaw, &cameraPitch, &cameraRadius, &previousMousePos);
        
        const float deltaTime = GetFrameTime();
        stateTimer += deltaTime;
        
        digPosition.x = guiDigX; digPosition.z = guiDigZ;
        dropPosition.x = guiDropX; dropPosition.z = guiDropZ;
        moveSpeed = guiMoveSpeed; digDepth = guiDigDepth;
        
        if (IsKeyPressed(KEY_SPACE)) {
            autoScoop = !autoScoop;
            if (autoScoop && currentState == ScoopState::IDLE) {
                currentState = ScoopState::MOVING_TO_DIG;
                stateTimer = 0.0f;
            }
        }
        
        if (IsKeyPressed(KEY_ENTER) && !autoScoop && currentState == ScoopState::IDLE) {
            currentState = ScoopState::MOVING_TO_DIG;
            stateTimer = 0.0f;
        }
        
        if (IsKeyPressed(KEY_R)) {
            bucketPosition = startPosition;
            bucketPitch = 0.0f;
            currentState = ScoopState::IDLE;
            autoScoop = false;
            scoopedVolume = 0.0f;
        }
        
        // STATE MACHINE
        switch (currentState) {
            case ScoopState::IDLE:
                targetPosition = startPosition;
                targetPitch = 0.0f;
                break;
                
            case ScoopState::MOVING_TO_DIG:
                targetPosition = { digPosition.x, liftHeight, digPosition.z };
                targetPitch = -0.3f;
                if (Vector3Distance(bucketPosition, targetPosition) < 0.1f) {
                    currentState = ScoopState::DIGGING;
                    stateTimer = 0.0f;
                }
                break;
                
            case ScoopState::DIGGING: {
                int hmX = (int)((digPosition.x + terrainWorldSize/2.0f) / terrainScale);
                int hmZ = (int)((digPosition.z + terrainWorldSize/2.0f) / terrainScale);
                hmX = Clamp(hmX, 0, heightmapSize - 1);
                hmZ = Clamp(hmZ, 0, heightmapSize - 1);
                float terrainHeight = heightSamples[hmZ * heightmapSize + hmX];
                
                targetPosition = { digPosition.x, terrainHeight - digDepth + bucketWallThickness, digPosition.z };
                targetPitch = -0.5f;
                
                if (bucketPosition.y < terrainHeight + 0.05f) {
                    int digRadiusH = (int)(bucketWidth / terrainScale / 2.0f) + 1;
                    int digRadiusD = (int)(bucketDepth / terrainScale / 2.0f) + 1;
                    
                    for (int dz = -digRadiusD; dz <= digRadiusD; dz++) {
                        for (int dx = -digRadiusH; dx <= digRadiusH; dx++) {
                            int tx = hmX + dx, tz = hmZ + dz;
                            if (tx >= 0 && tx < heightmapSize && tz >= 0 && tz < heightmapSize) {
                                int idx = tz * heightmapSize + tx;
                                float digAmt = digDepth * deltaTime * 2.0f;
                                if (heightSamples[idx] > 0.0f) {
                                    float actualDig = fminf(digAmt, heightSamples[idx]);
                                    heightSamples[idx] -= actualDig;
                                    scoopedVolume += actualDig * terrainScale * terrainScale;
                                }
                            }
                        }
                    }
                    
                    while (scoopedVolume > sphereVolume * 3 && (int)dynamicSpheres.size() < maxSphereCount) {
                        scoopedVolume -= sphereVolume;
                        
                        // Spawn in bucket's local space (inside the bucket)
                        float localX = ((float)GetRandomValue(-30, 30) / 100.0f) * bucketWidth * 0.4f;
                        float localY = bucketHeight * 0.3f + bucketWallThickness;  // Inside the bucket walls
                        float localZ = ((float)GetRandomValue(-30, 30) / 100.0f) * bucketDepth * 0.3f;
                        
                        // Transform local position by bucket rotation (pitch around X axis)
                        float cosPitch = cosf(bucketPitch);
                        float sinPitch = sinf(bucketPitch);
                        float rotatedY = localY * cosPitch - localZ * sinPitch;
                        float rotatedZ = localY * sinPitch + localZ * cosPitch;
                        
                        // Final world position
                        float spawnX = bucketPosition.x + localX;
                        float spawnY = bucketPosition.y + rotatedY;
                        float spawnZ = bucketPosition.z + rotatedZ;
                        
                        SphereShapeSettings sss(sphereRadius);
                        ShapeRefC ss = sss.Create().Get();
                        
                        BodyCreationSettings sbs(ss, RVec3(spawnX, spawnY, spawnZ), Quat::sIdentity(),
                            EMotionType::Dynamic, Layers::MOVING);
                        sbs.mFriction = 0.95f;
                        sbs.mRestitution = 0.02f;
                        sbs.mLinearDamping = guiParticleDamping;
                        sbs.mAngularDamping = guiParticleDamping * 1.5f;
                        
                        Body* sb = body_interface.CreateBody(sbs);
                        if (sb) {
                            BodyID sid = sb->GetID();
                            body_interface.AddBody(sid, EActivation::Activate);
                            dynamicSpheres.push_back({sid, {139, 90, 43, 255}, 0.0f, false, 
                                {spawnX, spawnY, spawnZ}, {0,0,0}, sphereMass});
                        }
                    }
                }
                
                if (bucketPosition.y <= targetPosition.y + 0.02f && stateTimer > 0.5f) {
                    currentState = ScoopState::LIFTING;
                    stateTimer = 0.0f;
                }
            } break;
                
            case ScoopState::LIFTING:
                targetPosition = { digPosition.x, liftHeight, digPosition.z };
                targetPitch = -0.6f;
                if (bucketPosition.y >= liftHeight - 0.05f) {
                    currentState = ScoopState::MOVING_TO_DROP;
                    stateTimer = 0.0f;
                }
                break;
                
            case ScoopState::MOVING_TO_DROP:
                targetPosition = { dropPosition.x, liftHeight, dropPosition.z };
                targetPitch = -0.6f;
                if (Vector3Distance(bucketPosition, targetPosition) < 0.1f) {
                    currentState = ScoopState::DROPPING;
                    stateTimer = 0.0f;
                }
                break;
                
            case ScoopState::DROPPING:
                targetPitch = 0.8f;
                if (stateTimer > 1.5f) {
                    currentState = ScoopState::RETURNING;
                    stateTimer = 0.0f;
                }
                break;
                
            case ScoopState::RETURNING:
                targetPosition = startPosition;
                targetPitch = 0.0f;
                if (Vector3Distance(bucketPosition, targetPosition) < 0.1f && fabsf(bucketPitch) < 0.1f) {
                    currentState = ScoopState::IDLE;
                    if (autoScoop) currentState = ScoopState::MOVING_TO_DIG;
                    stateTimer = 0.0f;
                }
                break;
        }
        
        // Smooth movement
        Vector3 toTarget = Vector3Subtract(targetPosition, bucketPosition);
        float dist = Vector3Length(toTarget);
        if (dist > 0.01f) {
            float moveAmt = fminf(moveSpeed * deltaTime, dist);
            bucketPosition = Vector3Add(bucketPosition, Vector3Scale(toTarget, moveAmt / dist));
        }
        
        float pitchDiff = targetPitch - bucketPitch;
        if (fabsf(pitchDiff) > 0.01f) {
            float pitchAmt = pitchSpeed * deltaTime;
            bucketPitch += (fabsf(pitchDiff) < pitchAmt) ? pitchDiff : (pitchDiff > 0 ? pitchAmt : -pitchAmt);
        }
        
        bucketRotation = Quat::sRotation(Vec3(0,1,0), bucketYaw) * Quat::sRotation(Vec3(1,0,0), bucketPitch);
        
        // Use MoveKinematic instead of SetPositionAndRotation for proper collision with dynamic bodies
        body_interface.MoveKinematic(bucket_body_id,
            RVec3(bucketPosition.x, bucketPosition.y, bucketPosition.z), bucketRotation, deltaTime);
        
        // ========================
        // NEW: Update blade if active
        // ========================
        if (IsKeyPressed(KEY_B)) bladeActive = !bladeActive;
        
        if (bladeActive) {
            // Handle blade manual input
            blade.HandleManualInput(deltaTime, 
                IsKeyDown(KEY_I),  // Forward
                IsKeyDown(KEY_K),  // Backward
                IsKeyDown(KEY_J),  // Turn left
                IsKeyDown(KEY_L)); // Turn right
            
            // Create dummy particle voxel grid (blade doesn't use it in simple mode)
            ParticleVoxelGrid dummyGrid(1, 1, 1, 1.0f, 0, 0, 0, 1, 1, 1);
            
            // Update blade
            blade.Update(deltaTime, body_interface, heightSamples, heightmapSize,
                        terrainScale, heightScale, densityGrid, dummyGrid);
            
            // Dig terrain with blade
            std::vector<Vector3> dugPositions;
            float displacedVolume = blade.DigTerrain(heightSamples, heightmapSize, 
                                                      terrainScale, heightScale, deltaTime, dugPositions);
            
            // Spawn spheres from displaced terrain
            while (displacedVolume > sphereVolume * 2 && (int)dynamicSpheres.size() < maxSphereCount) {
                displacedVolume -= sphereVolume;
                
                Vector3 bladePos = blade.GetPosition();
                float spawnX = bladePos.x + ((float)GetRandomValue(-30, 30) / 100.0f) * 0.4f;
                float spawnY = bladePos.y + 0.1f;
                float spawnZ = bladePos.z + ((float)GetRandomValue(-30, 30) / 100.0f) * 0.4f;
                
                SphereShapeSettings sss(sphereRadius);
                ShapeRefC ss = sss.Create().Get();
                
                BodyCreationSettings sbs(ss, RVec3(spawnX, spawnY, spawnZ), Quat::sIdentity(),
                    EMotionType::Dynamic, Layers::MOVING);
                sbs.mFriction = 0.9f;
                sbs.mRestitution = 0.05f;
                sbs.mLinearDamping = 0.5f;
                sbs.mAngularDamping = 0.5f;
                
                Body* sb = body_interface.CreateBody(sbs);
                if (sb) {
                    BodyID sid = sb->GetID();
                    body_interface.AddBody(sid, EActivation::Activate);
                    dynamicSpheres.push_back({sid, {139, 90, 43, 255}, 0.0f, false, 
                        {spawnX, spawnY, spawnZ}, {0,0,0}, sphereMass});
                }
            }
        }
        
        physics_system.Update(deltaTime, 1, &temp_allocator, &job_system);
        
        // Update spheres
        for (auto& sphere : dynamicSpheres) {
            if (sphere.markedForDestruction) continue;
            
            RVec3 pos = body_interface.GetPosition(sphere.bodyID);
            Vec3 vel = body_interface.GetLinearVelocity(sphere.bodyID);
            Vector3 currentPos = { (float)pos.GetX(), (float)pos.GetY(), (float)pos.GetZ() };
            
            float speed = sqrtf(vel.GetX()*vel.GetX() + vel.GetY()*vel.GetY() + vel.GetZ()*vel.GetZ());
            if (speed < 0.05f && currentPos.y < 0.5f) sphere.stoppedTimer += deltaTime;
            else sphere.stoppedTimer = 0.0f;
            
            if (sphere.stoppedTimer > stoppedTimeThreshold) {
                int hmX = (int)((currentPos.x + terrainWorldSize/2.0f) / terrainScale);
                int hmZ = (int)((currentPos.z + terrainWorldSize/2.0f) / terrainScale);
                if (hmX >= 0 && hmX < heightmapSize && hmZ >= 0 && hmZ < heightmapSize) {
                    heightSamples[hmZ * heightmapSize + hmX] += sphereVolume / (terrainScale * terrainScale) * 0.5f;
                }
                sphere.markedForDestruction = true;
            }
            
            if (currentPos.y < -2.0f || fabsf(currentPos.x) > 8.0f || fabsf(currentPos.z) > 8.0f)
                sphere.markedForDestruction = true;
            
            sphere.lastPosition = currentPos;
        }
        
        for (auto it = dynamicSpheres.begin(); it != dynamicSpheres.end(); ) {
            if (it->markedForDestruction) {
                body_interface.RemoveBody(it->bodyID);
                body_interface.DestroyBody(it->bodyID);
                it = dynamicSpheres.erase(it);
            } else ++it;
        }
        
        // Update heightmap physics periodically
        static int frameCounter = 0;
        if (++frameCounter % 30 == 0) {
            body_interface.RemoveBody(heightmap_body->GetID());
            body_interface.DestroyBody(heightmap_body->GetID());
            
            HeightFieldShapeSettings new_hf(heightSamples.data(), Vec3(0,0,0), 
                Vec3(terrainScale, 1.0f, terrainScale), heightmapSize);
            ShapeRefC new_shape = new_hf.Create().Get();
            
            BodyCreationSettings new_hm(new_shape, RVec3(terrainOffset, 0.0, terrainOffset),
                Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);
            new_hm.mFriction = 0.8f;
            heightmap_body = body_interface.CreateBody(new_hm);
            body_interface.AddBody(heightmap_body->GetID(), EActivation::DontActivate);
            
            ::Color* newPixels = (::Color*)RL_MALLOC(heightmapSize * heightmapSize * sizeof(::Color));
            for (int i = 0; i < heightmapSize * heightmapSize; i++) {
                unsigned char val = (unsigned char)Clamp(heightSamples[i] / heightScale * 255.0f, 0.0f, 255.0f);
                newPixels[i] = { val, val, val, 255 };
            }
            UpdateTexture(heightmapTexture, newPixels);
            RL_FREE(newPixels);
        }
        
        SetShaderValue(shader, viewPosLoc, &camera.position, SHADER_UNIFORM_VEC3);
        SetShaderValue(heightmapShader, hm_viewPosLoc, &camera.position, SHADER_UNIFORM_VEC3);
        
        // RENDERING
        BeginDrawing();
        ClearBackground(::SKYBLUE);

        BeginMode3D(camera);
        
        DrawModel(planeModel, Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, ::BROWN);
        DrawGrid(20, 0.5f);
        
        float bucketPitchDeg = bucketPitch * 180.0f / 3.14159f;
        DrawModelEx(bucketModel, bucketPosition, Vector3{1,0,0}, bucketPitchDeg, Vector3{1,1,1}, ::ORANGE);
        
        // Draw blade if active
        if (bladeActive) {
            blade.Draw();
        }
        
        for (const auto& sphere : dynamicSpheres) {
            RVec3 pos = body_interface.GetPosition(sphere.bodyID);
            DrawModel(sphereModel, {(float)pos.GetX(), (float)pos.GetY(), (float)pos.GetZ()}, 1.0f, sphere.color);
        }
        
        DrawModel(markerModel, Vector3{digPosition.x, 0.5f, digPosition.z}, 1.0f, ::RED);
        DrawCylinderWires(Vector3{digPosition.x, 0.0f, digPosition.z}, bucketWidth/2, bucketWidth/2, 0.01f, 16, ::RED);
        
        DrawModel(markerModel, Vector3{dropPosition.x, 0.5f, dropPosition.z}, 1.0f, ::GREEN);
        DrawCylinderWires(Vector3{dropPosition.x, 0.0f, dropPosition.z}, bucketWidth/2, bucketWidth/2, 0.01f, 16, ::GREEN);
        
        EndMode3D();

        // GUI
        DrawText("TERRAIN SCOOP SIMULATION", 10, 10, 24, ::DARKGRAY);
        DrawText(TextFormat("State: %s", GetStateName(currentState)), 10, 40, 20, ::BLUE);
        DrawText(TextFormat("Spheres: %d / %d", (int)dynamicSpheres.size(), maxSphereCount), 10, 65, 16, ::DARKGRAY);
        DrawText(TextFormat("Auto: %s", autoScoop ? "ON" : "OFF"), 10, 85, 16, autoScoop ? ::GREEN : ::RED);
        
        DrawText("Controls:", 10, 115, 16, ::DARKGRAY);
        DrawText("SPACE - Toggle auto scoop", 10, 135, 14, ::GRAY);
        DrawText("ENTER - Single step", 10, 150, 14, ::GRAY);
        DrawText("R - Reset", 10, 165, 14, ::GRAY);
        DrawText("G - Toggle GUI", 10, 180, 14, ::GRAY);
        DrawText("B - Toggle Blade", 10, 195, 14, ::GRAY);
        
        if (bladeActive) {
            DrawText("Blade ON - Use I/K/J/L to drive", 10, 215, 14, ::BLUE);
            Vector3 bladePos = blade.GetPosition();
            DrawText(TextFormat("Blade: (%.1f, %.1f) Speed: %.2f", bladePos.x, bladePos.z, blade.GetCurrentSpeed()), 
                     10, 235, 14, ::BLUE);
        }
        
        if (IsKeyPressed(KEY_G)) showGui = !showGui;
        
        if (showGui) {
            int panelX = GetScreenWidth() - 240, panelY = 10, panelW = 230, panelH = 220;
            
            DrawRectangle(panelX, panelY, panelW, panelH, Fade(::LIGHTGRAY, 0.9f));
            DrawRectangleLines(panelX, panelY, panelW, panelH, ::DARKGRAY);
            DrawText("Settings", panelX + 5, panelY + 5, 14, ::DARKGRAY);
            
            Vector2 mousePos = GetMousePosition();
            bool mouseDown = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
            bool mousePressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
            bool mouseReleased = IsMouseButtonReleased(MOUSE_BUTTON_LEFT);
            
            auto drawSlider = [&](const char* label, float& val, float minV, float maxV, int y, ::Color c, int id) {
                int sliderX = panelX + 70, sliderW = 120, sliderH = 14;
                DrawText(label, panelX + 5, y, 12, ::DARKGRAY);
                DrawRectangle(sliderX, y, sliderW, sliderH, ::DARKGRAY);
                float norm = (val - minV) / (maxV - minV);
                DrawRectangle(sliderX + (int)(norm * (sliderW - 8)), y, 8, sliderH, c);
                Rectangle rect = { (float)sliderX, (float)y, (float)sliderW, (float)sliderH };
                if (CheckCollisionPointRec(mousePos, rect) && mousePressed) activeSlider = id;
                if (activeSlider == id && mouseDown)
                    val = minV + Clamp((mousePos.x - sliderX) / sliderW, 0.0f, 1.0f) * (maxV - minV);
                DrawText(TextFormat("%.1f", val), sliderX + sliderW + 5, y, 12, ::BLACK);
            };
            
            drawSlider("Dig X:", guiDigX, -4.0f, 4.0f, panelY + 30, ::RED, 0);
            drawSlider("Dig Z:", guiDigZ, -4.0f, 4.0f, panelY + 55, ::RED, 1);
            drawSlider("Drop X:", guiDropX, -4.0f, 4.0f, panelY + 80, ::GREEN, 2);
            drawSlider("Drop Z:", guiDropZ, -4.0f, 4.0f, panelY + 105, ::GREEN, 3);
            drawSlider("Speed:", guiMoveSpeed, 0.5f, 4.0f, panelY + 130, ::BLUE, 4);
            
            int y = panelY + 155;
            DrawText("Depth:", panelX + 5, y, 12, ::DARKGRAY);
            DrawRectangle(panelX + 70, y, 120, 14, ::DARKGRAY);
            float norm = (guiDigDepth - 0.02f) / (0.15f - 0.02f);
            DrawRectangle(panelX + 70 + (int)(norm * 112), y, 8, 14, ::MAROON);
            Rectangle rect = { (float)(panelX + 70), (float)y, 120.0f, 14.0f };
            if (CheckCollisionPointRec(mousePos, rect) && mousePressed) activeSlider = 5;
            if (activeSlider == 5 && mouseDown)
                guiDigDepth = 0.02f + Clamp((mousePos.x - panelX - 70) / 120.0f, 0.0f, 1.0f) * (0.15f - 0.02f);
            DrawText(TextFormat("%.0fcm", guiDigDepth * 100), panelX + 195, y, 12, ::BLACK);
            
            // Damping slider (less rolling)
            y = panelY + 175;
            DrawText("Damping:", panelX + 5, y, 12, ::DARKGRAY);
            DrawRectangle(panelX + 70, y, 120, 14, ::DARKGRAY);
            float normDamp = (guiParticleDamping - 0.2f) / (1.5f - 0.2f);
            DrawRectangle(panelX + 70 + (int)(normDamp * 112), y, 8, 14, ::PURPLE);
            Rectangle rectDamp = { (float)(panelX + 70), (float)y, 120.0f, 14.0f };
            if (CheckCollisionPointRec(mousePos, rectDamp) && mousePressed) activeSlider = 6;
            if (activeSlider == 6 && mouseDown)
                guiParticleDamping = 0.2f + Clamp((mousePos.x - panelX - 70) / 120.0f, 0.0f, 1.0f) * (1.5f - 0.2f);
            DrawText(TextFormat("%.1f", guiParticleDamping), panelX + 195, y, 12, ::BLACK);
            
            DrawText("Initial terrain: 20cm", panelX + 5, panelY + 200, 12, ::GRAY);
            
            if (mouseReleased) activeSlider = -1;
        }
        
        DrawFPS(GetScreenWidth() - 100, GetScreenHeight() - 30);

        EndDrawing();
    }

    // Cleanup
    for (const auto& sphere : dynamicSpheres) {
        body_interface.RemoveBody(sphere.bodyID);
        body_interface.DestroyBody(sphere.bodyID);
    }
    blade.Cleanup(body_interface);
    body_interface.RemoveBody(bucket_body_id);
    body_interface.DestroyBody(bucket_body_id);
    body_interface.RemoveBody(heightmap_body->GetID());
    body_interface.DestroyBody(heightmap_body->GetID());

    UnloadShader(shader);
    UnloadShader(heightmapShader);
    UnloadTexture(heightmapTexture);
    UnloadModel(sphereModel);
    UnloadModel(bucketModel);
    UnloadModel(planeModel);
    UnloadModel(markerModel);

    CloseWindow();
    return 0;
}
