#include "raylib.h"
#include "raymath.h"
#include "MeshCutter.h"
#include "PhysicsLayers.h"

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

using namespace JPH;

// Jolt memory allocation hooks
static void* JoltAlloc(size_t inSize) { return malloc(inSize); }
static void JoltFree(void* inBlock) { free(inBlock); }
static void* JoltAlignedAlloc(size_t inSize, size_t inAlignment) { return aligned_alloc(inAlignment, inSize); }
static void JoltAlignedFree(void* inBlock) { free(inBlock); }

int main()
{
    // Initialize window
    const int screenWidth = 1280;
    const int screenHeight = 720;
    
    InitWindow(screenWidth, screenHeight, "Mesh Cutting with Physics");
    SetTargetFPS(60);
    
    // Initialize Jolt Physics
    RegisterDefaultAllocator();
    Factory::sInstance = new Factory();
    RegisterTypes();
    
    // Setup Jolt allocators
    TempAllocatorImpl tempAllocator(10 * 1024 * 1024);
    JobSystemThreadPool jobSystem(cMaxPhysicsJobs, cMaxPhysicsBarriers, 
                                   std::thread::hardware_concurrency() - 1);
    
    // Physics system configuration
    const uint maxBodies = 1024;
    const uint numBodyMutexes = 0;
    const uint maxBodyPairs = 1024;
    const uint maxContactConstraints = 1024;
    
    BPLayerInterfaceImpl broadPhaseLayerInterface;
    ObjectVsBroadPhaseLayerFilterImpl objectVsBroadPhaseLayerFilter;
    ObjectLayerPairFilterImpl objectLayerPairFilter;
    
    PhysicsSystem physicsSystem;
    physicsSystem.Init(maxBodies, numBodyMutexes, maxBodyPairs, maxContactConstraints,
                       broadPhaseLayerInterface, objectVsBroadPhaseLayerFilter, 
                       objectLayerPairFilter);
    
    physicsSystem.SetGravity(Vec3(0, -9.81f, 0));
    
    // Create ground plane
    BodyInterface& bodyInterface = physicsSystem.GetBodyInterface();
    BoxShapeSettings groundShapeSettings(Vec3(50.0f, 0.5f, 50.0f));
    BodyCreationSettings groundSettings(
        groundShapeSettings.Create().Get(),
        RVec3(0, -0.5f, 0),
        Quat::sIdentity(),
        EMotionType::Static,
        Layers::NON_MOVING
    );
    bodyInterface.CreateAndAddBody(groundSettings, EActivation::DontActivate);
    
    // Create mesh manager
    CuttableMeshManager meshManager(&physicsSystem);
    
    // Create initial cube
    meshManager.CreateCube({ 0, 5, 0 }, 2.0f, ::RED);
    
    // Setup camera
    Camera3D camera = { 0 };
    camera.position = { 10.0f, 10.0f, 10.0f };
    camera.target = { 0.0f, 2.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    
    // Cutting plane visualization
    Vector3 planePosition = { 0, 3, 0 };
    Vector3 planeNormal = { 1, 0, 0 };
    float planeYaw = 0.0f;    // Rotation around Y axis (A/D)
    float planePitch = 0.0f;  // Rotation around horizontal axis (W/S)
    bool showPlane = true;
    
    // Physics timing
    const float physicsTimeStep = 1.0f / 60.0f;
    float physicsAccumulator = 0.0f;
    
    // Main game loop
    while (!WindowShouldClose())
    {
        float deltaTime = GetFrameTime();
        
        // Update camera
        UpdateCamera(&camera, CAMERA_ORBITAL);
        
        // Control cutting plane
        if (IsKeyDown(KEY_UP)) planePosition.y += 2.0f * deltaTime;
        if (IsKeyDown(KEY_DOWN)) planePosition.y -= 2.0f * deltaTime;
        if (IsKeyDown(KEY_LEFT)) planePosition.x -= 2.0f * deltaTime;
        if (IsKeyDown(KEY_RIGHT)) planePosition.x += 2.0f * deltaTime;
        if (IsKeyDown(KEY_Q)) planePosition.z -= 2.0f * deltaTime;
        if (IsKeyDown(KEY_E)) planePosition.z += 2.0f * deltaTime;
        
        // Rotate plane normal (yaw - around Y axis)
        if (IsKeyDown(KEY_A)) planeYaw -= 1.5f * deltaTime;
        if (IsKeyDown(KEY_D)) planeYaw += 1.5f * deltaTime;
        
        // Tilt plane normal (pitch - tilt up/down)
        if (IsKeyDown(KEY_W)) planePitch -= 1.5f * deltaTime;
        if (IsKeyDown(KEY_S)) planePitch += 1.5f * deltaTime;
        
        // Clamp pitch to avoid gimbal lock issues
        if (planePitch > 1.5f) planePitch = 1.5f;
        if (planePitch < -1.5f) planePitch = -1.5f;
        
        // Calculate plane normal from yaw and pitch angles
        // Start with normal pointing in +X direction, then apply rotations
        planeNormal.x = cosf(planePitch) * cosf(planeYaw);
        planeNormal.y = sinf(planePitch);
        planeNormal.z = cosf(planePitch) * sinf(planeYaw);
        
        // Perform cut
        if (IsKeyPressed(KEY_SPACE))
        {
            CutPlane cutPlane = CutPlane::FromPointNormal(planePosition, planeNormal);
            meshManager.CutWithPlane(cutPlane, planePosition);
        }
        
        // Spawn new cube
        if (IsKeyPressed(KEY_N))
        {
            ::Color colors[] = { ::RED, ::GREEN, ::BLUE, ::YELLOW, ::ORANGE, ::PURPLE, ::PINK };
            int colorIndex = GetRandomValue(0, 6);
            meshManager.CreateCube({ (float)GetRandomValue(-3, 3), 8, (float)GetRandomValue(-3, 3) }, 
                                   2.0f, colors[colorIndex]);
        }
        
        // Spawn new sphere
        if (IsKeyPressed(KEY_M))
        {
            ::Color colors[] = { ::RED, ::GREEN, ::BLUE, ::YELLOW, ::ORANGE, ::PURPLE, ::PINK };
            int colorIndex = GetRandomValue(0, 6);
            meshManager.CreateSphere({ (float)GetRandomValue(-3, 3), 8, (float)GetRandomValue(-3, 3) }, 
                                     1.0f, colors[colorIndex]);
        }
        
        // Toggle plane visibility
        if (IsKeyPressed(KEY_P)) showPlane = !showPlane;
        
        // Reset scene
        if (IsKeyPressed(KEY_R))
        {
            // Will need to properly reset - for now just add a new cube
            meshManager.CreateCube({ 0, 5, 0 }, 2.0f, ::RED);
        }
        
        // Update physics
        physicsAccumulator += deltaTime;
        while (physicsAccumulator >= physicsTimeStep)
        {
            physicsSystem.Update(physicsTimeStep, 1, &tempAllocator, &jobSystem);
            physicsAccumulator -= physicsTimeStep;
        }
        
        // Update mesh manager
        meshManager.Update(deltaTime);
        
        // Draw
        BeginDrawing();
        ClearBackground(::DARKGRAY);
        
        BeginMode3D(camera);
        
        // Draw ground
        DrawPlane({ 0, 0, 0 }, { 20, 20 }, ::GRAY);
        DrawGrid(20, 1.0f);
        
        // Draw cutting plane (visualization)
        if (showPlane)
        {
            // Calculate proper tangent vectors for the plane based on the normal
            // Find a vector not parallel to the normal
            Vector3 tempUp = { 0, 1, 0 };
            if (fabsf(planeNormal.y) > 0.9f)
            {
                tempUp = { 1, 0, 0 };
            }
            
            // right = tempUp x normal (cross product)
            Vector3 right = {
                tempUp.y * planeNormal.z - tempUp.z * planeNormal.y,
                tempUp.z * planeNormal.x - tempUp.x * planeNormal.z,
                tempUp.x * planeNormal.y - tempUp.y * planeNormal.x
            };
            // Normalize right
            float rightLen = sqrtf(right.x * right.x + right.y * right.y + right.z * right.z);
            if (rightLen > 0.0001f)
            {
                right.x /= rightLen;
                right.y /= rightLen;
                right.z /= rightLen;
            }
            
            // up = normal x right (cross product)
            Vector3 up = {
                planeNormal.y * right.z - planeNormal.z * right.y,
                planeNormal.z * right.x - planeNormal.x * right.z,
                planeNormal.x * right.y - planeNormal.y * right.x
            };
            
            float planeSize = 5.0f;
            
            // Four corners of the plane
            Vector3 p1 = { 
                planePosition.x - right.x * planeSize - up.x * planeSize,
                planePosition.y - right.y * planeSize - up.y * planeSize,
                planePosition.z - right.z * planeSize - up.z * planeSize
            };
            Vector3 p2 = { 
                planePosition.x + right.x * planeSize - up.x * planeSize,
                planePosition.y + right.y * planeSize - up.y * planeSize,
                planePosition.z + right.z * planeSize - up.z * planeSize
            };
            Vector3 p3 = { 
                planePosition.x + right.x * planeSize + up.x * planeSize,
                planePosition.y + right.y * planeSize + up.y * planeSize,
                planePosition.z + right.z * planeSize + up.z * planeSize
            };
            Vector3 p4 = { 
                planePosition.x - right.x * planeSize + up.x * planeSize,
                planePosition.y - right.y * planeSize + up.y * planeSize,
                planePosition.z - right.z * planeSize + up.z * planeSize
            };
            
            // Draw plane as two triangles (both sides)
            ::Color planeColor = { 0, 255, 255, 100 };
            DrawTriangle3D(p1, p2, p3, planeColor);
            DrawTriangle3D(p1, p3, p4, planeColor);
            DrawTriangle3D(p3, p2, p1, planeColor);
            DrawTriangle3D(p4, p3, p1, planeColor);
            
            // Draw plane normal
            Vector3 normalEnd = {
                planePosition.x + planeNormal.x * 2,
                planePosition.y + planeNormal.y * 2,
                planePosition.z + planeNormal.z * 2
            };
            DrawLine3D(planePosition, normalEnd, ::YELLOW);
            DrawSphere(planePosition, 0.1f, ::YELLOW);
        }
        
        // Draw meshes
        meshManager.Draw();
        
        EndMode3D();
        
        // Draw UI
        DrawRectangle(10, 10, 320, 240, Fade(::BLACK, 0.7f));
        DrawText("MESH CUTTING DEMO", 20, 20, 20, ::WHITE);
        DrawText("Controls:", 20, 50, 16, ::LIGHTGRAY);
        DrawText("Arrow Keys: Move cutting plane (X/Y)", 30, 70, 14, ::GRAY);
        DrawText("Q/E: Move plane (Z)", 30, 85, 14, ::GRAY);
        DrawText("A/D: Rotate plane", 30, 100, 14, ::GRAY);
        DrawText("W/S: Tilt plane", 30, 115, 14, ::GRAY);
        DrawText("SPACE: Cut meshes", 30, 130, 14, ::GREEN);
        DrawText("N: Spawn new cube", 30, 145, 14, ::BLUE);
        DrawText("M: Spawn new sphere", 30, 160, 14, ::BLUE);
        DrawText("P: Toggle plane visibility", 30, 175, 14, ::GRAY);
        DrawText("R: Add cube at origin", 30, 190, 14, ::GRAY);
        DrawText("Mouse: Orbit camera", 30, 205, 14, ::GRAY);
        
        DrawText(TextFormat("Mesh count: %d", (int)meshManager.GetMeshCount()), 20, 225, 16, ::YELLOW);
        
        DrawFPS(screenWidth - 100, 10);
        
        EndDrawing();
    }
    
    // Cleanup
    CloseWindow();
    
    // Cleanup Jolt
    delete Factory::sInstance;
    Factory::sInstance = nullptr;
    
    return 0;
}
