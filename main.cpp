#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <cmath>

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

    InitWindow(screenWidth, screenHeight, "Directional Light Shader - raylib");

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
    
    // Get shader uniform locations
    int lightDirLoc = GetShaderLocation(shader, "lightDirection");
    int lightColorLoc = GetShaderLocation(shader, "lightColor");
    int ambientColorLoc = GetShaderLocation(shader, "ambientColor");
    int viewPosLoc = GetShaderLocation(shader, "viewPos");
    int ambientStrengthLoc = GetShaderLocation(shader, "ambientStrength");
    int specularStrengthLoc = GetShaderLocation(shader, "specularStrength");
    int shininessLoc = GetShaderLocation(shader, "shininess");
    
    // Set up directional light properties
    Vector3 lightDirection = { -0.5f, -1.0f, -0.3f };
    lightDirection = Vector3Normalize(lightDirection);
    Vector3 lightColor = { 1.0f, 1.0f, 1.0f };
    Vector3 ambientColor = { 0.3f, 0.3f, 0.3f };
    float ambientStrength = 0.2f;
    float specularStrength = 0.5f;
    float shininess = 32.0f;
    
    // Set shader values
    SetShaderValue(shader, lightDirLoc, &lightDirection, SHADER_UNIFORM_VEC3);
    SetShaderValue(shader, lightColorLoc, &lightColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(shader, ambientColorLoc, &ambientColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(shader, ambientStrengthLoc, &ambientStrength, SHADER_UNIFORM_FLOAT);
    SetShaderValue(shader, specularStrengthLoc, &specularStrength, SHADER_UNIFORM_FLOAT);
    SetShaderValue(shader, shininessLoc, &shininess, SHADER_UNIFORM_FLOAT);
    
    // Create a model to apply the shader to
    Model model = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    model.materials[0].shader = shader;
    
    // Create a sphere model
    Model sphere = LoadModelFromMesh(GenMeshSphere(0.5f, 32, 32));
    sphere.materials[0].shader = shader;

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        UpdateOrbitalCamera(&camera, &cameraYaw, &cameraPitch, &cameraRadius, &previousMousePos);

        // Update camera position in shader
        SetShaderValue(shader, viewPosLoc, &camera.position, SHADER_UNIFORM_VEC3);

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
        DrawGrid(20, 1.0f);
        
        Vector3 cubePos = { 0.0f, 0.5f, 0.0f };
        DrawModel(model, cubePos, 1.0f, WHITE);
        
        Vector3 sphere1Pos = { -2.5f, 0.5f, 0.0f };
        Vector3 sphere2Pos = { 2.5f, 0.5f, 0.0f };
        DrawModel(sphere, sphere1Pos, 1.0f, RED);
        DrawModel(sphere, sphere2Pos, 1.0f, BLUE);
        
        EndMode3D();

        DrawFPS(10, 40);

        EndDrawing();
    }

    // Unload shader and models
    UnloadShader(shader);
    UnloadModel(model);
    UnloadModel(sphere);

    CloseWindow();

    return 0;
}
