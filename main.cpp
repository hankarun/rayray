#include "raylib.h"
#include "raymath.h"
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

    InitWindow(screenWidth, screenHeight, "3D Camera and Grid - raylib");

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

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        UpdateOrbitalCamera(&camera, &cameraYaw, &cameraPitch, &cameraRadius, &previousMousePos);

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
        DrawGrid(20, 1.0f);
        DrawCube(Vector3{ 0.0f, 0.5f, 0.0f }, 1.0f, 1.0f, 1.0f, RED);
        DrawCubeWires(Vector3{ 0.0f, 0.5f, 0.0f }, 1.0f, 1.0f, 1.0f, MAROON);
        EndMode3D();

        DrawText("Left mouse: rotate | Mouse wheel: zoom", 10, 10, 20, DARKGRAY);
        DrawFPS(10, 40);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
