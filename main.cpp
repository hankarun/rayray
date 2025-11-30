#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <cmath>
#include <vector>

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

Image GeneratePerlinNoiseHeightmap(int width, int height, float scale, int octaves, float persistence) {
    PerlinNoise perlin;
    Image image = GenImageColor(width, height, BLACK);
    Color* pixels = LoadImageColors(image);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float nx = (float)x / width * scale;
            float ny = (float)y / height * scale;
            
            float noiseValue = perlin.OctaveNoise(nx, ny, octaves, persistence);
            // Normalize to 0-1 range
            noiseValue = (noiseValue + 1.0f) / 2.0f;
            
            unsigned char value = (unsigned char)(noiseValue * 255);
            pixels[y * width + x] = Color{ value, value, value, 255 };
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
    UnloadImage(heightmapImage);
    
    // Create plane mesh with high resolution for smooth heightmap displacement
    Mesh planeMesh = GenMeshPlane(20.0f, 20.0f, 128, 128);
    Model planeModel = LoadModelFromMesh(planeMesh);
    planeModel.materials[0].shader = heightmapShader;
    planeModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = heightmapTexture;
    
    // Create a model to apply the shader to
    Model model = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    model.materials[0].shader = shader;
    
    // Create a sphere model
    Model sphere = LoadModelFromMesh(GenMeshSphere(0.5f, 32, 32));
    sphere.materials[0].shader = shader;

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        UpdateOrbitalCamera(&camera, &cameraYaw, &cameraPitch, &cameraRadius, &previousMousePos);

        // Rotate light direction around a circle
        lightAngle += 0.5f * GetFrameTime();
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
        ClearBackground(GRAY);

        BeginMode3D(camera);
        DrawGrid(20, 1.0f);
        
        // Draw the heightmap plane at the origin
        DrawModel(planeModel, Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, WHITE);
        
        Vector3 cubePos = { 0.0f, 3.0f, 0.0f };
        DrawModel(model, cubePos, 1.0f, WHITE);
        
        Vector3 sphere1Pos = { -2.5f, 3.0f, 0.0f };
        Vector3 sphere2Pos = { 2.5f, 3.0f, 0.0f };
        DrawModel(sphere, sphere1Pos, 1.0f, RED);
        DrawModel(sphere, sphere2Pos, 1.0f, BLUE);
        
        EndMode3D();

        DrawFPS(10, 40);

        EndDrawing();
    }

    // Unload shader and models
    UnloadShader(shader);
    UnloadShader(heightmapShader);
    UnloadTexture(heightmapTexture);
    UnloadModel(model);
    UnloadModel(sphere);
    UnloadModel(planeModel);

    CloseWindow();

    return 0;
}
