#include "Helpers.h"
#include "raymath.h"
#include "rlgl.h"
#include <cmath>

void ModifyHeightmapWithDensity(DensityAutomata& densityGrid, float worldX, float worldZ, 
                                 float radius, float densityAmount, float viscosity, 
                                 float terrainScale, int heightmapSize) {
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

void ModifyHeightmap(std::vector<float>& heightSamples, int heightmapSize, 
                     float worldX, float worldZ, float radius, float heightIncrease, 
                     float terrainScale, float heightScale) {
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
