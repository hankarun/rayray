#pragma once

#include "raylib.h"
#include "DensityAutomata.h"
#include "PerlinNoise.h"
#include <vector>

// Helper function to modify heightmap with density
void ModifyHeightmapWithDensity(DensityAutomata& densityGrid, float worldX, float worldZ, 
                                 float radius, float densityAmount, float viscosity, 
                                 float terrainScale, int heightmapSize);

// Helper function to modify heightmap directly
void ModifyHeightmap(std::vector<float>& heightSamples, int heightmapSize, 
                     float worldX, float worldZ, float radius, float heightIncrease, 
                     float terrainScale, float heightScale);

// Generate Perlin noise heightmap
Image GeneratePerlinNoiseHeightmap(int width, int height, float scale, int octaves, float persistence);

// Update orbital camera based on mouse input
void UpdateOrbitalCamera(Camera3D* camera, float* yaw, float* pitch, float* radius, Vector2* previousMousePos);
