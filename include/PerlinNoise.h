#pragma once

#include <vector>

// Perlin noise helper functions
float PerlinFade(float t);
float PerlinLerp(float t, float a, float b);
float PerlinGrad(int hash, float x, float y);

// Perlin noise implementation
class PerlinNoise {
private:
    std::vector<int> p;

public:
    PerlinNoise(unsigned int seed = 237);
    float Noise(float x, float y);
    float OctaveNoise(float x, float y, int octaves, float persistence);
};
