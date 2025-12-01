#include "PerlinNoise.h"
#include <cmath>
#include <algorithm>

// Perlin noise helper functions
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

// PerlinNoise implementation
PerlinNoise::PerlinNoise(unsigned int seed) {
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

float PerlinNoise::Noise(float x, float y) {
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

float PerlinNoise::OctaveNoise(float x, float y, int octaves, float persistence) {
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
