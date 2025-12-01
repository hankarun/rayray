#pragma once

#include "PhysicsStructures.h"
#include <vector>

// Cellular Automata for density simulation
class DensityAutomata
{
private:
    std::vector<DensityCell> cells;
    std::vector<DensityCell> nextCells;
    int gridSize;
    float cellSize;
    
public:
    DensityAutomata(int size, float cellSz);
    
    void AddDensity(int x, int z, float amount, float visc = 0.5f);
    float GetDensity(int x, int z) const;
    
    // Cellular automata update step
    void Update(float deltaTime, std::vector<float>& heightSamples, float heightScale);
    
    int GetGridSize() const;
};
