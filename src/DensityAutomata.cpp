#include "DensityAutomata.h"
#include <cmath>

DensityAutomata::DensityAutomata(int size, float cellSz) : gridSize(size), cellSize(cellSz) {
    cells.resize(size * size);
    nextCells.resize(size * size);
    
    // Initialize all cells
    for (int i = 0; i < size * size; i++) {
        cells[i].density = 0.0f;
        cells[i].viscosity = 0.5f; // Default viscosity
    }
}

void DensityAutomata::AddDensity(int x, int z, float amount, float visc) {
    if (x >= 0 && x < gridSize && z >= 0 && z < gridSize) {
        int idx = z * gridSize + x;
        cells[idx].density += amount;
        cells[idx].viscosity = visc;
    }
}

float DensityAutomata::GetDensity(int x, int z) const {
    if (x >= 0 && x < gridSize && z >= 0 && z < gridSize) {
        return cells[z * gridSize + x].density;
    }
    return 0.0f;
}

// Cellular automata update step
void DensityAutomata::Update(float deltaTime, std::vector<float>& heightSamples, float heightScale) {
    // Copy current state to next state
    nextCells = cells;
    
    // Flow simulation based on density, viscosity, and height differences (like dirt)
    for (int z = 0; z < gridSize; z++) {
        for (int x = 0; x < gridSize; x++) {
            int idx = z * gridSize + x;
            DensityCell& cell = cells[idx];
            
            if (cell.density <= 0.01f) continue; // Skip empty cells
            
            // Calculate flow to neighbors based on density difference and viscosity
            float flowRate = 1.5f * (1.0f - cell.viscosity) * deltaTime; // Increased flow rate
            int neighbors[4][2] = {{-1,0}, {1,0}, {0,-1}, {0,1}};
            
            float currentHeight = heightSamples[idx];
            
            for (int n = 0; n < 4; n++) {
                int nx = x + neighbors[n][0];
                int nz = z + neighbors[n][1];
                
                if (nx >= 0 && nx < gridSize && nz >= 0 && nz < gridSize) {
                    int nidx = nz * gridSize + nx;
                    float neighborHeight = heightSamples[nidx];
                    
                    // Dirt flows based on both density and height differences (angle of repose)
                    float densityDiff = cell.density - cells[nidx].density;
                    float heightDiff = currentHeight - neighborHeight;
                    
                    // Simulate angle of repose - dirt flows down steep slopes
                    const float angleOfRepose = 0.3f; // About 35 degrees in height units
                    
                    if (densityDiff > 0.0f || heightDiff > angleOfRepose) {
                        float flow = (densityDiff * 0.3f + heightDiff * 0.7f) * flowRate * 0.25f;
                        flow = fmaxf(0.0f, flow); // Only positive flow
                        flow = fminf(flow, cell.density * 0.3f); // Don't flow more than 30% per neighbor
                        
                        nextCells[idx].density -= flow;
                        nextCells[nidx].density += flow;
                    }
                }
            }
            
            // Apply density to height (higher density = higher terrain)
            if (nextCells[idx].density > 0.1f) {
                float heightIncrease = nextCells[idx].density * 0.02f * deltaTime; // Faster settling
                heightSamples[idx] += heightIncrease;
                if (heightSamples[idx] > heightScale) {
                    heightSamples[idx] = heightScale;
                }
                
                // Reduce density as it converts to height
                nextCells[idx].density *= 0.95f; // Faster conversion
            }
        }
    }
    
    // Swap buffers
    cells = nextCells;
}

int DensityAutomata::GetGridSize() const {
    return gridSize;
}
