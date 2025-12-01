#pragma once

#include "raylib.h"
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>

using namespace JPH;

// Structure to hold sphere data
struct PhysicsSphere
{
    BodyID bodyID;
    ::Color color;
    float stoppedTimer; // Timer to track how long the sphere has been stopped
    bool markedForDestruction;
    Vector3 lastPosition; // Track position for velocity check
    Vector3 velocity;     // Current velocity of the sphere
    float mass;           // Mass of the sphere
};

// Cell structure for cellular automata
struct DensityCell
{
    float density;      // Material density at this cell
    float viscosity;    // Viscosity of material (resistance to flow)
};

// Voxel cell for particle tracking
struct ParticleVoxelCell
{
    float totalMass;           // Total mass of particles in this cell
    Vector3 averageVelocity;   // Average velocity of particles
    int particleCount;         // Number of particles in this cell
    float momentum;            // Magnitude of momentum (for force calculation)
};

// Helper to check if a sphere has stopped moving
bool IsSphereStationary(const Vector3& currentPos, const Vector3& lastPos, float threshold = 0.01f);
