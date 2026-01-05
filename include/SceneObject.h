#pragma once

#include "raylib.h"
#include "raymath.h"
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Math/Quat.h>
#include <vector>

// Forward declarations
class DensityAutomata;
class ParticleVoxelGrid;

// Base class for all scene objects that can interact with terrain
class SceneObject
{
protected:
    Vector3 position;
    float rotationY;          // Rotation around Y axis in radians
    JPH::BodyID bodyID;
    Model model;
    ::Color color;
    
    // Physics properties
    float mass;
    float enginePower;
    float currentSpeed;
    float maxSpeed;
    
    // Terrain interaction
    bool canDigTerrain;
    float digDepth;
    
public:
    SceneObject();
    virtual ~SceneObject();
    
    // Initialize the object with physics
    virtual void Initialize(JPH::BodyInterface& bodyInterface) = 0;
    
    // Update object state each frame
    virtual void Update(float deltaTime, JPH::BodyInterface& bodyInterface,
                       std::vector<float>& heightSamples, int heightmapSize,
                       float terrainScale, float heightScale,
                       DensityAutomata& densityGrid,
                       ParticleVoxelGrid& particleVoxelGrid) = 0;
    
    // Render the object
    virtual void Draw() = 0;
    
    // Calculate terrain resistance force
    virtual float CalculateTerrainResistance(const std::vector<float>& heightSamples,
                                             int heightmapSize, float terrainScale,
                                             float resistanceCoeff) = 0;
    
    // Dig terrain and return displaced volume
    virtual float DigTerrain(std::vector<float>& heightSamples, int heightmapSize,
                             float terrainScale, float heightScale, float deltaTime,
                             std::vector<Vector3>& dugPositions) = 0;
    
    // Getters
    Vector3 GetPosition() const { return position; }
    float GetRotationY() const { return rotationY; }
    float GetCurrentSpeed() const { return currentSpeed; }
    float GetEnginePower() const { return enginePower; }
    JPH::BodyID GetBodyID() const { return bodyID; }
    
    // Setters
    void SetPosition(Vector3 pos) { position = pos; }
    void SetRotationY(float rot) { rotationY = rot; }
    void SetEnginePower(float power) { enginePower = power; }
    void SetMaxSpeed(float speed) { maxSpeed = speed; }
    
    // Cleanup physics body
    virtual void Cleanup(JPH::BodyInterface& bodyInterface);
};
