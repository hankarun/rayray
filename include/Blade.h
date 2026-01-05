#pragma once

#include "SceneObject.h"
#include "PhysicsLayers.h"
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

// Blade class - a flat rectangular shape for pushing/digging terrain (originally the "moving cube")
class Blade : public SceneObject
{
public:
    // Drive modes
    enum class DriveMode {
        Linear = 0,
        Circle = 1,
        Manual = 2
    };
    
private:
    // Blade dimensions (half extents)
    float halfExtentX;  // Width
    float halfExtentY;  // Height
    float halfExtentZ;  // Depth (thickness)
    
    // Drive mode settings
    DriveMode driveMode;
    
    // Linear mode settings
    float startX;
    float startZ;
    float minX;
    float maxX;
    
    // Circle mode settings
    float circleCenterX;
    float circleCenterZ;
    float circleRadius;
    float circleAngle;
    
    // Manual mode settings
    Vector3 velocity;
    float acceleration;
    float deceleration;
    float turnSpeed;
    
    // Blade rotation (in addition to movement direction)
    float bladeRotation;  // Extra rotation in degrees
    
    // Height offset
    float heightOffset;
    
    // Resistance coefficient
    float resistanceCoeff;
    
    // Shader for rendering
    Shader* shader;
    
public:
    Blade(float halfX = 1.0f, float halfY = 0.5f, float halfZ = 0.1f);
    ~Blade() override;
    
    void Initialize(JPH::BodyInterface& bodyInterface) override;
    void Update(float deltaTime, JPH::BodyInterface& bodyInterface,
               std::vector<float>& heightSamples, int heightmapSize,
               float terrainScale, float heightScale,
               DensityAutomata& densityGrid,
               ParticleVoxelGrid& particleVoxelGrid) override;
    void Draw() override;
    
    float CalculateTerrainResistance(const std::vector<float>& heightSamples,
                                     int heightmapSize, float terrainScale,
                                     float resistanceCoeff) override;
    
    float DigTerrain(std::vector<float>& heightSamples, int heightmapSize,
                     float terrainScale, float heightScale, float deltaTime,
                     std::vector<Vector3>& dugPositions) override;
    
    // Update physics body position/rotation
    void UpdatePhysicsBody(JPH::BodyInterface& bodyInterface);
    
    // Get effective rotation (drive mode + blade rotation)
    float GetEffectiveRotation() const;
    
    // Get the Jolt quaternion for current rotation
    JPH::Quat GetRotationQuat() const;
    
    // Mode setters
    void SetDriveMode(DriveMode mode);
    DriveMode GetDriveMode() const { return driveMode; }
    
    // Linear mode settings
    void SetLinearSettings(float startX, float startZ, float minX, float maxX);
    
    // Circle mode settings
    void SetCircleSettings(float centerX, float centerZ, float radius);
    float GetCircleAngle() const { return circleAngle; }
    
    // Manual mode input handling
    void HandleManualInput(float deltaTime, bool forward, bool backward, bool turnLeft, bool turnRight);
    
    // GUI settings
    void SetBladeRotation(float degrees) { bladeRotation = degrees; }
    float GetBladeRotation() const { return bladeRotation; }
    void SetHeightOffset(float height) { heightOffset = height; }
    float GetHeightOffset() const { return heightOffset; }
    void SetResistanceCoeff(float coeff) { resistanceCoeff = coeff; }
    float GetResistanceCoeff() const { return resistanceCoeff; }
    void SetShader(Shader* s) { shader = s; }
    
    // Dimension getters
    float GetHalfExtentX() const { return halfExtentX; }
    float GetHalfExtentY() const { return halfExtentY; }
    float GetHalfExtentZ() const { return halfExtentZ; }
    
    // Get velocity for manual mode
    Vector3 GetVelocity() const { return velocity; }
};
