#pragma once

#include "SceneObject.h"
#include "PhysicsLayers.h"
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

// Excavator Bucket class - a scoop-shaped object for scooping terrain
class ExcavatorBucket : public SceneObject
{
public:
    // Bucket states
    enum class BucketState {
        Idle = 0,       // Not moving
        Lowering = 1,   // Moving down to dig
        Scooping = 2,   // Dragging along ground
        Lifting = 3,    // Lifting up with material
        Dumping = 4,    // Rotating to dump material
        Returning = 5   // Returning to start position
    };
    
private:
    // Bucket dimensions
    float bucketWidth;    // Width of bucket opening
    float bucketDepth;    // How deep the bucket is
    float bucketHeight;   // Height of bucket sides
    float wallThickness;  // Thickness of bucket walls
    
    // Bucket state
    BucketState state;
    float stateTimer;
    
    // Bucket rotation (tilt angle for scooping/dumping)
    float bucketTilt;     // Current tilt angle in radians
    float targetTilt;     // Target tilt angle
    float tiltSpeed;      // How fast to change tilt
    
    // Arm position (for excavator arm simulation)
    float armAngle;       // Angle of arm from vertical
    float armLength;      // Length of arm
    Vector3 pivotPoint;   // Where the arm pivots from
    
    // Target positions for automated operation
    Vector3 digPosition;  // Where to dig
    Vector3 dumpPosition; // Where to dump
    
    // Manual control
    bool manualControl;
    
    // Collected material (tracked for dumping)
    float collectedVolume;
    
    // Shader
    Shader* shader;
    
    // Models for bucket parts (main body + teeth)
    Model bucketBodyModel;
    Model bucketTeethModel;
    bool modelsLoaded;
    
public:
    ExcavatorBucket(float width = 1.5f, float depth = 1.0f, float height = 0.8f);
    ~ExcavatorBucket() override;
    
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
    
    void Cleanup(JPH::BodyInterface& bodyInterface) override;
    
    // State management
    void SetState(BucketState newState);
    BucketState GetState() const { return state; }
    
    // Set operation positions
    void SetDigPosition(Vector3 pos) { digPosition = pos; }
    void SetDumpPosition(Vector3 pos) { dumpPosition = pos; }
    void SetPivotPoint(Vector3 pos) { pivotPoint = pos; }
    
    // Manual control
    void SetManualControl(bool manual) { manualControl = manual; }
    void SetBucketTilt(float tilt) { targetTilt = tilt; }
    void SetArmAngle(float angle) { armAngle = angle; }
    
    // Start automated dig cycle
    void StartDigCycle(Vector3 digPos, Vector3 dumpPos);
    
    // Get collected volume
    float GetCollectedVolume() const { return collectedVolume; }
    
    // Dump the collected material (spawns particles)
    void DumpMaterial(JPH::BodyInterface& bodyInterface, 
                      std::vector<struct PhysicsSphere>& spheres,
                      float sphereRadius, float sphereMass);
    
    // Shader
    void SetShader(Shader* s) { shader = s; }
    
    // Getters for arm simulation
    float GetArmAngle() const { return armAngle; }
    float GetBucketTilt() const { return bucketTilt; }
    Vector3 GetPivotPoint() const { return pivotPoint; }
    
private:
    // Create bucket mesh
    void CreateBucketMesh();
    
    // Update arm/bucket position based on arm angle
    void UpdateArmPosition();
    
    // State update functions
    void UpdateIdleState(float deltaTime);
    void UpdateLoweringState(float deltaTime);
    void UpdateScoopingState(float deltaTime, std::vector<float>& heightSamples,
                             int heightmapSize, float terrainScale, float heightScale);
    void UpdateLiftingState(float deltaTime);
    void UpdateDumpingState(float deltaTime);
    void UpdateReturningState(float deltaTime);
};
