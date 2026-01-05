#include "ExcavatorBucket.h"
#include "DensityAutomata.h"
#include "ParticleVoxelGrid.h"
#include "PhysicsStructures.h"
#include "Helpers.h"
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <cmath>

ExcavatorBucket::ExcavatorBucket(float width, float depth, float height)
    : bucketWidth(width)
    , bucketDepth(depth)
    , bucketHeight(height)
    , wallThickness(0.1f)
    , state(BucketState::Idle)
    , stateTimer(0.0f)
    , bucketTilt(0.0f)
    , targetTilt(0.0f)
    , tiltSpeed(2.0f)
    , armAngle(0.0f)
    , armLength(3.0f)
    , pivotPoint({0.0f, 3.0f, 0.0f})
    , digPosition({0.0f, 0.0f, 0.0f})
    , dumpPosition({5.0f, 2.0f, 0.0f})
    , manualControl(true)
    , collectedVolume(0.0f)
    , shader(nullptr)
    , modelsLoaded(false)
{
    position = {0.0f, 2.0f, 0.0f};
    rotationY = 0.0f;
    mass = 200.0f;
    enginePower = 800.0f;
    maxSpeed = 2.0f;
    currentSpeed = 0.0f;
    digDepth = 0.5f;
    canDigTerrain = true;
    color = ::Color{255, 161, 0, 255};  // ORANGE
}

ExcavatorBucket::~ExcavatorBucket()
{
    if (modelsLoaded) {
        UnloadModel(bucketBodyModel);
        // bucketTeethModel shares mesh, so don't unload twice
    }
}

void ExcavatorBucket::CreateBucketMesh()
{
    // Create a simple bucket shape using a box with an open top
    // The bucket is essentially a box that's been made into a scoop shape
    
    // Main body - a box that represents the bucket body
    Mesh bodyMesh = GenMeshCube(bucketWidth, bucketHeight, bucketDepth);
    bucketBodyModel = LoadModelFromMesh(bodyMesh);
    
    if (shader) {
        bucketBodyModel.materials[0].shader = *shader;
    }
    
    // Create teeth mesh (small boxes along the front edge)
    // For simplicity, we'll use small cubes
    float toothWidth = bucketWidth / 5.0f;
    float toothHeight = 0.15f;
    float toothDepth = 0.1f;
    
    Mesh teethMesh = GenMeshCube(toothWidth * 0.8f, toothHeight, toothDepth);
    bucketTeethModel = LoadModelFromMesh(teethMesh);
    
    if (shader) {
        bucketTeethModel.materials[0].shader = *shader;
    }
    
    modelsLoaded = true;
}

void ExcavatorBucket::Initialize(BodyInterface& bodyInterface)
{
    CreateBucketMesh();
    
    // Create a box shape for collision
    BoxShapeSettings shapeSettings(Vec3(bucketWidth / 2.0f, bucketHeight / 2.0f, bucketDepth / 2.0f));
    ShapeSettings::ShapeResult shapeResult = shapeSettings.Create();
    ShapeRefC shape = shapeResult.Get();
    
    BodyCreationSettings bodySettings(shape,
        RVec3(position.x, position.y, position.z),
        Quat::sRotation(Vec3(1, 0, 0), bucketTilt) * Quat::sRotation(Vec3(0, 1, 0), rotationY),
        EMotionType::Kinematic,
        Layers::MOVING);
    bodySettings.mFriction = 0.8f;
    bodySettings.mRestitution = 0.1f;
    
    Body* body = bodyInterface.CreateBody(bodySettings);
    bodyID = body->GetID();
    bodyInterface.AddBody(bodyID, EActivation::Activate);
}

void ExcavatorBucket::Update(float deltaTime, BodyInterface& bodyInterface,
                             std::vector<float>& heightSamples, int heightmapSize,
                             float terrainScale, float heightScale,
                             DensityAutomata& densityGrid,
                             ParticleVoxelGrid& particleVoxelGrid)
{
    // Update tilt towards target
    if (bucketTilt < targetTilt) {
        bucketTilt += tiltSpeed * deltaTime;
        if (bucketTilt > targetTilt) bucketTilt = targetTilt;
    } else if (bucketTilt > targetTilt) {
        bucketTilt -= tiltSpeed * deltaTime;
        if (bucketTilt < targetTilt) bucketTilt = targetTilt;
    }
    
    // Update arm position
    UpdateArmPosition();
    
    // Update state machine
    stateTimer += deltaTime;
    
    switch (state) {
        case BucketState::Idle:
            UpdateIdleState(deltaTime);
            break;
        case BucketState::Lowering:
            UpdateLoweringState(deltaTime);
            break;
        case BucketState::Scooping:
            UpdateScoopingState(deltaTime, heightSamples, heightmapSize, terrainScale, heightScale);
            break;
        case BucketState::Lifting:
            UpdateLiftingState(deltaTime);
            break;
        case BucketState::Dumping:
            UpdateDumpingState(deltaTime);
            break;
        case BucketState::Returning:
            UpdateReturningState(deltaTime);
            break;
    }
    
    // Update physics body
    Quat rotation = Quat::sRotation(Vec3(1, 0, 0), bucketTilt) * Quat::sRotation(Vec3(0, 1, 0), rotationY);
    bodyInterface.SetPositionAndRotation(bodyID,
        RVec3(position.x, position.y, position.z),
        rotation,
        EActivation::Activate);
}

void ExcavatorBucket::UpdateArmPosition()
{
    // Calculate bucket position based on arm angle and pivot point
    // Arm rotates down from pivot point
    float armRad = armAngle * 3.14159f / 180.0f;
    
    position.x = pivotPoint.x + armLength * sinf(armRad);
    position.y = pivotPoint.y - armLength * cosf(armRad);
}

void ExcavatorBucket::UpdateIdleState(float deltaTime)
{
    // Do nothing in idle state, wait for commands
}

void ExcavatorBucket::UpdateLoweringState(float deltaTime)
{
    // Move arm down towards dig position
    float targetArmAngle = 60.0f;  // Arm angle when at dig depth
    
    if (armAngle < targetArmAngle) {
        armAngle += 30.0f * deltaTime;  // 30 degrees per second
        if (armAngle >= targetArmAngle) {
            armAngle = targetArmAngle;
            SetState(BucketState::Scooping);
        }
    }
    
    // Tilt bucket forward for scooping
    targetTilt = 0.5f;  // Tilted forward
}

void ExcavatorBucket::UpdateScoopingState(float deltaTime, std::vector<float>& heightSamples,
                                          int heightmapSize, float terrainScale, float heightScale)
{
    // Move forward while digging
    position.x += currentSpeed * deltaTime;
    currentSpeed = 1.0f;  // Slow scooping speed
    
    // Dig terrain
    std::vector<Vector3> dugPositions;
    float displaced = DigTerrain(heightSamples, heightmapSize, terrainScale, heightScale, 
                                  deltaTime, dugPositions);
    collectedVolume += displaced;
    
    // After scooping for a while or collected enough, start lifting
    if (stateTimer > 2.0f || collectedVolume > 0.5f) {
        SetState(BucketState::Lifting);
    }
}

void ExcavatorBucket::UpdateLiftingState(float deltaTime)
{
    // Raise arm
    float targetArmAngle = 0.0f;  // Arm straight up
    
    if (armAngle > targetArmAngle) {
        armAngle -= 40.0f * deltaTime;
        if (armAngle <= targetArmAngle) {
            armAngle = targetArmAngle;
            SetState(BucketState::Dumping);
        }
    }
    
    // Level bucket to hold material
    targetTilt = 0.0f;
}

void ExcavatorBucket::UpdateDumpingState(float deltaTime)
{
    // Tilt bucket to dump contents
    targetTilt = -1.2f;  // Tilted backward to dump
    
    // After dumping for a while
    if (stateTimer > 1.5f) {
        collectedVolume = 0.0f;  // Material dumped
        SetState(BucketState::Returning);
    }
}

void ExcavatorBucket::UpdateReturningState(float deltaTime)
{
    // Return bucket to starting position
    targetTilt = 0.0f;
    
    // Move back
    position.x -= currentSpeed * deltaTime * 0.5f;
    
    if (stateTimer > 2.0f) {
        SetState(BucketState::Idle);
    }
}

void ExcavatorBucket::Draw()
{
    // Draw the bucket body
    // Position is adjusted for bucket center
    Vector3 bucketCenter = position;
    bucketCenter.y -= bucketHeight / 2.0f;
    
    // Draw main body with rotation
    DrawModelEx(bucketBodyModel, position, 
                Vector3{1.0f, 0.0f, 0.0f}, bucketTilt * 180.0f / 3.14159f,
                Vector3{1.0f, 1.0f, 1.0f}, color);
    
    // Draw teeth along the front edge
    float teethY = -bucketHeight / 2.0f - 0.075f;
    float teethZ = bucketDepth / 2.0f;
    int numTeeth = 5;
    float toothSpacing = bucketWidth / (numTeeth);
    
    // Calculate rotated teeth positions
    float cosTilt = cosf(bucketTilt);
    float sinTilt = sinf(bucketTilt);
    
    for (int i = 0; i < numTeeth; i++) {
        float localTeethX = -bucketWidth / 2.0f + toothSpacing * (i + 0.5f);
        
        // Rotate by tilt around X axis
        float rotatedY = teethY * cosTilt - teethZ * sinTilt;
        float rotatedZ = teethY * sinTilt + teethZ * cosTilt;
        
        Vector3 toothPos = {
            position.x + localTeethX,
            position.y + rotatedY,
            position.z + rotatedZ
        };
        DrawModel(bucketTeethModel, toothPos, 1.0f, ::Color{80, 80, 80, 255});  // DARKGRAY
    }
    
    // Draw arm (simple line from pivot to bucket)
    DrawLine3D(pivotPoint, position, ::Color{130, 130, 130, 255});  // GRAY
    DrawSphere(pivotPoint, 0.1f, ::Color{80, 80, 80, 255});  // DARKGRAY
    
    // Draw collected volume indicator
    if (collectedVolume > 0.01f) {
        // Draw brown material inside bucket
        Vector3 materialPos = position;
        materialPos.y -= bucketHeight * 0.3f;
        float materialScale = fminf(collectedVolume * 2.0f, 0.8f);
        
        // Rotate material position by bucket tilt
        float localY = -bucketHeight * 0.3f;
        float rotatedMatY = localY * cosTilt;
        float rotatedMatZ = localY * sinTilt;
        
        Vector3 matPos = {
            position.x,
            position.y + rotatedMatY,
            position.z + rotatedMatZ
        };
        
        DrawCube(matPos, bucketWidth * 0.8f * materialScale, 
                 bucketHeight * 0.5f * materialScale, bucketDepth * 0.8f * materialScale, 
                 ::Color{139, 69, 19, 255});  // BROWN
    }
}

float ExcavatorBucket::CalculateTerrainResistance(const std::vector<float>& heightSamples,
                                                   int heightmapSize, float terrainScale,
                                                   float resistanceCoeff)
{
    // Similar to blade, but for bucket shape
    if (position.x < -10.0f || position.x > 10.0f ||
        position.z < -10.0f || position.z > 10.0f) {
        return 0.0f;
    }
    
    float bucketBottomY = position.y - bucketHeight;
    
    int hmX = (int)((position.x + 10.0f) / (20.0f / heightmapSize));
    int hmZ = (int)((position.z + 10.0f) / (20.0f / heightmapSize));
    
    if (hmX >= 0 && hmX < heightmapSize && hmZ >= 0 && hmZ < heightmapSize) {
        float terrainHeight = heightSamples[hmZ * heightmapSize + hmX];
        if (bucketBottomY < terrainHeight) {
            float penetration = terrainHeight - bucketBottomY;
            return resistanceCoeff * penetration * bucketWidth * bucketDepth * 100.0f;
        }
    }
    
    return 0.0f;
}

float ExcavatorBucket::DigTerrain(std::vector<float>& heightSamples, int heightmapSize,
                                   float terrainScale, float heightScale, float deltaTime,
                                   std::vector<Vector3>& dugPositions)
{
    float bucketBottomY = position.y - bucketHeight;
    
    if (position.x < -10.0f || position.x > 10.0f ||
        position.z < -10.0f || position.z > 10.0f ||
        bucketBottomY >= heightScale) {
        return 0.0f;
    }
    
    // Calculate bucket footprint
    float halfWidth = bucketWidth / 2.0f;
    float halfDepth = bucketDepth / 2.0f;
    
    int hmMinX = (int)((position.x - halfWidth + 10.0f) / (20.0f / heightmapSize));
    int hmMaxX = (int)((position.x + halfWidth + 10.0f) / (20.0f / heightmapSize));
    int hmMinZ = (int)((position.z - halfDepth + 10.0f) / (20.0f / heightmapSize));
    int hmMaxZ = (int)((position.z + halfDepth + 10.0f) / (20.0f / heightmapSize));
    
    float totalDisplacedVolume = 0.0f;
    
    for (int z = hmMinZ; z <= hmMaxZ; z++) {
        for (int x = hmMinX; x <= hmMaxX; x++) {
            if (x >= 0 && x < heightmapSize && z >= 0 && z < heightmapSize) {
                float cellWorldX = (float)x * (20.0f / heightmapSize) - 10.0f;
                float cellWorldZ = (float)z * (20.0f / heightmapSize) - 10.0f;
                
                int idx = z * heightmapSize + x;
                float currentHeight = heightSamples[idx];
                
                if (bucketBottomY < currentHeight && currentHeight > 0.0f) {
                    float digAmount = digDepth * deltaTime * 3.0f;
                    float actualDig = fminf(digAmount, currentHeight - bucketBottomY);
                    actualDig = fminf(actualDig, currentHeight);
                    
                    if (actualDig > 0.0f) {
                        float heightBeforeDig = currentHeight;
                        heightSamples[idx] -= actualDig;
                        if (heightSamples[idx] < 0.0f) heightSamples[idx] = 0.0f;
                        
                        dugPositions.push_back({cellWorldX, heightBeforeDig, cellWorldZ});
                        
                        float cellArea = terrainScale * terrainScale;
                        totalDisplacedVolume += actualDig * cellArea;
                    }
                }
            }
        }
    }
    
    return totalDisplacedVolume;
}

void ExcavatorBucket::Cleanup(BodyInterface& bodyInterface)
{
    SceneObject::Cleanup(bodyInterface);
    
    if (modelsLoaded) {
        UnloadModel(bucketBodyModel);
        modelsLoaded = false;
    }
}

void ExcavatorBucket::SetState(BucketState newState)
{
    state = newState;
    stateTimer = 0.0f;
}

void ExcavatorBucket::StartDigCycle(Vector3 digPos, Vector3 dumpPos)
{
    digPosition = digPos;
    dumpPosition = dumpPos;
    SetState(BucketState::Lowering);
}

void ExcavatorBucket::DumpMaterial(BodyInterface& bodyInterface,
                                    std::vector<PhysicsSphere>& spheres,
                                    float sphereRadius, float sphereMass)
{
    if (collectedVolume < 0.01f) return;
    
    // Calculate number of spheres to spawn based on collected volume
    float sphereVolume = (4.0f / 3.0f) * 3.14159f * sphereRadius * sphereRadius * sphereRadius;
    int numSpheres = (int)(collectedVolume / sphereVolume);
    numSpheres = fminf(numSpheres, 20);  // Limit to prevent overflow
    
    for (int i = 0; i < numSpheres; i++) {
        // Spawn position relative to bucket
        float offsetX = ((float)GetRandomValue(-50, 50) / 100.0f) * bucketWidth * 0.4f;
        float offsetZ = ((float)GetRandomValue(-50, 50) / 100.0f) * bucketDepth * 0.4f;
        
        Vector3 spawnPos = {
            position.x + offsetX,
            position.y - bucketHeight * 0.3f,
            position.z + offsetZ
        };
        
        // Create sphere
        SphereShapeSettings sphereSettings(sphereRadius);
        ShapeSettings::ShapeResult sphereResult = sphereSettings.Create();
        ShapeRefC sphereShape = sphereResult.Get();
        
        BodyCreationSettings sphereBodySettings(sphereShape,
            RVec3(spawnPos.x, spawnPos.y, spawnPos.z),
            Quat::sIdentity(),
            EMotionType::Dynamic,
            Layers::MOVING);
        
        sphereBodySettings.mFriction = 50.0f;
        sphereBodySettings.mRestitution = 0.05f;
        sphereBodySettings.mLinearDamping = 2.0f;
        sphereBodySettings.mAngularDamping = 5.0f;
        
        Body* sphereBody = bodyInterface.CreateBody(sphereBodySettings);
        if (sphereBody != nullptr) {
            BodyID sphereID = sphereBody->GetID();
            bodyInterface.AddBody(sphereID, EActivation::Activate);
            
            // Give initial velocity based on bucket tilt
            Vec3 initialVel(0.0f, -1.0f, cosf(bucketTilt) * 2.0f);
            bodyInterface.SetLinearVelocity(sphereID, initialVel);
            
            ::Color sphereColor = {139, 90, 43, 255};  // Brown
            Vector3 velocity = {initialVel.GetX(), initialVel.GetY(), initialVel.GetZ()};
            spheres.push_back({sphereID, sphereColor, 0.0f, false, spawnPos, velocity, sphereMass});
        }
    }
    
    collectedVolume = 0.0f;
}
