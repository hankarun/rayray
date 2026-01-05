#include "Blade.h"
#include "DensityAutomata.h"
#include "ParticleVoxelGrid.h"
#include "Helpers.h"
#include <cmath>

using namespace JPH;

Blade::Blade(float halfX, float halfY, float halfZ)
    : halfExtentX(halfX)
    , halfExtentY(halfY)
    , halfExtentZ(halfZ)
    , driveMode(DriveMode::Linear)
    , startX(-12.0f)
    , startZ(0.0f)
    , minX(-12.0f)
    , maxX(12.0f)
    , circleCenterX(0.0f)
    , circleCenterZ(0.0f)
    , circleRadius(5.0f)
    , circleAngle(0.0f)
    , velocity({0.0f, 0.0f, 0.0f})
    , acceleration(10.0f)
    , deceleration(5.0f)
    , turnSpeed(2.0f)
    , bladeRotation(45.0f)
    , heightOffset(0.0f)
    , resistanceCoeff(50.0f)
    , shader(nullptr)
{
    position = {startX, halfY + heightOffset, startZ};
    rotationY = bladeRotation * 3.14159f / 180.0f;
    maxSpeed = 3.0f;
    currentSpeed = maxSpeed;
    enginePower = 500.0f;
    mass = 100.0f;
    digDepth = 0.3f;
    canDigTerrain = true;
    color = ::Color{0, 121, 241, 255};  // BLUE
}

Blade::~Blade()
{
    if (model.meshCount > 0) {
        UnloadModel(model);
    }
}

void Blade::Initialize(BodyInterface& bodyInterface)
{
    // Create the visual model
    model = LoadModelFromMesh(GenMeshCube(halfExtentX * 2.0f, halfExtentY * 2.0f, halfExtentZ * 2.0f));
    if (shader) {
        model.materials[0].shader = *shader;
    }
    
    // Create physics body
    BoxShapeSettings shapeSettings(Vec3(halfExtentX, halfExtentY, halfExtentZ));
    ShapeSettings::ShapeResult shapeResult = shapeSettings.Create();
    ShapeRefC shape = shapeResult.Get();
    
    Quat rotation = GetRotationQuat();
    
    BodyCreationSettings bodySettings(shape,
        RVec3(position.x, position.y, position.z),
        rotation,
        EMotionType::Kinematic,
        Layers::MOVING);
    bodySettings.mFriction = 0.5f;
    bodySettings.mRestitution = 0.1f;
    
    Body* body = bodyInterface.CreateBody(bodySettings);
    bodyID = body->GetID();
    bodyInterface.AddBody(bodyID, EActivation::Activate);
}

void Blade::Update(float deltaTime, BodyInterface& bodyInterface,
                   std::vector<float>& heightSamples, int heightmapSize,
                   float terrainScale, float heightScale,
                   DensityAutomata& densityGrid,
                   ParticleVoxelGrid& particleVoxelGrid)
{
    // Calculate terrain resistance
    float terrainResistance = CalculateTerrainResistance(heightSamples, heightmapSize, 
                                                          terrainScale, resistanceCoeff);
    
    // Physics-based speed calculation
    float netForce = enginePower - terrainResistance;
    float accel = netForce / mass;
    currentSpeed += accel * deltaTime;
    currentSpeed = Clamp(currentSpeed, 0.01f, maxSpeed);
    
    // If resistance exceeds engine power, apply stronger braking
    if (terrainResistance > enginePower) {
        float overloadRatio = terrainResistance / enginePower;
        currentSpeed *= (1.0f - 0.1f * fminf(overloadRatio - 1.0f, 5.0f) * deltaTime * 60.0f);
    }
    
    // Update position based on drive mode
    switch (driveMode) {
        case DriveMode::Linear:
            position.x += currentSpeed * deltaTime;
            if (position.x > maxX) {
                position.x = startX;
                position.z = startZ;
                currentSpeed = maxSpeed;
            }
            rotationY = bladeRotation * 3.14159f / 180.0f;
            break;
        case DriveMode::Circle:
            {
                float angularSpeed = currentSpeed / circleRadius;
                circleAngle += angularSpeed * deltaTime;
                if (circleAngle > 2.0f * 3.14159f) {
                    circleAngle -= 2.0f * 3.14159f;
                }
                position.x = circleCenterX + circleRadius * cosf(circleAngle);
                position.z = circleCenterZ + circleRadius * sinf(circleAngle);
                float tangentAngle = circleAngle + 3.14159f / 2.0f;
                rotationY = tangentAngle + bladeRotation * 3.14159f / 180.0f;
            }
            break;
        case DriveMode::Manual:
            {
                // Apply terrain resistance to velocity
                float speed = sqrtf(velocity.x * velocity.x + velocity.z * velocity.z);
                if (speed > 0.01f && terrainResistance > 0.0f) {
                    float resistanceDecel = (terrainResistance / mass) * deltaTime;
                    float newSpeed = fmaxf(0.0f, speed - resistanceDecel);
                    velocity.x *= newSpeed / speed;
                    velocity.z *= newSpeed / speed;
                }
                // Clamp max speed
                speed = sqrtf(velocity.x * velocity.x + velocity.z * velocity.z);
                if (speed > maxSpeed) {
                    velocity.x *= maxSpeed / speed;
                    velocity.z *= maxSpeed / speed;
                }
                // Update position
                position.x += velocity.x * deltaTime;
                position.z += velocity.z * deltaTime;
                // Update current speed for display
                currentSpeed = sqrtf(velocity.x * velocity.x + velocity.z * velocity.z);
            }
            break;
    }
    
    // Set Y position from height offset
    position.y = halfExtentY + heightOffset;
    
    // Update physics body
    UpdatePhysicsBody(bodyInterface);
}

void Blade::HandleManualInput(float deltaTime, bool forward, bool backward, 
                              bool turnLeft, bool turnRight)
{
    // Rotation with turn inputs
    if (turnLeft) {
        rotationY += turnSpeed * deltaTime;
    }
    if (turnRight) {
        rotationY -= turnSpeed * deltaTime;
    }
    
    // Forward/backward acceleration
    float forwardInput = 0.0f;
    if (forward) {
        forwardInput = 1.0f;
    }
    if (backward) {
        forwardInput = -0.5f;  // Slower reverse
    }
    
    // Calculate forward direction based on rotation
    float forwardX = cosf(rotationY - bladeRotation * 3.14159f / 180.0f);
    float forwardZ = -sinf(rotationY - bladeRotation * 3.14159f / 180.0f);
    
    if (forwardInput != 0.0f) {
        velocity.x += forwardX * forwardInput * acceleration * deltaTime;
        velocity.z += forwardZ * forwardInput * acceleration * deltaTime;
    } else {
        // Decelerate when no input
        float speed = sqrtf(velocity.x * velocity.x + velocity.z * velocity.z);
        if (speed > 0.01f) {
            float decel = deceleration * deltaTime;
            float newSpeed = fmaxf(0.0f, speed - decel);
            velocity.x *= newSpeed / speed;
            velocity.z *= newSpeed / speed;
        } else {
            velocity.x = 0.0f;
            velocity.z = 0.0f;
        }
    }
}

void Blade::Draw()
{
    float visualRotation = GetEffectiveRotation() * 180.0f / 3.14159f;
    DrawModelEx(model, position, Vector3{0.0f, 1.0f, 0.0f}, visualRotation, 
                Vector3{1.0f, 1.0f, 1.0f}, color);
}

float Blade::CalculateTerrainResistance(const std::vector<float>& heightSamples,
                                        int heightmapSize, float terrainScale,
                                        float resistCoeff)
{
    if (position.x < -10.0f || position.x > 10.0f ||
        position.z < -10.0f || position.z > 10.0f) {
        return 0.0f;
    }
    
    float cubeBottomY = heightOffset;
    float rotationRad = GetEffectiveRotation();
    float cosR = cosf(rotationRad);
    float sinR = sinf(rotationRad);
    
    // Calculate rotated corners
    float corners[4][2] = {
        { halfExtentX * cosR - halfExtentZ * sinR,  halfExtentX * sinR + halfExtentZ * cosR},
        {-halfExtentX * cosR - halfExtentZ * sinR, -halfExtentX * sinR + halfExtentZ * cosR},
        { halfExtentX * cosR + halfExtentZ * sinR,  halfExtentX * sinR - halfExtentZ * cosR},
        {-halfExtentX * cosR + halfExtentZ * sinR, -halfExtentX * sinR - halfExtentZ * cosR}
    };
    
    float minX = corners[0][0], maxX = corners[0][0];
    float minZ = corners[0][1], maxZ = corners[0][1];
    for (int i = 1; i < 4; i++) {
        if (corners[i][0] < minX) minX = corners[i][0];
        if (corners[i][0] > maxX) maxX = corners[i][0];
        if (corners[i][1] < minZ) minZ = corners[i][1];
        if (corners[i][1] > maxZ) maxZ = corners[i][1];
    }
    
    int hmMinX = (int)((position.x + minX + 10.0f) / (20.0f / heightmapSize));
    int hmMaxX = (int)((position.x + maxX + 10.0f) / (20.0f / heightmapSize));
    int hmMinZ = (int)((position.z + minZ + 10.0f) / (20.0f / heightmapSize));
    int hmMaxZ = (int)((position.z + maxZ + 10.0f) / (20.0f / heightmapSize));
    
    float totalPenetration = 0.0f;
    for (int z = hmMinZ; z <= hmMaxZ; z++) {
        for (int x = hmMinX; x <= hmMaxX; x++) {
            if (x >= 0 && x < heightmapSize && z >= 0 && z < heightmapSize) {
                float cellWorldX = (float)x * (20.0f / heightmapSize) - 10.0f;
                float cellWorldZ = (float)z * (20.0f / heightmapSize) - 10.0f;
                
                float localX = (cellWorldX - position.x) * cosR + (cellWorldZ - position.z) * sinR;
                float localZ = -(cellWorldX - position.x) * sinR + (cellWorldZ - position.z) * cosR;
                
                if (fabsf(localX) > halfExtentX || fabsf(localZ) > halfExtentZ) {
                    continue;
                }
                
                float terrainHeight = heightSamples[z * heightmapSize + x];
                if (cubeBottomY < terrainHeight) {
                    totalPenetration += terrainHeight - cubeBottomY;
                }
            }
        }
    }
    
    float cellArea = terrainScale * terrainScale;
    float penetrationVolume = totalPenetration * cellArea;
    return resistCoeff * penetrationVolume * 1000.0f * (1.0f + currentSpeed * currentSpeed);
}

float Blade::DigTerrain(std::vector<float>& heightSamples, int heightmapSize,
                        float terrainScale, float heightScale, float deltaTime,
                        std::vector<Vector3>& dugPositions)
{
    float cubeBottomY = position.y - halfExtentY;
    
    if (position.x < -10.0f || position.x > 10.0f ||
        position.z < -10.0f || position.z > 10.0f ||
        cubeBottomY >= heightScale) {
        return 0.0f;
    }
    
    float rotationRad = GetEffectiveRotation();
    float cosR = cosf(rotationRad);
    float sinR = sinf(rotationRad);
    
    // Calculate rotated corners
    float corners[4][2] = {
        { halfExtentX * cosR - halfExtentZ * sinR,  halfExtentX * sinR + halfExtentZ * cosR},
        {-halfExtentX * cosR - halfExtentZ * sinR, -halfExtentX * sinR + halfExtentZ * cosR},
        { halfExtentX * cosR + halfExtentZ * sinR,  halfExtentX * sinR - halfExtentZ * cosR},
        {-halfExtentX * cosR + halfExtentZ * sinR, -halfExtentX * sinR - halfExtentZ * cosR}
    };
    
    float minX = corners[0][0], maxX = corners[0][0];
    float minZ = corners[0][1], maxZ = corners[0][1];
    for (int i = 1; i < 4; i++) {
        if (corners[i][0] < minX) minX = corners[i][0];
        if (corners[i][0] > maxX) maxX = corners[i][0];
        if (corners[i][1] < minZ) minZ = corners[i][1];
        if (corners[i][1] > maxZ) maxZ = corners[i][1];
    }
    
    int hmMinX = (int)((position.x + minX + 10.0f) / (20.0f / heightmapSize));
    int hmMaxX = (int)((position.x + maxX + 10.0f) / (20.0f / heightmapSize));
    int hmMinZ = (int)((position.z + minZ + 10.0f) / (20.0f / heightmapSize));
    int hmMaxZ = (int)((position.z + maxZ + 10.0f) / (20.0f / heightmapSize));
    
    float totalDisplacedVolume = 0.0f;
    
    for (int z = hmMinZ; z <= hmMaxZ; z++) {
        for (int x = hmMinX; x <= hmMaxX; x++) {
            if (x >= 0 && x < heightmapSize && z >= 0 && z < heightmapSize) {
                float cellWorldX = (float)x * (20.0f / heightmapSize) - 10.0f;
                float cellWorldZ = (float)z * (20.0f / heightmapSize) - 10.0f;
                
                float localX = (cellWorldX - position.x) * cosR + (cellWorldZ - position.z) * sinR;
                float localZ = (cellWorldX - position.x) * sinR + (cellWorldZ - position.z) * cosR;
                
                if (fabsf(localX) > halfExtentX || fabsf(localZ) > halfExtentZ) {
                    continue;
                }
                
                int idx = z * heightmapSize + x;
                float currentHeight = heightSamples[idx];
                
                if (cubeBottomY < currentHeight && currentHeight > 0.0f) {
                    float digAmount = digDepth * deltaTime * 5.0f;
                    float actualDig = fminf(digAmount, currentHeight - cubeBottomY);
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

void Blade::UpdatePhysicsBody(BodyInterface& bodyInterface)
{
    Quat rotation = GetRotationQuat();
    bodyInterface.SetPositionAndRotation(bodyID,
        RVec3(position.x, position.y, position.z),
        rotation,
        EActivation::Activate);
    
    // Set velocity for proper collision response
    if (driveMode == DriveMode::Manual) {
        bodyInterface.SetLinearVelocity(bodyID, Vec3(velocity.x, 0.0f, velocity.z));
    } else if (driveMode == DriveMode::Circle) {
        float vx = -currentSpeed * sinf(circleAngle);
        float vz = currentSpeed * cosf(circleAngle);
        bodyInterface.SetLinearVelocity(bodyID, Vec3(vx, 0.0f, vz));
    } else {
        bodyInterface.SetLinearVelocity(bodyID, Vec3(currentSpeed, 0.0f, 0.0f));
    }
}

float Blade::GetEffectiveRotation() const
{
    switch (driveMode) {
        case DriveMode::Manual:
            return rotationY;
        case DriveMode::Circle:
            return circleAngle + 3.14159f / 2.0f + bladeRotation * 3.14159f / 180.0f;
        case DriveMode::Linear:
        default:
            return bladeRotation * 3.14159f / 180.0f;
    }
}

Quat Blade::GetRotationQuat() const
{
    return Quat::sRotation(Vec3(0, 1, 0), GetEffectiveRotation());
}

void Blade::SetDriveMode(DriveMode mode)
{
    driveMode = mode;
    
    if (mode == DriveMode::Circle) {
        circleAngle = 0.0f;
        position.x = circleCenterX + circleRadius;
        position.z = circleCenterZ;
    } else if (mode == DriveMode::Manual) {
        velocity = {0.0f, 0.0f, 0.0f};
    }
}

void Blade::SetLinearSettings(float newStartX, float newStartZ, float newMinX, float newMaxX)
{
    startX = newStartX;
    startZ = newStartZ;
    minX = newMinX;
    maxX = newMaxX;
}

void Blade::SetCircleSettings(float centerX, float centerZ, float radius)
{
    circleCenterX = centerX;
    circleCenterZ = centerZ;
    circleRadius = radius;
}
