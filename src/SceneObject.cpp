#include "SceneObject.h"

SceneObject::SceneObject()
    : position({0.0f, 0.0f, 0.0f})
    , rotationY(0.0f)
    , bodyID()
    , color(::Color{0, 121, 241, 255})  // BLUE
    , mass(100.0f)
    , enginePower(500.0f)
    , currentSpeed(0.0f)
    , maxSpeed(3.0f)
    , canDigTerrain(true)
    , digDepth(0.3f)
{
}

SceneObject::~SceneObject()
{
}

void SceneObject::Cleanup(JPH::BodyInterface& bodyInterface)
{
    if (!bodyID.IsInvalid()) {
        bodyInterface.RemoveBody(bodyID);
        bodyInterface.DestroyBody(bodyID);
    }
}
