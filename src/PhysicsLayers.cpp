#include "PhysicsLayers.h"

// BPLayerInterfaceImpl implementation
BPLayerInterfaceImpl::BPLayerInterfaceImpl()
{
    mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayer(0);
    mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayer(1);
}

uint BPLayerInterfaceImpl::GetNumBroadPhaseLayers() const
{
    return 2;
}

BroadPhaseLayer BPLayerInterfaceImpl::GetBroadPhaseLayer(ObjectLayer inLayer) const
{
    return mObjectToBroadPhase[inLayer];
}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
const char* BPLayerInterfaceImpl::GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const
{
    switch ((BroadPhaseLayer::Type)inLayer)
    {
    case 0: return "NON_MOVING";
    case 1: return "MOVING";
    default: return "INVALID";
    }
}
#endif

// ObjectVsBroadPhaseLayerFilterImpl implementation
bool ObjectVsBroadPhaseLayerFilterImpl::ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const
{
    switch (inLayer1)
    {
    case Layers::NON_MOVING:
        return inLayer2 == BroadPhaseLayer(1);
    case Layers::MOVING:
        return true;
    default:
        return false;
    }
}

// ObjectLayerPairFilterImpl implementation
bool ObjectLayerPairFilterImpl::ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const
{
    switch (inObject1)
    {
    case Layers::NON_MOVING:
        return inObject2 == Layers::MOVING;
    case Layers::MOVING:
        return true;
    default:
        return false;
    }
}

// MyContactListener implementation
ValidateResult MyContactListener::OnContactValidate(const Body &inBody1, const Body &inBody2, RVec3Arg inBaseOffset, const CollideShapeResult &inCollisionResult)
{
    return ValidateResult::AcceptAllContactsForThisBodyPair;
}

void MyContactListener::OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings)
{
}

void MyContactListener::OnContactPersisted(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings)
{
}

void MyContactListener::OnContactRemoved(const SubShapeIDPair &inSubShapePair)
{
}
