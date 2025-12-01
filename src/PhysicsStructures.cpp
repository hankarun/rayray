#include "PhysicsStructures.h"

bool IsSphereStationary(const Vector3& currentPos, const Vector3& lastPos, float threshold) {
    float dx = currentPos.x - lastPos.x;
    float dy = currentPos.y - lastPos.y;
    float dz = currentPos.z - lastPos.z;
    return (dx * dx + dy * dy + dz * dz) < threshold * threshold;
}
