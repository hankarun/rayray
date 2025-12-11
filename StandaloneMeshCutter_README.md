# Standalone Mesh Cutter

A dependency-free, single-file C++ implementation of a mesh cutting algorithm. This library can be easily integrated into any project without requiring external dependencies.

## Features

- ✅ **Dependency-free**: No external libraries required (pure C++ with STL)
- ✅ **Single header file**: Just include `StandaloneMeshCutter.h`
- ✅ **Complete algorithm**: Cut triangle meshes with arbitrary planes
- ✅ **Cap generation**: Automatically closes cut surfaces with proper triangulation
- ✅ **Center of mass calculation**: Compute weighted center of mass for mesh pieces
- ✅ **Robust**: Handles edge cases and degenerate triangles

## Quick Start

### 1. Include the header

```cpp
#include "StandaloneMeshCutter.h"
```

### 2. Define your mesh

```cpp
std::vector<SMC::MeshTriangle> mesh;

// Create a triangle
SMC::MeshTriangle tri;
tri.v0 = SMC::Vector3(0.0f, 0.0f, 0.0f);
tri.v1 = SMC::Vector3(1.0f, 0.0f, 0.0f);
tri.v2 = SMC::Vector3(0.5f, 1.0f, 0.0f);
tri.n0 = tri.n1 = tri.n2 = SMC::Vector3(0.0f, 0.0f, 1.0f); // Normal
tri.color = SMC::Color(255, 0, 0, 255); // Red

mesh.push_back(tri);
// Add more triangles...
```

### 3. Define a cutting plane

```cpp
// Create a plane from a point and normal vector
SMC::Vector3 planePoint(0.0f, 0.5f, 0.0f);
SMC::Vector3 planeNormal(0.0f, 1.0f, 0.0f); // Normal points up
SMC::CutPlane plane = SMC::CutPlane::FromPointNormal(planePoint, planeNormal);
```

### 4. Cut the mesh

```cpp
// Perform the cut
SMC::CutResult result = SMC::MeshCutter::CutMesh(mesh, plane);

if (result.wasCut) {
    // Mesh was successfully divided
    std::cout << "Front piece has " << result.frontMesh.size() << " triangles\n";
    std::cout << "Back piece has " << result.backMesh.size() << " triangles\n";
    
    // Calculate center of mass for each piece
    SMC::Vector3 frontCOM = SMC::MeshCutter::CalculateCenterOfMass(result.frontMesh);
    SMC::Vector3 backCOM = SMC::MeshCutter::CalculateCenterOfMass(result.backMesh);
}
```

## API Reference

### Core Types

#### `SMC::Vector3`
Simple 3D vector with basic operations.

```cpp
struct Vector3 {
    float x, y, z;
    
    Vector3();
    Vector3(float x, float y, float z);
    
    // Operations
    Vector3 operator+(const Vector3& v) const;
    Vector3 operator-(const Vector3& v) const;
    Vector3 operator*(float s) const;
    Vector3 operator/(float s) const;
    
    float Dot(const Vector3& v) const;
    Vector3 Cross(const Vector3& v) const;
    float Length() const;
    Vector3 Normalized() const;
};
```

#### `SMC::Color`
RGBA color representation.

```cpp
struct Color {
    unsigned char r, g, b, a;
    
    Color();
    Color(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255);
};
```

#### `SMC::CutPlane`
Represents a cutting plane defined by normal and distance from origin.

```cpp
struct CutPlane {
    Vector3 normal;
    float distance;
    
    // Create plane from point and normal
    static CutPlane FromPointNormal(Vector3 point, Vector3 normal);
    
    // Get signed distance from point to plane
    float SignedDistance(Vector3 point) const;
    
    // Classify point: 1 = front, -1 = back, 0 = on plane
    int ClassifyPoint(Vector3 point, float epsilon = 0.0001f) const;
};
```

#### `SMC::MeshTriangle`
Triangle with positions, normals, and color.

```cpp
struct MeshTriangle {
    Vector3 v0, v1, v2;     // Vertex positions
    Vector3 n0, n1, n2;     // Per-vertex normals
    Color color;            // Triangle color
};
```

#### `SMC::CutResult`
Result of a mesh cutting operation.

```cpp
struct CutResult {
    std::vector<MeshTriangle> frontMesh;  // Triangles on positive side of plane
    std::vector<MeshTriangle> backMesh;   // Triangles on negative side of plane
    bool wasCut;                          // True if mesh was divided
};
```

### Core Functions

#### `SMC::MeshCutter::CutMesh()`
Cut a mesh with a plane.

```cpp
static CutResult CutMesh(
    const std::vector<MeshTriangle>& triangles,
    const CutPlane& plane
);
```

**Parameters:**
- `triangles`: Input mesh as a vector of triangles
- `plane`: Cutting plane

**Returns:** `CutResult` containing the two mesh pieces and cut status

**Behavior:**
- Clips each triangle against the plane
- Generates cap faces to close the cut surfaces
- Returns `wasCut = true` only if geometry exists on both sides

#### `SMC::MeshCutter::CalculateCenterOfMass()`
Calculate the area-weighted center of mass for a mesh.

```cpp
static Vector3 CalculateCenterOfMass(
    const std::vector<MeshTriangle>& triangles
);
```

**Parameters:**
- `triangles`: Input mesh as a vector of triangles

**Returns:** Center of mass as a `Vector3`

## Integration Examples

### Using with Your Own Vector/Color Types

If you already have Vector3 or Color types in your project, you can easily convert between them:

```cpp
// Convert from your types to SMC types
SMC::Vector3 ToSMC(const YourVector3& v) {
    return SMC::Vector3(v.x, v.y, v.z);
}

// Convert from SMC types to your types
YourVector3 FromSMC(const SMC::Vector3& v) {
    return YourVector3(v.x, v.y, v.z);
}

// Use in your code
std::vector<SMC::MeshTriangle> smcMesh = ConvertToSMCMesh(yourMesh);
SMC::CutResult result = SMC::MeshCutter::CutMesh(smcMesh, plane);
YourMesh yourResultMesh = ConvertFromSMCMesh(result.frontMesh);
```

### Integration with Raylib

```cpp
#include "raylib.h"
#include "StandaloneMeshCutter.h"

SMC::Vector3 RaylibToSMC(Vector3 v) {
    return SMC::Vector3(v.x, v.y, v.z);
}

Vector3 SMCToRaylib(SMC::Vector3 v) {
    return Vector3{v.x, v.y, v.z};
}

// Convert and cut...
```

### Integration with GLM

```cpp
#include <glm/glm.hpp>
#include "StandaloneMeshCutter.h"

SMC::Vector3 GLMToSMC(const glm::vec3& v) {
    return SMC::Vector3(v.x, v.y, v.z);
}

glm::vec3 SMCToGLM(const SMC::Vector3& v) {
    return glm::vec3(v.x, v.y, v.z);
}

// Convert and cut...
```

## Algorithm Details

### Mesh Cutting Process

1. **Triangle Classification**: Each triangle is classified based on which side of the plane its vertices lie on
2. **Triangle Clipping**: Triangles that cross the plane are split into multiple triangles
3. **Cap Generation**: The algorithm generates triangles to cap the cut surface, creating closed meshes
4. **Result Assembly**: The clipped triangles are assembled into two separate meshes

### Cap Generation

The algorithm automatically generates cap faces to close cut surfaces:
- Collects all edge intersection points
- Removes duplicate points
- Sorts points around their centroid
- Creates a fan triangulation to fill the cap

### Performance Considerations

- Time complexity: O(n) where n is the number of triangles
- Space complexity: O(n) for the output meshes
- No dynamic memory allocation in hot paths (except for result storage)

## Testing

A test program is provided in `test_standalone_cutter.cpp`:

```bash
# Compile the test
g++ -std=c++17 -o test_standalone_cutter test_standalone_cutter.cpp

# Run the test
./test_standalone_cutter
```

## License

This is a standalone extraction of the mesh cutting algorithm from the RayRay project. Use freely in your projects.

## Credits

Extracted from the RayRay physics simulation project.
