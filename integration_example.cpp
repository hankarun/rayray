/*
 * Integration Example for StandaloneMeshCutter
 * 
 * This demonstrates how to integrate the standalone mesh cutter
 * into another project that uses different vector/color types.
 */

#include "StandaloneMeshCutter.h"
#include <iostream>
#include <vector>
#include <cmath>

// ============================================================================
// Example: Your Project's Types
// ============================================================================

// Let's say your project uses these types:
struct MyVector3 {
    double x, y, z;
    MyVector3(double x_ = 0, double y_ = 0, double z_ = 0) : x(x_), y(y_), z(z_) {}
};

struct MyColor {
    int r, g, b;
    MyColor(int r_ = 0, int g_ = 0, int b_ = 0) : r(r_), g(g_), b(b_) {}
};

struct MyTriangle {
    MyVector3 vertices[3];
    MyVector3 normals[3];
    MyColor color;
};

struct MyMesh {
    std::vector<MyTriangle> triangles;
};

// ============================================================================
// Conversion Functions
// ============================================================================

SMC::Vector3 ConvertVector(const MyVector3& v) {
    return SMC::Vector3(static_cast<float>(v.x), 
                       static_cast<float>(v.y), 
                       static_cast<float>(v.z));
}

MyVector3 ConvertVector(const SMC::Vector3& v) {
    return MyVector3(static_cast<double>(v.x), 
                    static_cast<double>(v.y), 
                    static_cast<double>(v.z));
}

SMC::Color ConvertColor(const MyColor& c) {
    return SMC::Color(static_cast<unsigned char>(c.r),
                     static_cast<unsigned char>(c.g),
                     static_cast<unsigned char>(c.b));
}

MyColor ConvertColor(const SMC::Color& c) {
    return MyColor(static_cast<int>(c.r),
                  static_cast<int>(c.g),
                  static_cast<int>(c.b));
}

SMC::MeshTriangle ConvertTriangle(const MyTriangle& tri) {
    SMC::MeshTriangle result;
    result.v0 = ConvertVector(tri.vertices[0]);
    result.v1 = ConvertVector(tri.vertices[1]);
    result.v2 = ConvertVector(tri.vertices[2]);
    result.n0 = ConvertVector(tri.normals[0]);
    result.n1 = ConvertVector(tri.normals[1]);
    result.n2 = ConvertVector(tri.normals[2]);
    result.color = ConvertColor(tri.color);
    return result;
}

MyTriangle ConvertTriangle(const SMC::MeshTriangle& tri) {
    MyTriangle result;
    result.vertices[0] = ConvertVector(tri.v0);
    result.vertices[1] = ConvertVector(tri.v1);
    result.vertices[2] = ConvertVector(tri.v2);
    result.normals[0] = ConvertVector(tri.n0);
    result.normals[1] = ConvertVector(tri.n1);
    result.normals[2] = ConvertVector(tri.n2);
    result.color = ConvertColor(tri.color);
    return result;
}

std::vector<SMC::MeshTriangle> ConvertMesh(const MyMesh& mesh) {
    std::vector<SMC::MeshTriangle> result;
    result.reserve(mesh.triangles.size());
    for (const auto& tri : mesh.triangles) {
        result.push_back(ConvertTriangle(tri));
    }
    return result;
}

MyMesh ConvertMesh(const std::vector<SMC::MeshTriangle>& triangles) {
    MyMesh result;
    result.triangles.reserve(triangles.size());
    for (const auto& tri : triangles) {
        result.triangles.push_back(ConvertTriangle(tri));
    }
    return result;
}

// ============================================================================
// High-Level API for Your Project
// ============================================================================

struct MyCutResult {
    MyMesh frontPiece;
    MyMesh backPiece;
    bool wasCut;
};

// Cut a mesh with your project's types
MyCutResult CutMyMesh(const MyMesh& mesh, 
                      const MyVector3& planePoint, 
                      const MyVector3& planeNormal) 
{
    // Convert to SMC types
    std::vector<SMC::MeshTriangle> smcMesh = ConvertMesh(mesh);
    SMC::CutPlane plane = SMC::CutPlane::FromPointNormal(
        ConvertVector(planePoint),
        ConvertVector(planeNormal)
    );
    
    // Perform the cut
    SMC::CutResult smcResult = SMC::MeshCutter::CutMesh(smcMesh, plane);
    
    // Convert result back to your types
    MyCutResult result;
    result.frontPiece = ConvertMesh(smcResult.frontMesh);
    result.backPiece = ConvertMesh(smcResult.backMesh);
    result.wasCut = smcResult.wasCut;
    
    return result;
}

// ============================================================================
// Example Usage
// ============================================================================

MyMesh CreateCubeMesh() {
    MyMesh mesh;
    MyColor color(200, 100, 50);
    
    // Create a simple cube with 12 triangles (2 per face)
    // For brevity, we'll just create 2 triangles forming a square on the XY plane
    
    MyTriangle tri1;
    tri1.vertices[0] = MyVector3(-1, -1, 0);
    tri1.vertices[1] = MyVector3( 1, -1, 0);
    tri1.vertices[2] = MyVector3( 1,  1, 0);
    tri1.normals[0] = tri1.normals[1] = tri1.normals[2] = MyVector3(0, 0, 1);
    tri1.color = color;
    mesh.triangles.push_back(tri1);
    
    MyTriangle tri2;
    tri2.vertices[0] = MyVector3(-1, -1, 0);
    tri2.vertices[1] = MyVector3( 1,  1, 0);
    tri2.vertices[2] = MyVector3(-1,  1, 0);
    tri2.normals[0] = tri2.normals[1] = tri2.normals[2] = MyVector3(0, 0, 1);
    tri2.color = color;
    mesh.triangles.push_back(tri2);
    
    return mesh;
}

int main() {
    std::cout << "=== Mesh Cutting Integration Example ===" << std::endl << std::endl;
    
    // Create a mesh using your project's types
    MyMesh mesh = CreateCubeMesh();
    std::cout << "Created mesh with " << mesh.triangles.size() << " triangles" << std::endl;
    
    // Define a cutting plane
    MyVector3 planePoint(0, 0, 0);
    MyVector3 planeNormal(0, 1, 0); // Cut horizontally
    std::cout << "Cutting with plane at Y=0" << std::endl << std::endl;
    
    // Cut the mesh using the wrapper function
    MyCutResult result = CutMyMesh(mesh, planePoint, planeNormal);
    
    std::cout << "Cut result:" << std::endl;
    std::cout << "  Was cut: " << (result.wasCut ? "YES" : "NO") << std::endl;
    std::cout << "  Front piece: " << result.frontPiece.triangles.size() << " triangles" << std::endl;
    std::cout << "  Back piece: " << result.backPiece.triangles.size() << " triangles" << std::endl;
    std::cout << std::endl;
    
    if (result.wasCut) {
        // Calculate center of mass for each piece
        std::vector<SMC::MeshTriangle> frontSMC = ConvertMesh(result.frontPiece);
        std::vector<SMC::MeshTriangle> backSMC = ConvertMesh(result.backPiece);
        
        SMC::Vector3 frontCOM = SMC::MeshCutter::CalculateCenterOfMass(frontSMC);
        SMC::Vector3 backCOM = SMC::MeshCutter::CalculateCenterOfMass(backSMC);
        
        std::cout << "Front piece center of mass: (" 
                  << frontCOM.x << ", " << frontCOM.y << ", " << frontCOM.z << ")" << std::endl;
        std::cout << "Back piece center of mass: (" 
                  << backCOM.x << ", " << backCOM.y << ", " << backCOM.z << ")" << std::endl;
    }
    
    std::cout << std::endl << "=== Integration successful! ===" << std::endl;
    
    return 0;
}
