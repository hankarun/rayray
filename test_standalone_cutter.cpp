/*
 * Test file for StandaloneMeshCutter.h
 * This demonstrates that the mesh cutter is dependency-free and can be used standalone
 */

#include "StandaloneMeshCutter.h"
#include <iostream>
#include <iomanip>

void PrintVector3(const char* name, const SMC::Vector3& v)
{
    std::cout << name << ": (" << std::fixed << std::setprecision(2) 
              << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
}

void PrintTriangleCount(const char* name, size_t count)
{
    std::cout << name << ": " << count << " triangles" << std::endl;
}

int main()
{
    std::cout << "=== Standalone Mesh Cutter Test ===" << std::endl << std::endl;
    
    // Create a simple cube mesh (8 vertices, 12 triangles)
    // For simplicity, we'll create a smaller test mesh with just 2 triangles forming a square
    std::vector<SMC::MeshTriangle> mesh;
    
    // Triangle 1: forms part of a square in XY plane
    SMC::MeshTriangle tri1;
    tri1.v0 = SMC::Vector3(-1.0f, -1.0f, 0.0f);
    tri1.v1 = SMC::Vector3( 1.0f, -1.0f, 0.0f);
    tri1.v2 = SMC::Vector3( 1.0f,  1.0f, 0.0f);
    tri1.n0 = tri1.n1 = tri1.n2 = SMC::Vector3(0.0f, 0.0f, 1.0f);
    tri1.color = SMC::Color(255, 0, 0, 255); // Red
    mesh.push_back(tri1);
    
    // Triangle 2: completes the square
    SMC::MeshTriangle tri2;
    tri2.v0 = SMC::Vector3(-1.0f, -1.0f, 0.0f);
    tri2.v1 = SMC::Vector3( 1.0f,  1.0f, 0.0f);
    tri2.v2 = SMC::Vector3(-1.0f,  1.0f, 0.0f);
    tri2.n0 = tri2.n1 = tri2.n2 = SMC::Vector3(0.0f, 0.0f, 1.0f);
    tri2.color = SMC::Color(255, 0, 0, 255); // Red
    mesh.push_back(tri2);
    
    std::cout << "Input mesh:" << std::endl;
    PrintTriangleCount("  Original mesh", mesh.size());
    
    // Calculate center of mass
    SMC::Vector3 centerOfMass = SMC::MeshCutter::CalculateCenterOfMass(mesh);
    PrintVector3("  Center of mass", centerOfMass);
    std::cout << std::endl;
    
    // Create a cutting plane that cuts through the middle horizontally (Y = 0)
    SMC::Vector3 planePoint(0.0f, 0.0f, 0.0f);
    SMC::Vector3 planeNormal(0.0f, 1.0f, 0.0f); // Normal points up (Y+)
    SMC::CutPlane plane = SMC::CutPlane::FromPointNormal(planePoint, planeNormal);
    
    std::cout << "Cutting plane:" << std::endl;
    PrintVector3("  Point", planePoint);
    PrintVector3("  Normal", planeNormal);
    std::cout << "  Distance from origin: " << plane.distance << std::endl << std::endl;
    
    // Perform the cut
    SMC::CutResult result = SMC::MeshCutter::CutMesh(mesh, plane);
    
    std::cout << "Cut result:" << std::endl;
    std::cout << "  Was cut: " << (result.wasCut ? "YES" : "NO") << std::endl;
    PrintTriangleCount("  Front mesh", result.frontMesh.size());
    PrintTriangleCount("  Back mesh", result.backMesh.size());
    std::cout << std::endl;
    
    if (result.wasCut)
    {
        // Calculate center of mass for each piece
        SMC::Vector3 frontCOM = SMC::MeshCutter::CalculateCenterOfMass(result.frontMesh);
        SMC::Vector3 backCOM = SMC::MeshCutter::CalculateCenterOfMass(result.backMesh);
        
        std::cout << "Front piece:" << std::endl;
        PrintVector3("  Center of mass", frontCOM);
        
        std::cout << "Back piece:" << std::endl;
        PrintVector3("  Center of mass", backCOM);
        std::cout << std::endl;
    }
    
    // Test edge cases
    std::cout << "=== Edge Case Tests ===" << std::endl << std::endl;
    
    // Test 1: Plane that doesn't intersect mesh
    SMC::CutPlane planeAbove = SMC::CutPlane::FromPointNormal(SMC::Vector3(0, 10, 0), SMC::Vector3(0, 1, 0));
    SMC::CutResult resultAbove = SMC::MeshCutter::CutMesh(mesh, planeAbove);
    std::cout << "Test 1 - Plane above mesh:" << std::endl;
    std::cout << "  Was cut: " << (resultAbove.wasCut ? "YES" : "NO") << std::endl;
    PrintTriangleCount("  Front mesh", resultAbove.frontMesh.size());
    PrintTriangleCount("  Back mesh", resultAbove.backMesh.size());
    std::cout << std::endl;
    
    // Test 2: Plane that cuts at an angle
    SMC::CutPlane planeAngled = SMC::CutPlane::FromPointNormal(
        SMC::Vector3(0, 0, 0),
        SMC::Vector3(1, 1, 0).Normalized()
    );
    SMC::CutResult resultAngled = SMC::MeshCutter::CutMesh(mesh, planeAngled);
    std::cout << "Test 2 - Angled plane:" << std::endl;
    std::cout << "  Was cut: " << (resultAngled.wasCut ? "YES" : "NO") << std::endl;
    PrintTriangleCount("  Front mesh", resultAngled.frontMesh.size());
    PrintTriangleCount("  Back mesh", resultAngled.backMesh.size());
    std::cout << std::endl;
    
    std::cout << "=== All tests completed successfully ===" << std::endl;
    
    return 0;
}
