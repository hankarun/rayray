cmake_minimum_required(VERSION 3.15)

project(HelloWorld)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force static runtime library for all targets
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

include(FetchContent)

FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
)

FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG v5.2.0
    SOURCE_SUBDIR Build
)

# Force static runtime library for all projects
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Jolt configuration
set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)

# Force raylib to use static runtime
set(USE_MSVC_RUNTIME_LIBRARY_DLL OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(raylib JoltPhysics)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    main.cpp
    src/PhysicsLayers.cpp
    src/PhysicsStructures.cpp
    src/DensityAutomata.cpp
    src/ParticleVoxelGrid.cpp
    src/PerlinNoise.cpp
    src/Helpers.cpp
    src/MeshCutter.cpp
    src/GeometryBuilder.cpp
    src/JoltPhysicsBackend.cpp
)

# Optional: Enable PhysX backend (uncomment when PhysX is available)
# option(USE_PHYSX "Enable NVIDIA PhysX backend" OFF)
# if(USE_PHYSX)
#     find_package(PhysX REQUIRED)
#     target_compile_definitions(hello_world PRIVATE USE_PHYSX)
#     target_link_libraries(hello_world PhysX PhysXCooking PhysXCommon PhysXFoundation)
# endif()

add_executable(hello_world ${SOURCES})

target_link_libraries(hello_world raylib Jolt)

# Set working directory to source directory for debugging
set_target_properties(hello_world PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)
