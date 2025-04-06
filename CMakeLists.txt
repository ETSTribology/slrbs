cmake_minimum_required(VERSION 3.12)
project(slrbs VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add cmake modules path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Find required packages
find_package(OpenGL REQUIRED)

# Platform-specific settings
if(APPLE)
  add_definitions(-DGL_SILENCE_DEPRECATION)
endif()

# Enable strict warnings and treat them as errors (can be commented out during development)
if(MSVC)
  add_compile_options(/W4)
  # Uncomment to treat warnings as errors: add_compile_options(/WX)
else()
  add_compile_options(-Wall -Wextra -pedantic)
  # Uncomment to treat warnings as errors: add_compile_options(-Werror)
endif()

# External dependencies
include(FetchContent)

# Polyscope
FetchContent_Declare(
  polyscope
  GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
  GIT_TAG v2.2.1
  GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(polyscope)
FetchContent_GetProperties(polyscope SOURCE_DIR polyscope_SRC_DIR BINARY_DIR polyscope_BIN_DIR)

# Eigen
FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_SUBDIR cmake
)
FetchContent_MakeAvailable(Eigen)
FetchContent_GetProperties(Eigen SOURCE_DIR Eigen_SRC_DIR BINARY_DIR Eigen_BIN_DIR)

# Define source files by component
# Collision component
set(collision_HEADERS 
  include/collision/CollisionDetect.h 
  include/collision/Geometry.h
)
set(collision_SOURCE 
  src/collision/CollisionDetect.cpp
)

# Rigid body component
set(rigidbody_HEADERS 
  include/rigidbody/RigidBody.h
  include/rigidbody/RigidBodyState.h
  include/rigidbody/RigidBodySystem.h 
  include/rigidbody/Scenarios.h
)
set(rigidbody_SOURCE 
  src/rigidbody/RigidBody.cpp 
  src/rigidbody/RigidBodyState.cpp
  src/rigidbody/RigidBodySystem.cpp
)

# Joint component
set(joint_HEADERS 
  include/joint/Joint.h
  include/joint/Hinge.h
  include/joint/Spherical.h
)
set(joint_SOURCE 
  src/joint/Joint.cpp
  src/joint/Hinge.cpp
  src/joint/Spherical.cpp
)

# Contact component
set(contact_HEADERS 
  include/contact/Contact.h
)
set(contact_SOURCE 
  src/contact/Contact.cpp
)

# Solver component
set(solver_HEADERS 
  include/solvers/Solver.h 
  include/solvers/SolverBoxPGS.h
  include/solvers/SolverConjGradient.h
  include/solvers/SolverConjResidual.h
)
set(solver_SOURCE 
  src/solvers/SolverBoxPGS.cpp
  src/solvers/SolverConjGradient.cpp
  src/solvers/SolverConjResidual.cpp
)

# Utility component
set(util_HEADERS 
  include/util/OBJLoader.h
  include/util/MeshAssets.h
  include/util/Types.h
)
set(util_SOURCE 
  src/util/OBJLoader.cpp
  src/util/MeshAssets.cpp
)

# Viewer component
set(viewer_HEADERS 
  include/viewer/SimViewer.h
)
set(viewer_SOURCE 
  src/viewer/SimViewer.cpp
)

# Create executable target
add_executable(slrbs 
  main.cpp
  ${contact_HEADERS} ${contact_SOURCE} 
  ${joint_HEADERS} ${joint_SOURCE}
  ${collision_HEADERS} ${collision_SOURCE}
  ${rigidbody_HEADERS} ${rigidbody_SOURCE}
  ${solver_HEADERS} ${solver_SOURCE}
  ${util_HEADERS} ${util_SOURCE}
  ${viewer_HEADERS} ${viewer_SOURCE}
)

# Set include directories
target_include_directories(slrbs
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Eigen_SRC_DIR}
)

# Link libraries
target_link_libraries(slrbs
  PRIVATE
    OpenGL::GL
    polyscope
)

# Setup file grouping for IDEs (particularly Visual Studio)
source_group(src FILES main.cpp)
source_group(src/collision FILES ${collision_SOURCE})
source_group(include/collision FILES ${collision_HEADERS})
source_group(src/joint FILES ${joint_SOURCE})
source_group(include/joint FILES ${joint_HEADERS})
source_group(src/contact FILES ${contact_SOURCE})
source_group(include/contact FILES ${contact_HEADERS})
source_group(src/rigidbody FILES ${rigidbody_SOURCE})
source_group(include/rigidbody FILES ${rigidbody_HEADERS})
source_group(src/solvers FILES ${solver_SOURCE})
source_group(include/solvers FILES ${solver_HEADERS})
source_group(src/util FILES ${util_SOURCE})
source_group(include/util FILES ${util_HEADERS})
source_group(src/viewer FILES ${viewer_SOURCE})
source_group(include/viewer FILES ${viewer_HEADERS})

# Generate compile_commands.json for tooling support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set working directory for Visual Studio
if(MSVC) 
  set_property(TARGET slrbs PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

# Install configuration
install(TARGETS slrbs
  RUNTIME DESTINATION bin
)

# Testing (optional)
option(BUILD_TESTING "Build tests" OFF)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()

# Print summary
message(STATUS "Configuration summary:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Using polyscope: ${polyscope_SRC_DIR}")
message(STATUS "  Using Eigen: ${Eigen_SRC_DIR}")
