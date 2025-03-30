cmake_minimum_required(VERSION 3.12)
project(slrbs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(OpenGL REQUIRED)

# Find and enable OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  add_compile_definitions(USE_OPENMP)
endif()

# Enable SIMD instructions
if(MSVC)
  add_compile_options(/arch:AVX2)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-mavx2)
endif()

if (APPLE)
  add_definitions( -DGL_SILENCE_DEPRECATION )
endif()

include(FetchContent)
FetchContent_Declare(
  polyscope
  GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
  GIT_TAG v2.3.0  # Upgraded from v2.2.1
  GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(polyscope)
FetchContent_GetProperties(polyscope SOURCE_DIR polyscope_SRC_DIR BINARY_DIR polyscope_BIN_DIR)

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

# Add JSON library (nlohmann/json)
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.2
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(json)

# Add stb image library (header-only)
FetchContent_Declare(
  stb
  GIT_REPOSITORY https://github.com/nothings/stb.git
  GIT_TAG master
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(stb)
FetchContent_GetProperties(stb SOURCE_DIR stb_SRC_DIR)

set(collision_HEADERS include/collision/CollisionDetect.h 
                      include/collision/Geometry.h )
set(collision_SOURCE src/collision/CollisionDetect.cpp ) 

set(rigidbody_HEADERS include/rigidbody/RigidBody.h
					  include/rigidbody/RigidBodyState.h
                      include/rigidbody/RigidBodySystem.h 
                      include/rigidbody/Scenarios.h )
set(rigidbody_SOURCE src/rigidbody/RigidBody.cpp 
					 src/rigidbody/RigidBodyState.cpp
                     src/rigidbody/RigidBodySystem.cpp )

set(joint_HEADERS include/joint/Joint.h
				  include/joint/Hinge.h
				  include/joint/Spherical.h )
set(joint_SOURCE src/joint/Joint.cpp
				 src/joint/Hinge.cpp
			     src/joint/Spherical.cpp )

set(contact_HEADERS include/contact/Contact.h )
set(contact_SOURCE src/contact/Contact.cpp )

set(solver_HEADERS include/solvers/Solver.h 
                   include/solvers/SolverBoxPGS.h
				   include/solvers/SolverConjGradient.h
				   include/solvers/SolverConjResidual.h)
set(solver_SOURCE src/solvers/SolverBoxPGS.cpp
				  src/solvers/SolverConjGradient.cpp
	              src/solvers/SolverConjResidual.cpp)

set(utils_HEADERS include/utils/OBJLoader.h
                 include/utils/MeshAssets.h
				 include/utils/Types.h )
set(utils_SOURCE src/utils/OBJLoader.cpp
                src/utils/MeshAssets.cpp )

set(viewer_HEADERS include/viewer/SimViewer.h)
set(viewer_SOURCE src/viewer/SimViewer.cpp)
        
add_executable (slrbs main.cpp
          ${contact_HEADERS} ${contact_SOURCE} 
          ${joint_HEADERS} ${joint_SOURCE}
          ${collision_HEADERS} ${collision_SOURCE}
          ${rigidbody_HEADERS} ${rigidbody_SOURCE}
          ${solver_HEADERS} ${solver_SOURCE}
		  ${utils_HEADERS} ${utils_SOURCE}
          ${viewer_HEADERS} ${viewer_SOURCE} )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${Eigen_SRC_DIR}
  ${stb_SRC_DIR}
  ${COMMON_INCLUDES}
)

target_link_libraries(slrbs OpenGL::GL polyscope nlohmann_json::nlohmann_json)

# Link OpenMP if found
if(OpenMP_CXX_FOUND)
  target_link_libraries(slrbs OpenMP::OpenMP_CXX)
endif()

# Setup source code groups
#
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
source_group(src/util FILES ${util_SOURCE})

if(MSVC) 
	set_property(TARGET slrbs PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()