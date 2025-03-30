cmake_minimum_required(VERSION 3.5...4.0)

# Setup versioning
set(SLRBS_MAJOR_VERSION 1)
set(SLRBS_MINOR_VERSION 0)
set(SLRBS_PATCH_VERSION 0)
set(SLRBS_VERSION "${SLRBS_MAJOR_VERSION}.${SLRBS_MINOR_VERSION}.${SLRBS_PATCH_VERSION}")

project(slrbs VERSION ${SLRBS_VERSION} LANGUAGES CXX)

# ============================================================================
# Load SLRBS CMake modules
# ============================================================================
list(APPEND CMAKE_MODULE_PATH 
     "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
     "${CMAKE_CURRENT_SOURCE_DIR}/cmake/find"
     "${CMAKE_CURRENT_SOURCE_DIR}/cmake/recipes"
     "${CMAKE_CURRENT_SOURCE_DIR}/cmake/slrbs")

# Include SLRBS-specific CMake utilities
include(slrbs_use_colors)
include(slrbs_warnings)
include(slrbs_filter_flags)
include(slrbs_cpm_cache)

# Include CMP configuration
include(cmake/cmp/CMPConfig.cmake)

# ============================================================================
# Enable ccache support
# ============================================================================
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
  set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  set(CMAKE_CUDA_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
else()
  message(STATUS "ccache not found")
endif()

# Silence CMake deprecation warnings from dependencies
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)

# Configure compiler settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# Set policy versions to avoid warnings
if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif()
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

# ============================================================================
# Set a default build configuration (Release)
# ============================================================================
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Choose the type of build." FORCE
  )
  set_property(
    CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo"
  )
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" U_CMAKE_BUILD_TYPE)

# ============================================================================
# Platform specific settings
# ============================================================================
# Find SIMD support
include(FindSIMD)
if(SIMD_SSE STREQUAL "FALSE" AND SIMD_AVX STREQUAL "FALSE" AND NOT SIMD_NEON)
  message(WARNING "No SIMD instruction sets detected. Performance may be reduced.")
endif()

# Add flags to compiler
if(NOT EMSCRIPTEN)
  foreach(flag ${SIMD_CXX_FLAGS})
    add_compile_options("${flag}")
  endforeach()
endif()

# ============================================================================
# Setup for Emscripten if enabled
# ============================================================================
if(EMSCRIPTEN)
  set(CMAKE_EXECUTABLE_SUFFIX ".html")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s USE_PTHREADS=1")
  set(CMAKE_THREAD_LIBS_INIT "-lpthread")
  set(CMAKE_HAVE_THREADS_LIBRARY 1)
  set(CMAKE_USE_WIN32_THREADS_INIT 0)
  set(CMAKE_USE_PTHREADS_INIT 1)
  set(THREADS_PREFER_PTHREAD_FLAG ON)
  add_compile_options(-Wno-unsafe-buffer-usage)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-conversion")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-conversion -Wno-atomic-alignment")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-conversion")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-function")
endif()

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Release)

# For Python module output
if(WIN32)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Debug)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Release)
else()
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

# ============================================================================
# Dependencies
# ============================================================================
include(CPM)

# Core dependencies
include(eigen)
include(spdlog)
include(json)
include(pybind11)

# Optional dependencies based on components
if(BUILD_VIEWER)
    find_package(OpenGL REQUIRED)
endif()

# Find and enable OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  add_compile_definitions(USE_OPENMP)
endif()

if (APPLE)
  add_definitions( -DGL_SILENCE_DEPRECATION )
endif()

# ============================================================================
# Project source files
# ============================================================================

set(collision_HEADERS include/collision/CollisionDetect.h 
                      include/collision/Geometry.h)
set(collision_SOURCE src/collision/CollisionDetect.cpp
                     src/collision/CollisionDetectBoxBox.cpp) 

set(contact_HEADERS include/contact/Contact.h)
set(contact_SOURCE src/contact/Contact.cpp) 

set(joint_HEADERS include/joint/Joint.h
                 include/joint/Hinge.h
                 include/joint/Spherical.h
                 include/joint/Distance.h
                 include/joint/Prismatic.h
                 include/joint/Universal.h)
set(joint_SOURCE src/joint/Joint.cpp
                src/joint/Hinge.cpp
                src/joint/Spherical.cpp
                src/joint/Distance.cpp
                src/joint/Prismatic.cpp
                src/joint/Universal.cpp)

set(rigidbody_HEADERS include/rigidbody/RigidBody.h
                     include/rigidbody/RigidBodyState.h
                     include/rigidbody/RigidBodySystem.h)
set(rigidbody_SOURCE src/rigidbody/RigidBody.cpp 
                    src/rigidbody/RigidBodyState.cpp
                    src/rigidbody/RigidBodySystem.cpp)

set(solver_HEADERS include/solvers/Solver.h 
                   include/solvers/SolverBoxPGS.h
                   include/solvers/SolverConjGradient.h
                   include/solvers/SolverConjResidual.h
                   include/solvers/SolverProximal.h
                   include/solvers/SolverBoxBPP.h
                   include/solvers/SolverPGSSM.h)
set(solver_SOURCE src/solvers/SolverBoxPGS.cpp
                  src/solvers/SolverConjGradient.cpp
                  src/solvers/SolverConjResidual.cpp
                  src/solvers/SolverProximal.cpp
                  src/solvers/SolverBoxBPP.cpp
                  src/solvers/SolverPGSSM.cpp)

set(utils_HEADERS include/utils/OBJLoader.h
                 include/utils/MeshAssets.h
                 include/utils/Types.h
                 include/utils/Serializer.h
                 include/utils/JsonLoader.h
                 include/utils/TextureLoader.h)
set(utils_SOURCE src/utils/OBJLoader.cpp
                src/utils/MeshAssets.cpp
                src/utils/Serializer.cpp
                src/utils/JsonLoader.cpp
                src/utils/TextureLoader.cpp)


set(renderer_HEADERS include/renderer/Material.h
                     include/renderer/RigidBodyRenderer.h
                     include/renderer/PolyscopeRenderer.h
                     include/renderer/RendererFactory.h
                     include/renderer/ShaderManager.h
                     include/renderer/ShaderVars.h
                     include/renderer/TextureManager.h
                    include/renderer/RenderUtils.h)
set(renderer_SOURCE src/renderer/Material.cpp
                   src/renderer/PolyscopeRenderer.cpp
                   src/renderer/RendererFactory.cpp
                   src/renderer/ShaderManager.cpp
                   src/renderer/TextureManager.cpp
                   src/renderer/RenderUtils.cpp)

set(viewer_HEADERS include/viewer/SimViewer.h
                  include/viewer/ViewerConfig.h
                  include/viewer/ViewerUI.h
                  include/viewer/ViewerInteraction.h
                  include/viewer/ScenarioManager.h
                  include/viewer/SimulationManager.h)
set(viewer_SOURCE src/viewer/ViewerUI.cpp
                  src/viewer/ViewerInteraction.cpp
                  src/viewer/ViewerConfig.cpp
                  src/viewer/SimViewer.cpp
                  src/viewer/ScenarioManager.cpp
                  src/viewer/SimulationManager.cpp)

set(scenarios_HEADERS include/scenarios/Scenarios.h
                     include/scenarios/ScenarioLoader.h)
set(scenarios_SOURCE src/scenarios/Scenarios.cpp
                    src/scenarios/ScenarioLoader.cpp)

set(log_HEADERS include/log/Logger.h 
                include/log/CSVLogger.h
                include/log/JSONLogger.h  
                include/log/BinaryLogger.h
                include/log/LoggerFactory.h)
set(log_SOURCE src/log/Logger.cpp
               src/log/CSVLogger.cpp
               src/log/JSONLogger.cpp 
               src/log/BinaryLogger.cpp
               src/log/LoggerFactory.cpp)

# Define the Python binding sources
if(BUILD_PYTHON_BINDINGS)
  set(python_SOURCE bindings/python/py_module.cpp
                    bindings/python/py_rigidbody.cpp
                    bindings/python/py_collision.cpp
                    bindings/python/py_joint.cpp
                    bindings/python/py_solver.cpp
                    bindings/python/py_scenarios.cpp)
endif()
        
# ============================================================================
# Set up target and dependencies
# ============================================================================

# Main executable
add_executable(slrbs main.cpp
          ${contact_HEADERS} ${contact_SOURCE} 
          ${joint_HEADERS} ${joint_SOURCE}
          ${collision_HEADERS} ${collision_SOURCE}
          ${rigidbody_HEADERS} ${rigidbody_SOURCE}
          ${solver_HEADERS} ${solver_SOURCE}
          ${utils_HEADERS} ${utils_SOURCE}
          ${common_HEADERS} ${common_SOURCE}
          ${renderer_HEADERS} ${renderer_SOURCE}
          ${viewer_HEADERS} ${viewer_SOURCE}
          ${scenarios_HEADERS} ${scenarios_SOURCE}
          ${log_HEADERS} ${log_SOURCE})

target_include_directories(slrbs PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${Eigen_SRC_DIR}
  ${stb_SRC_DIR}
  ${linalg_SRC_DIR}
  ${pcg32_SRC_DIR}
)

target_link_libraries(slrbs 
  OpenGL::GL 
  polyscope 
  nlohmann_json::nlohmann_json
  spdlog::spdlog
  tinyobjloader
  glm::glm
  nanothread::nanothread
  argparse
  Threads::Threads
)

# Link OpenMP if found
if(OpenMP_CXX_FOUND)
  target_link_libraries(slrbs OpenMP::OpenMP_CXX)
endif()

# Create the Python module if enabled
if(BUILD_PYTHON_BINDINGS)
  pybind11_add_module(pyslrbs 
    ${python_SOURCE}
    ${contact_SOURCE} 
    ${joint_SOURCE}
    ${collision_SOURCE}
    ${rigidbody_SOURCE}
    ${solver_SOURCE}
    ${utils_SOURCE}
    ${scenarios_SOURCE}
    ${log_SOURCE}
  )

  target_include_directories(pyslrbs PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${Eigen_SRC_DIR}
    ${stb_SRC_DIR}
    ${linalg_SRC_DIR}
    ${pcg32_SRC_DIR}
  )

  target_link_libraries(pyslrbs PRIVATE
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    tinyobjloader
    glm::glm
    nanothread::nanothread
    Threads::Threads
  )

  # Link OpenMP if found
  if(OpenMP_CXX_FOUND)
    target_link_libraries(pyslrbs PRIVATE OpenMP::OpenMP_CXX)
  endif()

  # Install Python module
  install(TARGETS pyslrbs DESTINATION .)
endif()

# Copy resources to build directory
set(RESOURCE_DIRS
  "resources/configs"
  "resources/models"
  "resources/scenarios"
  "resources/shaders"
  "resources/textures"
)

# Function to copy resources
function(copy_resources target)
  foreach(dir ${RESOURCE_DIRS})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
      # For multi-config generators (like Visual Studio)
      if(CMAKE_CONFIGURATION_TYPES)
        foreach(config ${CMAKE_CONFIGURATION_TYPES})
          file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/${config}/${dir}")
          add_custom_command(
            TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/${dir}"
            "${CMAKE_BINARY_DIR}/${config}/${dir}"
            COMMENT "Copying ${dir} to ${config} build directory..."
          )
        endforeach()
      else()
        # For single-config generators (like Makefiles)
        file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/${dir}")
        add_custom_command(
          TARGET ${target} POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy_directory
          "${CMAKE_CURRENT_SOURCE_DIR}/${dir}"
          "${CMAKE_BINARY_DIR}/${dir}"
          COMMENT "Copying ${dir} to build directory..."
        )
      endif()
    endif()
  endforeach()
endfunction()

# Copy resources for main executable
copy_resources(slrbs)

# Setup source code groups for better organization in IDEs
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
source_group(src/utils FILES ${utils_SOURCE})
source_group(include/utils FILES ${utils_HEADERS})
source_group(src/renderer FILES ${renderer_SOURCE})
source_group(include/renderer FILES ${renderer_HEADERS})
source_group(src/viewer FILES ${viewer_SOURCE})
source_group(include/viewer FILES ${viewer_HEADERS})
source_group(src/scenarios FILES ${scenarios_SOURCE})
source_group(include/scenarios FILES ${scenarios_HEADERS})
source_group(src/log FILES ${log_SOURCE})
source_group(include/log FILES ${log_HEADERS})

if(BUILD_PYTHON_BINDINGS)
  source_group(bindings/python FILES ${python_SOURCE})
endif()

if(MSVC) 
  set_property(TARGET slrbs PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

# ============================================================================
# Configure package and install
# ============================================================================
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Generate version file
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/slrbsConfigVersion.cmake"
    VERSION ${SLRBS_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Install targets
install(TARGETS slrbs
    EXPORT slrbs-targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Install headers
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/slrbs
    FILES_MATCHING PATTERN "*.h"
)

# Export targets
install(EXPORT slrbs-targets
    FILE slrbsTargets.cmake
    NAMESPACE slrbs::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/slrbs
)