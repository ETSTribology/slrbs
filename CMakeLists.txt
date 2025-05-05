cmake_minimum_required(VERSION 3.23)
project(slrbs 
  VERSION 0.1.0
  DESCRIPTION "SLRBS"
  HOMEPAGE_URL "https://github.com/ETSim/slrbs"
  LANGUAGES CXX)
find_package(Qt5 COMPONENTS Core Gui OpenGL REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
#  Global options
# ─────────────────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build options (defaults to Release build)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)

# Enable testing
option(BUILD_TESTING "Enable testing" ON)
if(BUILD_TESTING)
  enable_testing()
  include(CTest)
endif()

# Friendly warnings
if(MSVC)
  add_compile_options(/permissive- /W4 /Zc:preprocessor /MP)
  # Disable some MSVC warnings
  add_compile_options(/wd4267 /wd4244)
else()
  add_compile_options(-Wall -Wextra -pedantic -Wshadow -Wconversion)
  # Enable faster builds with ccache if available
  find_program(CCACHE_PROGRAM ccache)
  if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  endif()
endif()

# Silence the "OpenGL deprecated" spam on recent macOS
if(APPLE)
  add_compile_definitions(GL_SILENCE_DEPRECATION)
endif()

# ─────────────────────────────────────────────────────────────────────────────
#  OpenGL — one target name for every platform
# ─────────────────────────────────────────────────────────────────────────────
find_package(OpenGL REQUIRED)

if(TARGET OpenGL::GL)             # Linux / most Windows toolchains
  set(OPENGL_LIB OpenGL::GL)
elseif(TARGET OpenGL::OpenGL)     # macOS framework
  set(OPENGL_LIB OpenGL::OpenGL)
elseif(WIN32)                     # really old CMake on Windows
  set(OPENGL_LIB opengl32)
else()
  message(FATAL_ERROR "Could not locate a usable OpenGL target")
endif()

# ─────────────────────────────────────────────────────────────────────────────
#  Third‑party libraries (fully reproducible with FetchContent)
# ─────────────────────────────────────────────────────────────────────────────
include(FetchContent)

# Polyscope (visualisation)
FetchContent_Declare(
  polyscope
  GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
  GIT_TAG        v2.4.0
  GIT_PROGRESS   YES
)

# Eigen (linear algebra)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4
  GIT_SHALLOW    TRUE
  SOURCE_SUBDIR  cmake
)

# Actually download and configure them
FetchContent_MakeAvailable(polyscope eigen)

FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.11.2
  GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(nlohmann_json)
if(NOT nlohmann_json_POPULATED)
  FetchContent_Populate(nlohmann_json)
endif()

file(GLOB_RECURSE SRC CONFIGURE_DEPENDS
     src/*.cpp
     include/*.h)

list(APPEND SRC "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")

add_executable(slrbs ${SRC})

# ───────────────────────────────────────F──────────────────────────────────────
#  Target configuration
# ─────────────────────────────────────────────────────────────────────────────
target_include_directories(slrbs PRIVATE
${nlohmann_json_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${eigen_SOURCE_DIR})

target_link_libraries(slrbs PRIVATE
  ${OPENGL_LIB}
  polyscope
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL)

# Add postfix to Debug builds
set_target_properties(slrbs PROPERTIES DEBUG_POSTFIX "d")

# ─────────────────────────────────────────────────────────────────────────────
#  Install targets
# ─────────────────────────────────────────────────────────────────────────────
include(GNUInstallDirs)

install(TARGETS slrbs
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        BUNDLE DESTINATION ${CMAKE_INSTALL_BINDIR})

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
foreach(file IN LISTS SRC)
  get_filename_component(folder "${file}" PATH)
  string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/src/"     "" folder "${folder}")
  string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/include/" "" folder "${folder}")
  source_group("${folder}" FILES "${file}")
endforeach()

# Start Visual Studio's debugger in project root
if(MSVC)
  set_property(TARGET slrbs PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
endif()