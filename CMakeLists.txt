cmake_minimum_required(VERSION 3.23)
project(slrbs
VERSION         0.1.0
DESCRIPTION     "SLRBS"
HOMEPAGE_URL    "https://github.com/ETSim/slrbs"
LANGUAGES       CXX
)

# -- Project-wide settings ----------------------------------------------------
set(CMAKE_CXX_STANDARD            17)
set(CMAKE_CXX_STANDARD_REQUIRED   ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -- Experimental GLM extensions ----------------------------------------------
add_compile_definitions(GLM_ENABLE_EXPERIMENTAL)

# -- Options -------------------------------------------------------------------
option(BUILD_TESTING       "Enable testing"                       ON)
option(USE_SYSTEM_QT       "Use system Qt5 installation"         ON)
option(USE_OPENCV          "Enable OpenCV support"               OFF)
option(USE_TORCH           "Enable LibTorch support"             OFF)
option(USE_OPENMP          "Enable OpenMP"                       ON)
option(USE_OPENMP_SIMD     "Enable OpenMP SIMD directives"       ON)
option(USE_SIMD_INTRINSICS "Enable low-level SIMD intrinsics"    ON)
option(USE_PYBIND          "Enable Python bindings via pybind11" ON)
option(USE_CCACHE          "Enable compiler caching with ccache" OFF)

# -- Suppress developer warnings ---------------------------------------------
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS TRUE CACHE INTERNAL "Suppress CMake dev warnings")
cmake_policy(SET CMP0169 OLD)

# -- Include CPM recipes ------------------------------------------------------
include(cmake/CPMBootstrap.cmake)
include(cmake/CPMRecipe_GLM.cmake)
include(cmake/CPMRecipe_Polyscope.cmake)
include(cmake/CPMRecipe_Pybind11.cmake)
include(cmake/CPMRecipe_OpenCV.cmake)
include(cmake/CPMRecipe_Torch.cmake)

# -- Project options and flags ------------------------------------------------
include(cmake/Options.cmake)
if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)

# -- Testing support ----------------------------------------------------------
if(BUILD_TESTING)
enable_testing()
include(CTest)
endif()

# -- ccache integration -------------------------------------------------------
if(USE_CCACHE)
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
endif()
endif()

# -- Compiler warnings --------------------------------------------------------
if(MSVC)
add_compile_options(/W4 /permissive- /Zc:preprocessor /MP /wd4267 /wd4244)
else()
add_compile_options(-Wall -Wextra -pedantic -Wshadow -Wconversion)
endif()

# -- Platform-specific definitions --------------------------------------------
if(APPLE)
add_compile_definitions(GL_SILENCE_DEPRECATION)
endif()

# -- Qt integration -----------------------------------------------------------
if(USE_SYSTEM_QT)
find_package(Qt5 COMPONENTS Core Gui OpenGL Widgets REQUIRED)
endif()

# -- OpenCV integration -------------------------------------------------------
if(USE_OPENCV)
find_package(OpenCV REQUIRED)
endif()

# -- LibTorch integration -----------------------------------------------------
if(USE_TORCH)
find_package(Torch REQUIRED)
endif()

# -- Eigen3 integration -------------------------------------------------------
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# -- OpenGL -------------------------------------------------------------------
find_package(OpenGL REQUIRED)
set(OPENGL_LIB $<IF:$<TARGET_EXISTS:OpenGL::GL>,OpenGL::GL,opengl32>)

# -- OpenMP & SIMD ------------------------------------------------------------
if(USE_OPENMP)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
add_compile_definitions(USE_OPENMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
if(USE_OPENMP_SIMD)
add_compile_definitions(USE_OPENMP_SIMD)
endif()
endif()
endif()
if(USE_SIMD_INTRINSICS)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mavx2" HAS_AVX2)
if(HAS_AVX2)
add_compile_options(-mavx2)
elseif(CHECK_CXXCompilerFlag("-msse4.2" HAS_SSE42))
add_compile_options(-msse4.2)
else()
add_compile_options(-msse2)
endif()
endif()

# -- Source files and executable ---------------------------------------------
file(GLOB_RECURSE SOURCES_CPP CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE SOURCES_H   CONFIGURE_DEPENDS include/*.h)
add_executable(slrbs ${SOURCES_CPP} ${SOURCES_H})

target_include_directories(slrbs PRIVATE
include
${polyscope_INCLUDE_DIRS}
${Eigen3_INCLUDE_DIRS}
$<TARGET_PROPERTY:OpenCV,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(slrbs PRIVATE
${OPENGL_LIB}
Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::Widgets
$<$<BOOL:${USE_OPENCV}>:OpenCV::opencv_core OpenCV::opencv_imgproc>
$<$<BOOL:${USE_TORCH}>:Torch::Torch>
pybind11::pybind11
Eigen3::Eigen
OpenMP::OpenMP_CXX
polyscope::polyscope
)

# -- Install rules -----------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS slrbs RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# -- Post-build resource copying ---------------------------------------------
set(RESOURCE_DIRS scenarios models ml_models)
foreach(_dir IN LISTS RESOURCE_DIRS)
add_custom_command(TARGET slrbs POST_BUILD
COMMAND "${CMAKE_COMMAND}" -E copy_directory
"${CMAKE_SOURCE_DIR}/resources/${_dir}"
"$<TARGET_FILE_DIR:slrbs>/resources/${_dir}"
)
endforeach()

# -- IDE source grouping -----------------------------------------------------
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
source_group(TREE ${CMAKE_SOURCE_DIR}/src PREFIX "src" FILES ${SOURCES_CPP})
source_group(TREE ${CMAKE_SOURCE_DIR}/include PREFIX "include" FILES ${SOURCES_H})
