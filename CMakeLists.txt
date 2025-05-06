cmake_minimum_required(VERSION 3.23)
project(slrbs
        VERSION 0.1.0
        DESCRIPTION "SLRBS"
        HOMEPAGE_URL "https://github.com/ETSim/slrbs"
        LANGUAGES CXX)

# -- User-configurable options (default values) --
option(BUILD_TESTING        "Enable testing"                          ON)
option(USE_SYSTEM_QT        "Use system Qt5 installation"            ON)
option(USE_OPENCV           "Enable OpenCV support"                  OFF)
option(USE_TORCH            "Enable LibTorch support"                OFF)
option(USE_OPENMP           "Enable OpenMP"                          ON)
option(USE_OPENMP_SIMD      "Enable OpenMP SIMD directives"          ON)
option(USE_SIMD_INTRINSICS  "Enable low-level SIMD intrinsics"       ON)
option(USE_PYBIND           "Enable Python bindings via pybind11"    ON)
option(USE_CCACHE           "Enable C/C++ compiler caching with ccache" OFF)

# -- Include user options and suppress developer warnings --
include(cmake/Options.cmake)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS TRUE CACHE INTERNAL "Suppress CMake dev warnings")
cmake_policy(SET CMP0169 OLD)

# -- CPM.cmake (FetchContent-based package manager) --
set(CPM_DOWNLOAD_VERSION 0.38.1)
set(CPM_DOWNLOAD_LOCATION "${CMAKE_BINARY_DIR}/cmake/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
if(NOT EXISTS ${CPM_DOWNLOAD_LOCATION})
    message(STATUS "Downloading CPM.cmake to ${CPM_DOWNLOAD_LOCATION}")
    file(DOWNLOAD
            https://github.com/cpm-cmake/CPM.cmake/releases/download/v${CPM_DOWNLOAD_VERSION}/CPM.cmake
            ${CPM_DOWNLOAD_LOCATION}
            SHOW_PROGRESS
    )
endif()
include(${CPM_DOWNLOAD_LOCATION})

# -- CPM recipes directory --
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/cpm/recipes)
file(GLOB CPM_RECIPE_FILES "${CMAKE_SOURCE_DIR}/cpm/recipes/*.cmake")
foreach(_recipe IN LISTS CPM_RECIPE_FILES)
    message(STATUS "Including recipe: ${_recipe}")
    include(${_recipe})
endforeach()

# -- Polyscope integration via CPM recipe --
if(polyscope_ADDED)
    message(STATUS "🎉 polyscope v${polyscope_VERSION} added via CPM")
    list(APPEND EXTRA_LIBS ${polyscope_LIBRARIES})
    list(APPEND INCLUDE_DIRS ${polyscope_INCLUDE_DIRS})
endif()

# -- Pybind11 for Python bindings --
if(USE_PYBIND)
    CPMAddPackage(
            NAME pybind11
            GITHUB_REPOSITORY pybind/pybind11
            VERSION 2.10.4
    )
    if(pybind11_ADDED)
        message(STATUS "🎉 pybind11 v${pybind11_VERSION} added via CPM")
    endif()
endif()

# -- Global settings --
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)

# -- Testing --
if(BUILD_TESTING)
    enable_testing()
    include(CTest)
endif()

# -- Compiler warnings & optional ccache --
if(USE_CCACHE)
    find_program(CCACHE_PROGRAM ccache)
    if(CCACHE_PROGRAM)
        message(STATUS "ccache found: ${CCACHE_PROGRAM}")
        set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
        set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    endif()
endif()

if(MSVC)
    add_compile_options(/W4 /permissive- /Zc:preprocessor /MP /wd4267 /wd4244)
else()
    add_compile_options(-Wall -Wextra -pedantic -Wshadow -Wconversion)
endif()

if(APPLE)
    add_compile_definitions(GL_SILENCE_DEPRECATION)
endif()

# -- Qt5 integration --
if(USE_SYSTEM_QT)
    find_package(Qt5 COMPONENTS Core Gui OpenGL REQUIRED)
endif()

# -- OpenCV integration --
if(USE_OPENCV)
    CPMAddPackage(
            NAME OpenCV
            GITHUB_REPOSITORY opencv/opencv
            GIT_TAG 4.8.0
            OPTIONS
            "BUILD_EXAMPLES OFF"
            "BUILD_TESTS OFF"
            "BUILD_PERF_TESTS OFF"
            "BUILD_opencv_apps OFF"
            "BUILD_DOCS OFF"
            "WITH_CUDA OFF"
            "WITH_EIGEN ON"
            "BUILD_opencv_python OFF"
            "WITH_OPENCL OFF"
            "WITH_FFMPEG OFF"
    )
    if(OpenCV_ADDED)
        message(STATUS "🎉 OpenCV v${OpenCV_VERSION} added via CPM")
        list(APPEND EXTRA_LIBS ${OpenCV_LIBS})
        list(APPEND INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
    endif()
endif()

# -- LibTorch integration --
if(USE_TORCH)
    CPMAddPackage(
            NAME Torch
            URL https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
            DOWNLOAD_ONLY YES
    )
    if(Torch_ADDED)
        set(Torch_DIR "${Torch_SOURCE_DIR}/share/cmake/Torch")
        find_package(Torch REQUIRED)
        message(STATUS "🔥 LibTorch v${TORCH_VERSION} added via CPM")
        list(APPEND EXTRA_LIBS ${TORCH_LIBRARIES})
    endif()
endif()

# -- Eigen3 integration --
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
if(Eigen3_FOUND)
    message(STATUS "🎉 Eigen3 v${Eigen3_VERSION} found at ${Eigen3_DIR}")
    list(APPEND INCLUDE_DIRS ${Eigen3_INCLUDE_DIR})
else()
    message(FATAL_ERROR "Eigen3 >= 3.4 not found—please install it or point EIGEN3_ROOT to your Eigen install.")
endif()

# -- OpenGL --
find_package(OpenGL REQUIRED)
set(OPENGL_LIB $<IF:$<TARGET_EXISTS:OpenGL::GL>,OpenGL::GL,opengl32>)

# -- Parallel & SIMD support --
if(USE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        add_compile_definitions(USE_OPENMP)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        if(USE_OPENMP_SIMD)
            add_compile_definitions(USE_OPENMP_SIMD)
            if(MSVC)
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /openmp:experimental")
            else()
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp-simd")
            endif()
        endif()
        list(APPEND EXTRA_LIBS OpenMP::OpenMP_CXX)
    endif()
endif()

if(USE_SIMD_INTRINSICS)
    add_compile_definitions(USE_SIMD_INTRINSICS)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-mavx2" HAS_AVX2)
    if(HAS_AVX2)
        add_compile_options(-mavx2)
    elseif(CHECK_CXX_COMPILER_FLAG("-msse4.2" HAS_SSE42))
        add_compile_options(-msse4.2)
    else()
        add_compile_options(-msse2)
    endif()
endif()

# -- Source & executable --
file(GLOB_RECURSE SOURCES_CPP CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE SOURCES_H CONFIGURE_DEPENDS include/*.h)
set(SOURCES ${SOURCES_CPP} ${SOURCES_H})
add_executable(slrbs ${SOURCES})

target_include_directories(slrbs PRIVATE include ${INCLUDE_DIRS})

# -- Link libraries --
# Note: polyscope libs appended to EXTRA_LIBS above
target_link_libraries(slrbs PRIVATE
        ${OPENGL_LIB}
        Qt5::Core Qt5::Gui Qt5::OpenGL
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        pybind11::module
        ${EXTRA_LIBS}
)
set_property(TARGET slrbs PROPERTY CXX_STANDARD 17)

# -- Install rules --
include(GNUInstallDirs)
install(TARGETS slrbs RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# -- Post-build resource copying --
set(RESOURCE_DIRS scenarios models ml_models)
foreach(_dir IN LISTS RESOURCE_DIRS)
    add_custom_command(TARGET slrbs POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/resources/${_dir}"
            "$<TARGET_FILE_DIR:slrbs>/resources/${_dir}"
    )
endforeach()

# -- IDE source grouping --
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
source_group(TREE ${CMAKE_SOURCE_DIR}/src PREFIX "src" FILES ${SOURCES_CPP})
source_group(TREE ${CMAKE_SOURCE_DIR}/include PREFIX "include" FILES ${SOURCES_H})
