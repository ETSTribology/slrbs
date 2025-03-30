# Eigen (https://gitlab.com/libeigen/eigen)
# License: MPL 2.0
if(TARGET Eigen3::Eigen)
    return()
endif()

# Build options
option(EIGEN_WITH_MKL "Use Eigen with MKL" OFF)
option(EIGEN_DONT_VECTORIZE "Disable Eigen vectorization" OFF)
option(EIGEN_MPL2_ONLY "Enable Eigen MPL2 license only" OFF)
option(EIGEN_USE_BLAS "Use Eigen with BLAS" OFF)
option(EIGEN_USE_LAPACKE "Use Eigen with LAPACKE" OFF)
option(EIGEN_USE_THREADS "Enable multi-threading support in Eigen" ON)
option(EIGEN_NO_DEBUG "Disable Eigen's assertions" OFF)
option(EIGEN_FAST_MATH "Enable fast math optimizations" OFF)

message(STATUS "Third-party: creating target 'Eigen3::Eigen'")

include(CPM)
CPMAddPackage(
    NAME eigen
    GITLAB_REPOSITORY libeigen/eigen
    GIT_TAG 3.4.0
    DOWNLOAD_ONLY YES
)

add_library(Eigen3_Eigen INTERFACE)
add_library(Eigen3::Eigen ALIAS Eigen3_Eigen)

include(GNUInstallDirs)
target_include_directories(Eigen3_Eigen SYSTEM INTERFACE
    $<BUILD_INTERFACE:${eigen_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Apply base configuration options
if(EIGEN_MPL2_ONLY)
  target_compile_definitions(Eigen3_Eigen INTERFACE EIGEN_MPL2_ONLY)
endif()

if(EIGEN_DONT_VECTORIZE)
  target_compile_definitions(Eigen3_Eigen INTERFACE EIGEN_DONT_VECTORIZE)
endif()

if(EIGEN_USE_THREADS)
  find_package(Threads REQUIRED)
  target_link_libraries(Eigen3_Eigen INTERFACE Threads::Threads)
  target_compile_definitions(Eigen3_Eigen INTERFACE EIGEN_USE_THREADS)
endif()

if(EIGEN_NO_DEBUG)
  target_compile_definitions(Eigen3_Eigen INTERFACE EIGEN_NO_DEBUG)
endif()

if(EIGEN_FAST_MATH)
  target_compile_definitions(Eigen3_Eigen INTERFACE EIGEN_FAST_MATH)
endif()

# Platform specific optimizations
if(EIGEN_WITH_MKL)
    # TODO: Checks that, on 64bits systems, `mkl::mkl` is using the LP64 interface
    # (by looking at the compile definition of the target)
    include(mkl)
    target_link_libraries(Eigen3_Eigen INTERFACE mkl::mkl)
    target_compile_definitions(Eigen3_Eigen INTERFACE
        EIGEN_USE_MKL_ALL
        EIGEN_USE_LAPACKE_STRICT
    )
elseif(EIGEN_USE_BLAS)
    find_package(BLAS REQUIRED)
    
    # Platform-specific LAPACKE handling
    if(EIGEN_USE_LAPACKE)
        if(APPLE)
            find_library(LAPACKE lapacke PATHS
                "/opt/local/lib/lapack"
                "/opt/homebrew/opt/lapack/lib"
            )
        elseif(UNIX)
            find_library(LAPACKE lapacke PATHS
                "/usr/lib"
                "/usr/local/lib"
                "/usr/lib/x86_64-linux-gnu"
            )
        endif()
        
        if(NOT LAPACKE)
            message(WARNING "LAPACKE library not found (required for EIGEN_USE_LAPACKE)! "
                "Eigen will be built without LAPACKE support.")
            set(EIGEN_USE_LAPACKE OFF)
        else()
            message(STATUS "Found BLAS and LAPACKE. Enabling Eigen LAPACKE support.")
            target_link_libraries(Eigen3_Eigen INTERFACE
                ${BLAS_LIBRARIES} ${LAPACKE}
            )
            target_compile_definitions(Eigen3_Eigen INTERFACE
                EIGEN_USE_BLAS
                EIGEN_USE_LAPACKE_STRICT
            )
        endif()
    else()
        # Use just BLAS without LAPACKE
        target_link_libraries(Eigen3_Eigen INTERFACE ${BLAS_LIBRARIES})
        target_compile_definitions(Eigen3_Eigen INTERFACE EIGEN_USE_BLAS)
    endif()
endif()

# On Windows, enable natvis files to improve debugging experience
if(WIN32 AND eigen_SOURCE_DIR)
    target_sources(Eigen3_Eigen INTERFACE $<BUILD_INTERFACE:${eigen_SOURCE_DIR}/debug/msvc/eigen.natvis>)
endif()

# Install rules
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME eigen)
set_target_properties(Eigen3_Eigen PROPERTIES EXPORT_NAME Eigen)
install(DIRECTORY ${eigen_SOURCE_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS Eigen3_Eigen EXPORT Eigen_Targets)
install(EXPORT Eigen_Targets DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/eigen NAMESPACE Eigen3::)