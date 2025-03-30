# SLRBS Component Management Platform Configuration
# Configure component-level build settings and options

# Project Component Options
option(BUILD_RENDERER "Build the renderer component" ON)
option(BUILD_VIEWER "Build the viewer component" ON)
option(BUILD_SCENARIOS "Build the built-in scenarios" ON)
option(BUILD_LOGGING "Build the logging components" ON)
option(USE_EIGEN_FROM_SYSTEM "Use system-installed Eigen instead of downloading" OFF)
option(ENABLE_TESTS "Build test suite" OFF)
option(ENABLE_BENCHMARKS "Build benchmarks" OFF)
option(BUILD_DOCUMENTATION "Build documentation" OFF)

# Performance Options
option(ENABLE_SIMD_OPTIMIZATIONS "Use SIMD optimizations where available" ON)
option(USE_VECTORIZATION "Enable Eigen vectorization" ON)
option(USE_OPENMP_ACCELERATION "Use OpenMP for multi-threading acceleration" ON)
option(USE_CUSTOM_MEMORY_ALLOCATOR "Use a custom memory allocator for better performance" OFF)
option(ENABLE_PROFILING "Enable performance profiling" OFF)
option(ENABLE_LTO "Enable Link Time Optimization" OFF)

# Debug Options
option(USE_ASSERTIONS "Enable assertion checking" ON)
option(ENABLE_DEBUG_RENDERING "Show debug visualizations in rendering" OFF)
option(ENABLE_MEMORY_TRACKING "Track memory allocations for debugging" OFF)

# Configure components based on options
if(BUILD_PYTHON_BINDINGS)
  message(STATUS "Python bindings enabled")
endif()

if(ENABLE_PROFILING)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-pg)
    add_link_options(-pg)
  endif()
endif()

if(USE_ASSERTIONS)
  add_compile_definitions(SLRBS_USE_ASSERTIONS)
else()
  add_compile_definitions(NDEBUG)
endif()

if(ENABLE_LTO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT IPO_SUPPORTED OUTPUT error)
  if(IPO_SUPPORTED)
    message(STATUS "IPO/LTO enabled")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
  else()
    message(STATUS "IPO/LTO not supported: <${error}>")
  endif()
endif()

if(NOT USE_VECTORIZATION)
  set(EIGEN_DONT_VECTORIZE ON)
endif()

if(ENABLE_DEBUG_RENDERING)
  add_compile_definitions(DEBUG_RENDERING)
endif()

# Platform-specific adjustments
if(WIN32)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

message(STATUS "SLRBS CMP configuration complete")
