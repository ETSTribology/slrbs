# cmake/Options.cmake – User‑configurable options for SLRBS

option(BUILD_TESTING        "Enable testing"                          ON)
option(USE_QT "Enable Qt support" ON)
option(USE_SYSTEM_QT        "Use system Qt5 installation"            ON)
option(USE_OPENCV           "Enable OpenCV support"                  OFF)
option(USE_TORCH            "Enable LibTorch support"                OFF)
option(USE_OPENMP           "Enable OpenMP"                          ON)
option(USE_OPENMP_SIMD      "Enable OpenMP SIMD directives"          ON)
option(USE_SIMD_INTRINSICS  "Enable low‑level SIMD intrinsics"       ON)
option(USE_PYBIND           "Enable Python bindings via pybind11"    ON)
option(USE_CCACHE        "Enable C/C++ compiler caching with ccache"  OFF)
