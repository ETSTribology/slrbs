# Sheldon's Little Rigid-Body Simulator (SLRBS)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![CMake](https://img.shields.io/badge/CMake-3.15+-blue.svg)](https://cmake.org/)
[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey.svg)](https://github.com/yourusername/SLRBS)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](https://github.com/yourusername/SLRBS/pulls)

A simple, lightweight rigid body simulation engine. 

I developed this rigid body simulator primarily for the purposes of teaching. It has been used during my course work at the [École de technologie supérieure](https://www.etsmtl.ca/) for our [SIGGRAPH course](https://siggraphcontact.github.io/) on simulating contact. 

Please feel free to use it for your own projects.

## Features

- 3D rigid body dynamics simulation
- Various collision geometries (sphere, box, cylinder, plane)
- Multiple joint types (hinge, spherical, prismatic, universal, distance)
- JSON-based scenario loading and saving
- Intuitive API for scene creation
- Interactive visualization through Polyscope

## Getting started

The simulator can be configured and compiled using CMake. The following steps should work on Windows with Visual Studio 2022 installed:

```bash
mkdir build/
cd build/
cmake ..\CMakeLists.txt -G "Visual Studio 17 2022"
cmake --build .
```

For Linux or macOS:

```bash
mkdir build
cd build
cmake ..
make
```

## Creating scenarios

The file *include/rigidbody/Scenarios.h* shows some examples of how to create scenes using the various collision geometries and joint types.

### Available scenarios

SLRBS comes with several pre-built scenarios:

1. **MarbleBox** - A box filled with spherical marbles that interact with gravity
2. **SphereOnBox** - A simple sphere falling onto a box
3. **Swinging