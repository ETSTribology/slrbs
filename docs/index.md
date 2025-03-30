# SLRBS: Simple Lightweight Rigid Body Simulator

SLRBS is a high-performance rigid body physics simulator designed for real-time applications. It provides a comprehensive framework for simulating rigid body dynamics, collision detection, and constraint solving.

## Features

- Fast and efficient rigid body simulation
- Multiple constraint solvers (PGS, CG, CR, BPP)
- Various joint types (spherical, hinge, distance, prismatic, universal)
- Collision detection for different primitive types
- JSON-based scenario descriptions
- Material and texture support for visualization
- Polyscope-based visualization
- Extensible architecture

## Quick Links

- [Installation Guide](getting-started/installation.md)
- [Quick Start](getting-started/quick-start.md)
- [Scenario JSON Format](file-formats/scenario-json.md)
- [Solver Documentation](technical/solvers/overview.md)

## Example

Here's a simple example of creating a scenario with SLRBS:

```cpp
#include "viewer/SimViewer.h"
#include "scenarios/Scenarios.h"

int main(int argc, char* argv[]) {
    slrbs::SimViewer viewer;
    viewer.initialize();
    
    // Load a predefined scenario
    viewer.loadScenario("RopeBridge");
    
    // Run the simulation
    viewer.run();
    
    return 0;
}
```

## License

SLRBS is licensed under the MIT License. See the LICENSE file for more details.
