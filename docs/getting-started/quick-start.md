# Quick Start Guide

This guide will help you get started with SLRBS quickly. We'll cover how to set up a basic scene, run a simulation, and visualize the results.

## Loading a Pre-defined Scenario

SLRBS comes with several built-in scenarios that you can use to get started quickly.

```cpp
#include "viewer/SimViewer.h"
#include "scenarios/Scenarios.h"

int main(int argc, char* argv[]) {
    // Initialize the simulation viewer
    slrbs::SimViewer viewer;
    viewer.initialize();
    
    // Load a predefined scenario (0-6)
    // 0: Sphere on box
    // 1: Marble box
    // 2: Swinging boxes
    // 3: Stack
    // 4: Cylinder on plane
    // 5: Car scene
    // 6: Rope bridge
    viewer.loadScenario(6); // Load rope bridge scenario
    
    // Start the simulation
    viewer.run();
    
    return 0;
}
```

## Creating a Custom Scenario

You can also create a custom scenario programmatically:

```cpp
#include "viewer/SimViewer.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include "collision/Geometry.h"
#include "utils/MeshAssets.h"

int main(int argc, char* argv[]) {
    // Initialize the simulation viewer
    slrbs::SimViewer viewer;
    viewer.initialize();
    
    // Get reference to rigid body system
    RigidBodySystem& system = viewer.getRigidBodySystem();
    system.clear();
    
    // Create a ground plane
    RigidBody* ground = new RigidBody(1.0f, new Plane({0, 0, 0}, {0, 1, 0}), "");
    ground->fixed = true;
    system.addBody(ground);
    
    // Create a box
    const Eigen::Vector3f boxDim(1.0f, 1.0f, 1.0f);
    RigidBody* box = new RigidBody(1.0f, new Box(boxDim), createBox(boxDim));
    box->x = {0, 5, 0}; // Position the box 5 units above ground
    box->mesh->setSurfaceColor({0.8f, 0.2f, 0.2f}); // Set red color
    system.addBody(box);
    
    // Save initial state for reset
    viewer.save();
    
    // Start the simulation
    viewer.run();
    
    return 0;
}
```

## Loading a Scenario from JSON

SLRBS also supports loading scenarios from JSON files:

```cpp
#include "viewer/SimViewer.h"
#include "scenarios/ScenarioLoader.h"
#include "rigidbody/RigidBodySystem.h"

int main(int argc, char* argv[]) {
    // Initialize the simulation viewer
    slrbs::SimViewer viewer;
    viewer.initialize();
    
    // Load a scenario from JSON file
    slrbs::ScenarioLoader::loadScenarioFromFile(
        viewer.getRigidBodySystem(), 
        "resources/scenarios/rope_bridge.json"
    );
    
    // Save initial state for reset
    viewer.save();
    
    // Start the simulation
    viewer.run();
    
    return 0;
}
```

## Interacting with the Simulation

Once the simulation is running, you can interact with it:

- **Camera Controls**:
  - Left mouse button: Rotate camera
  - Right mouse button: Pan camera
  - Mouse wheel: Zoom in/out

- **Simulation Controls**:
  - Pause/Resume: Click the "Pause" checkbox
  - Single step: Click the "Step once" button
  - Reset: Click the "Reset" button

## Customizing Solver Settings

You can customize the physics solver settings:

```cpp
// Select solver type
viewer.getRigidBodySystem().solverId = 0; // 0: PGS, 1: CG, 2: CR, 3: BPP

// Set solver iterations
viewer.getRigidBodySystem().solverIter = 50;

// Set friction coefficient
Contact::mu = 0.5f; // Static friction coefficient

// Enable/disable adaptive time stepping
viewer.setAdaptiveTimesteps(true);
viewer.setAlpha(0.5f); // Timestep scaling factor
```

## Next Steps

- Learn how to [create more complex scenarios](../user-guide/creating-scenarios.md)
- Explore [material and texture options](../user-guide/materials.md)
- Understand [joints and constraints](../technical/joints/overview.md)
- Study the [JSON file format](../file-formats/scenario-json.md) for defining scenarios
