# Scenario JSON Format

SLRBS uses JSON files to describe simulation scenarios. This page documents the structure and properties of these files.

## Overview

The scenario JSON format allows you to define:

- Rigid bodies with their properties (mass, position, geometry, etc.)
- Joints connecting bodies
- Visual properties (colors, textures, materials)
- Physics settings (gravity, time step, solver iterations)

## Basic Structure

A scenario JSON file has the following top-level structure:

```json
{
    "name": "Scenario Name",
    "description": "A brief description of the scenario",
    "version": 1.0,
    "bodies": [...],
    "joints": [...],
    "physics": {...}
}
```

## Bodies

The `bodies` section contains an array of rigid body definitions:

```json
{
    "id": 0,
    "mass": 1.0,
    "fixed": true,
    "position": {
        "x": -6.0,
        "y": 3.0,
        "z": 0.0
    },
    "orientation": {
        "w": 1.0,
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "linear_velocity": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "angular_velocity": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "geometry": {
        "type": "box",
        "dimensions": {
            "x": 1.0,
            "y": 1.0,
            "z": 1.0
        }
    },
    "visual": {
        "color": {
            "r": 1.0,
            "g": 0.5,
            "b": 0.2
        },
        "material": "clay",
        "texture": "wood.png",
        "transparency": 0.0,
        "smooth_shade": true,
        "edge_width": 0.0
    }
}
```

### Body Properties

| Property | Type | Description |
|----------|------|-------------|
| id | integer | Unique identifier for the body |
| mass | float | Mass of the body |
| fixed | boolean | Whether the body is static (immovable) |
| position | object | 3D position coordinates |
| orientation | object | Quaternion (w, x, y, z) |
| linear_velocity | object | Initial linear velocity |
| angular_velocity | object | Initial angular velocity |
| geometry | object | Geometry definition |
| visual | object | Visual properties |

### Geometry Types

The following geometry types are supported:

#### Box

```json
"geometry": {
    "type": "box",
    "dimensions": {
        "x": 1.0,
        "y": 1.0,
        "z": 1.0
    }
}
```

#### Sphere

```json
"geometry": {
    "type": "sphere",
    "radius": 0.5
}
```

#### Cylinder

```json
"geometry": {
    "type": "cylinder",
    "height": 2.0,
    "radius": 0.5
}
```

#### Plane

```json
"geometry": {
    "type": "plane",
    "point": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "normal": {
        "x": 0.0,
        "y": 1.0,
        "z": 0.0
    }
}
```

### Visual Properties

| Property | Type | Description |
|----------|------|-------------|
| color | object | RGB color values (0.0-1.0) |
| material | string | Material name |
| texture | string | Path to texture file |
| transparency | float | Transparency (0.0-1.0) |
| smooth_shade | boolean | Whether to use smooth shading |
| edge_width | float | Width of wireframe edges |

## Joints

The `joints` section contains an array of joint definitions:

```json
{
    "id": 0,
    "type": "spherical",
    "body0_id": 0,
    "body1_id": 1,
    "body0_offset": {
        "x": 0.5,
        "y": 0.0,
        "z": 0.0
    },
    "body1_offset": {
        "x": -0.5,
        "y": 0.0,
        "z": 0.0
    }
}
```

### Joint Types

#### Spherical Joint

```json
{
    "id": 0,
    "type": "spherical",
    "body0_id": 0,
    "body1_id": 1,
    "body0_offset": {
        "x": 0.5,
        "y": 0.0,
        "z": 0.0
    },
    "body1_offset": {
        "x": -0.5,
        "y": 0.0,
        "z": 0.0
    }
}
```

#### Hinge Joint

```json
{
    "id": 0,
    "type": "hinge",
    "body0_id": 0,
    "body1_id": 1,
    "body0_offset": {
        "x": 0.5,
        "y": 0.0,
        "z": 0.0
    },
    "body1_offset": {
        "x": -0.5,
        "y": 0.0,
        "z": 0.0
    },
    "body0_orientation": {
        "w": 1.0,
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "body1_orientation": {
        "w": 1.0,
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    }
}
```

## Physics Settings

The `physics` section defines global physics parameters:

```json
{
    "gravity": {
        "x": 0.0,
        "y": -9.81,
        "z": 0.0
    },
    "time_step": 0.01,
    "solver_iterations": 10
}
```

| Property | Type | Description |
|----------|------|-------------|
| gravity | object | Gravity vector |
| time_step | float | Simulation time step |
| solver_iterations | integer | Number of solver iterations |

## Example Scenario

Here's a complete example of a rope bridge scenario:

```json
{
    "name": "RopeBridge",
    "description": "Rope bridge constructed with boxes connected by spherical joints",
    "version": 1.0,
    "bodies": [
        {
            "id": 0,
            "mass": 1.0,
            "fixed": true,
            "position": {
                "x": -6.0,
                "y": 3.0,
                "z": 0.0
            },
            "orientation": {
                "w": 1.0,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            },
            "geometry": {
                "type": "box",
                "dimensions": {
                    "x": 0.5,
                    "y": 0.1,
                    "z": 1.0
                }
            },
            "visual": {
                "color": {
                    "r": 1.0,
                    "g": 1.0,
                    "b": 0.1
                },
                "transparency": 0.0,
                "smooth_shade": true,
                "edge_width": 0.0
            }
        },
        {
            "id": 20,
            "mass": 1.0,
            "fixed": true,
            "position": {
                "x": 6.0,
                "y": 3.0,
                "z": 0.0
            },
            "orientation": {
                "w": 1.0,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            },
            "geometry": {
                "type": "box",
                "dimensions": {
                    "x": 0.5,
                    "y": 0.1,
                    "z": 1.0
                }
            },
            "visual": {
                "color": {
                    "r": 1.0,
                    "g": 1.0,
                    "b": 0.1
                }
            }
        }
    ],
    "joints": [
        {
            "id": 0,
            "type": "spherical",
            "body0_id": 0,
            "body1_id": 1,
            "body0_offset": {
                "x": 0.3,
                "y": 0.0,
                "z": 0.5
            },
            "body1_offset": {
                "x": -0.3,
                "y": 0.0,
                "z": 0.5
            }
        }
    ],
    "physics": {
        "gravity": {
            "x": 0.0,
            "y": -9.81,
            "z": 0.0
        },
        "time_step": 0.01,
        "solver_iterations": 10
    }
}
```
