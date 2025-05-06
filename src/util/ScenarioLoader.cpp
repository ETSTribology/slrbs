#include "util/ScenarioLoader.h"
#include "collision/Geometry.h"
#include "rigidbody/RigidBody.h"
#include "joint/Hinge.h"
#include "joint/Spherical.h"
#include "util/MeshAssets.h"
#include "util/VisualProperties.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>

// JSON parser library
#include <nlohmann/json.hpp>
#include <algorithm>
using json = nlohmann::json;

std::string ScenarioLoader::getScenariosPath()
{
    // Try common paths
    std::vector<std::string> paths = {
        "./resources/scenarios/",
        "../resources/scenarios/",
        "../../resources/scenarios/",
        "./",
        "../"};

    for (const auto &path : paths)
    {
        if (std::filesystem::exists(path))
        {
            return path;
        }
    }

    // Default to current directory
    return "./";
}

std::vector<std::string> ScenarioLoader::listAvailableScenarios()
{
    std::vector<std::string> scenarios;

    try
    {
        std::string scenariosPath = getScenariosPath();
        std::cout << "Searching for scenarios in: " << scenariosPath << std::endl;

        // List all JSON files in scenarios directory
        for (const auto &entry : std::filesystem::directory_iterator(scenariosPath))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".json")
            {
                // Add filename to the list
                scenarios.push_back(entry.path().filename().string());
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error listing scenarios: " << e.what() << std::endl;
    }

    return scenarios;
}

bool ScenarioLoader::loadFromFile(RigidBodySystem &system, const std::string &filename)
{
    try
    {
        // First try direct path
        std::ifstream file(filename);

        // If file doesn't exist, try with scenarios path
        if (!file.is_open())
        {
            std::string fullPath = getScenariosPath() + filename;
            std::cout << "Trying to open scenario from: " << fullPath << std::endl;
            file.open(fullPath);

            if (!file.is_open())
            {
                std::cerr << "Failed to open file: " << filename << std::endl;
                return false;
            }
        }

        // Read JSON content
        std::stringstream buffer;
        buffer << file.rdbuf();

        return parseScenario(system, buffer.str());
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading scenario from JSON: " << e.what() << std::endl;
        return false;
    }
}

bool ScenarioLoader::parseScenario(RigidBodySystem &system, const std::string &jsonContent)
{
    try
    {
        // Parse JSON
        json scenario = json::parse(jsonContent);

        std::cout << "Loading scenario: " << scenario["name"].get<std::string>() << std::endl;

        // Clear existing system - but NOT the visualization structures
        system.clear();

        // Clear previous visual properties
        g_visualProperties.clear();

        // Map to store bodies by ID for joint creation
        std::map<int, RigidBody *> bodyMap;

        // Load bodies
        int bodyIndex = 0;
        for (const auto &bodyJson : scenario["bodies"])
        {
            RigidBody *body = nullptr;

            // Create geometry
            Geometry *geometry = nullptr;
            if (bodyJson.contains("geometry"))
            {
                const auto &geo = bodyJson["geometry"];
                std::string type = geo["type"];

                if (type == "sphere")
                {
                    float radius = geo["radius"];
                    geometry = new Sphere(radius);
                    body = new RigidBody(bodyJson["mass"], geometry, createSphere(radius));
                }
                else if (type == "box")
                {
                    Eigen::Vector3f dim(
                        geo["dimensions"]["x"],
                        geo["dimensions"]["y"],
                        geo["dimensions"]["z"]);
                    geometry = new Box(dim);
                    body = new RigidBody(bodyJson["mass"], geometry, createBox(dim));
                }
                else if (type == "cylinder")
                {
                    float height = geo["height"];
                    float radius = geo["radius"];
                    geometry = new Cylinder(height, radius);
                    body = new RigidBody(bodyJson["mass"], geometry, createCylinder(16, radius, height));
                }
                else if (type == "plane")
                {
                    Eigen::Vector3f point(
                        geo["point"]["x"],
                        geo["point"]["y"],
                        geo["point"]["z"]);
                    Eigen::Vector3f normal(
                        geo["normal"]["x"],
                        geo["normal"]["y"],
                        geo["normal"]["z"]);
                    geometry = new Plane(point, normal);
                    body = new RigidBody(bodyJson["mass"], geometry, ""); // Planes don't have mesh visualization
                }
            }

            if (!body)
            {
                std::cerr << "Failed to create body from JSON" << std::endl;
                continue;
            }

            // Set properties
            body->fixed = bodyJson["fixed"];

            // Position
            body->x = Eigen::Vector3f(
                bodyJson["position"]["x"],
                bodyJson["position"]["y"],
                bodyJson["position"]["z"]);

            // Orientation
            body->q = Eigen::Quaternionf(
                bodyJson["orientation"]["w"],
                bodyJson["orientation"]["x"],
                bodyJson["orientation"]["y"],
                bodyJson["orientation"]["z"]);

            // Linear velocity if present
            if (bodyJson.contains("linear_velocity"))
            {
                body->xdot = Eigen::Vector3f(
                    bodyJson["linear_velocity"]["x"],
                    bodyJson["linear_velocity"]["y"],
                    bodyJson["linear_velocity"]["z"]);
            }

            // Angular velocity if present
            if (bodyJson.contains("angular_velocity"))
            {
                body->omega = Eigen::Vector3f(
                    bodyJson["angular_velocity"]["x"],
                    bodyJson["angular_velocity"]["y"],
                    bodyJson["angular_velocity"]["z"]);
            }

            // Store visual properties for later use
            if (bodyJson.contains("visual") && body->mesh)
            {
                const auto &visual = bodyJson["visual"];

                // Basic properties with defaults
                float r = 0.5f, g = 0.5f, b = 0.5f;
                float transparency = 0.0f;
                bool smoothShade = false;
                float edgeWidth = 1.0f;

                // Texture properties with defaults
                bool showTextureSpace = false;
                std::string textureParamName = "uv_coords";
                std::optional<std::string> texturePath = std::nullopt;

                // Material properties with defaults
                float shininess = 0.0f;
                float reflectivity = 0.0f;

                // Visualization options with defaults
                bool showWireframe = false;
                bool showNormals = false;
                float normalLength = 0.1f;
                bool showBoundingBox = false;

                // Debug visualization with defaults
                bool showContactPoints = false;
                bool showInertiaEllipsoid = false;
                float debugColorR = 1.0f, debugColorG = 1.0f, debugColorB = 0.0f;

                // Basic properties
                if (visual.contains("color"))
                {
                    r = visual["color"]["r"];
                    g = visual["color"]["g"];
                    b = visual["color"]["b"];
                }

                if (visual.contains("transparency"))
                {
                    transparency = visual["transparency"];
                }

                if (visual.contains("smooth_shade"))
                {
                    smoothShade = visual["smooth_shade"];
                }

                if (visual.contains("edge_width"))
                {
                    edgeWidth = visual["edge_width"];
                }

                // Parse texture properties if present
                if (visual.contains("texture"))
                {
                    const auto &texture = visual["texture"];

                    if (texture.contains("show_texture_space"))
                    {
                        showTextureSpace = texture["show_texture_space"];
                    }

                    if (texture.contains("param_name"))
                    {
                        textureParamName = texture["param_name"];
                    }

                    if (texture.contains("path"))
                    {
                        texturePath = texture["path"];
                    }
                }

                // Parse material properties if present
                if (visual.contains("material"))
                {
                    const auto &material = visual["material"];

                    if (material.contains("shininess"))
                    {
                        shininess = material["shininess"];
                    }

                    if (material.contains("reflectivity"))
                    {
                        reflectivity = material["reflectivity"];
                    }
                }

                // Parse visualization options if present
                if (visual.contains("visualization"))
                {
                    const auto &viz = visual["visualization"];

                    if (viz.contains("wireframe"))
                    {
                        showWireframe = viz["wireframe"];
                    }

                    if (viz.contains("normals"))
                    {
                        showNormals = viz["normals"];
                    }

                    if (viz.contains("normal_length"))
                    {
                        normalLength = viz["normal_length"];
                    }

                    if (viz.contains("bounding_box"))
                    {
                        showBoundingBox = viz["bounding_box"];
                    }
                }

                // Parse debug visualization if present
                if (visual.contains("debug"))
                {
                    const auto &debug = visual["debug"];

                    if (debug.contains("contact_points"))
                    {
                        showContactPoints = debug["contact_points"];
                    }

                    if (debug.contains("inertia_ellipsoid"))
                    {
                        showInertiaEllipsoid = debug["inertia_ellipsoid"];
                    }

                    if (debug.contains("color"))
                    {
                        debugColorR = debug["color"]["r"];
                        debugColorG = debug["color"]["g"];
                        debugColorB = debug["color"]["b"];
                    }
                }

                // Add all the properties to the visual properties manager
                g_visualProperties.addBodyProperties(
                    bodyIndex, r, g, b, transparency, smoothShade, edgeWidth,
                    showTextureSpace, textureParamName, texturePath,
                    shininess, reflectivity,
                    showWireframe, showNormals, normalLength, showBoundingBox,
                    showContactPoints, showInertiaEllipsoid,
                    debugColorR, debugColorG, debugColorB);

                body->visualProperties["colorR"] = r;
                body->visualProperties["colorG"] = g;
                body->visualProperties["colorB"] = b;
                body->visualProperties["transparency"] = transparency;
                body->visualProperties["smoothShade"] = smoothShade ? 1.0f : 0.0f;
                body->visualProperties["edgeWidth"] = edgeWidth;
                body->visualProperties["shininess"] = shininess;
                body->visualProperties["reflectivity"] = reflectivity;
                body->visualProperties["showWireframe"] = showWireframe ? 1.0f : 0.0f;
                body->visualProperties["showNormals"] = showNormals ? 1.0f : 0.0f;
                body->visualProperties["normalLength"] = normalLength;
                body->visualProperties["showBoundingBox"] = showBoundingBox ? 1.0f : 0.0f;
            }

            // Add body to the bodyMap using ID if available
            if (bodyJson.contains("id"))
            {
                int id = bodyJson["id"];
                bodyMap[id] = body;
            }

            // Add body to the system
            system.addBody(body);
            bodyIndex++;
        }

        // Process joints if present
        if (scenario.contains("joints"))
        {
            for (const auto &jointJson : scenario["joints"])
            {
                // Get bodies
                int body0Id = jointJson["body0_id"];
                int body1Id = jointJson["body1_id"];

                if (bodyMap.find(body0Id) == bodyMap.end() || bodyMap.find(body1Id) == bodyMap.end())
                {
                    std::cerr << "Invalid body IDs in joint definition" << std::endl;
                    continue;
                }

                std::string type = jointJson["type"];
                Joint *joint = nullptr;

                if (type == "hinge")
                {
                    Eigen::Vector3f r0(
                        jointJson["body0_offset"]["x"],
                        jointJson["body0_offset"]["y"],
                        jointJson["body0_offset"]["z"]);

                    Eigen::Vector3f r1(
                        jointJson["body1_offset"]["x"],
                        jointJson["body1_offset"]["y"],
                        jointJson["body1_offset"]["z"]);

                    Eigen::Quaternionf q0(
                        jointJson["body0_orientation"]["w"],
                        jointJson["body0_orientation"]["x"],
                        jointJson["body0_orientation"]["y"],
                        jointJson["body0_orientation"]["z"]);

                    Eigen::Quaternionf q1(
                        jointJson["body1_orientation"]["w"],
                        jointJson["body1_orientation"]["x"],
                        jointJson["body1_orientation"]["y"],
                        jointJson["body1_orientation"]["z"]);

                    joint = new Hinge(bodyMap[body0Id], bodyMap[body1Id], r0, q0, r1, q1);
                }
                else if (type == "spherical")
                {
                    Eigen::Vector3f r0(
                        jointJson["body0_offset"]["x"],
                        jointJson["body0_offset"]["y"],
                        jointJson["body0_offset"]["z"]);

                    Eigen::Vector3f r1(
                        jointJson["body1_offset"]["x"],
                        jointJson["body1_offset"]["y"],
                        jointJson["body1_offset"]["z"]);

                    joint = new Spherical(bodyMap[body0Id], bodyMap[body1Id], r0, r1);
                }

                if (joint)
                {
                    system.addJoint(joint);
                }
            }
        }

        // Apply physics settings if available
        if (scenario.contains("physics"))
        {
            const auto &physics = scenario["physics"];

            if (physics.contains("solver_iterations"))
            {
                system.setSolverIterations(physics["solver_iterations"]);
            }

            // Add gravity handling
            if (physics.contains("gravity"))
            {
                // Get gravity vector from JSON
                Eigen::Vector3f gravity(
                    physics["gravity"]["x"],
                    physics["gravity"]["y"],
                    physics["gravity"]["z"]);

                // Set gravity in the rigid body system
                system.setGravity(gravity);

                std::cout << "Set gravity to: [" << gravity.x() << ", "
                          << gravity.y() << ", " << gravity.z() << "]" << std::endl;
            }
        }

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error parsing scenario JSON: " << e.what() << std::endl;
        return false;
    }
}