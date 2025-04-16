#include "util/ScenarioLoader.h"
#include "collision/Geometry.h"
#include "rigidbody/RigidBody.h"
#include "joint/Hinge.h"
#include "joint/Spherical.h"
#include "util/MeshAssets.h"

#include <polyscope/polyscope.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>

// JSON parser library
#include <nlohmann/json.hpp>
using json = nlohmann::json;

std::string ScenarioLoader::getScenariosPath() {
    // Try common paths
    std::vector<std::string> paths = {
        "./scenarios/",
        "../scenarios/",
        "../../scenarios/",
        "./",
        "../"
    };
    
    for (const auto& path : paths) {
        if (std::filesystem::exists(path)) {
            return path;
        }
    }
    
    // Default to current directory
    return "./";
}

std::vector<std::string> ScenarioLoader::listAvailableScenarios() {
    std::vector<std::string> scenarios;
    
    try {
        std::string scenariosPath = getScenariosPath();
        
        // List all JSON files in scenarios directory
        for (const auto& entry : std::filesystem::directory_iterator(scenariosPath)) {
            if (entry.is_regular_file() && entry.path().extension() == ".json") {
                // Add filename to the list
                scenarios.push_back(entry.path().filename().string());
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error listing scenarios: " << e.what() << std::endl;
    }
    
    return scenarios;
}

bool ScenarioLoader::loadFromFile(RigidBodySystem& system, const std::string& filename) {
    try {
        // First try direct path
        std::ifstream file(filename);
        
        // If file doesn't exist, try with scenarios path
        if (!file.is_open()) {
            std::string fullPath = getScenariosPath() + filename;
            file.open(fullPath);
            
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
                return false;
            }
        }
        
        // Read JSON content
        std::stringstream buffer;
        buffer << file.rdbuf();
        
        return parseScenario(system, buffer.str());
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading scenario from JSON: " << e.what() << std::endl;
        return false;
    }
}

bool ScenarioLoader::parseScenario(RigidBodySystem& system, const std::string& jsonContent) {
    try {
        // Parse JSON
        json scenario = json::parse(jsonContent);
        
        std::cout << "Loading scenario: " << scenario["name"].get<std::string>() << std::endl;
        
        // Clear existing system
        system.clear();
        polyscope::removeAllStructures();
        
        // Map to store bodies by ID for joint creation
        std::map<int, RigidBody*> bodyMap;
        
        // Load bodies
        for (const auto& bodyJson : scenario["bodies"]) {
            RigidBody* body = nullptr;
            
            // Create geometry
            Geometry* geometry = nullptr;
            if (bodyJson.contains("geometry")) {
                const auto& geo = bodyJson["geometry"];
                std::string type = geo["type"];
                
                if (type == "sphere") {
                    float radius = geo["radius"];
                    geometry = new Sphere(radius);
                    body = new RigidBody(bodyJson["mass"], geometry, createSphere(radius));
                }
                else if (type == "box") {
                    Eigen::Vector3f dim(
                        geo["dimensions"]["x"],
                        geo["dimensions"]["y"],
                        geo["dimensions"]["z"]
                    );
                    geometry = new Box(dim);
                    body = new RigidBody(bodyJson["mass"], geometry, createBox(dim));
                }
                else if (type == "cylinder") {
                    float height = geo["height"];
                    float radius = geo["radius"];
                    geometry = new Cylinder(height, radius);
                    body = new RigidBody(bodyJson["mass"], geometry, createCylinder(16, radius, height));
                }
                else if (type == "plane") {
                    Eigen::Vector3f point(
                        geo["point"]["x"],
                        geo["point"]["y"],
                        geo["point"]["z"]
                    );
                    Eigen::Vector3f normal(
                        geo["normal"]["x"],
                        geo["normal"]["y"],
                        geo["normal"]["z"]
                    );
                    geometry = new Plane(point, normal);
                    body = new RigidBody(bodyJson["mass"], geometry, ""); // Planes don't have mesh visualization
                }
            }
            
            if (!body) {
                std::cerr << "Failed to create body from JSON" << std::endl;
                continue;
            }
            
            // Set properties
            body->fixed = bodyJson["fixed"];
            
            // Position
            body->x = Eigen::Vector3f(
                bodyJson["position"]["x"],
                bodyJson["position"]["y"],
                bodyJson["position"]["z"]
            );
            
            // Orientation
            body->q = Eigen::Quaternionf(
                bodyJson["orientation"]["w"],
                bodyJson["orientation"]["x"],
                bodyJson["orientation"]["y"],
                bodyJson["orientation"]["z"]
            );
            
            // Linear velocity if present
            if (bodyJson.contains("linear_velocity")) {
                body->xdot = Eigen::Vector3f(
                    bodyJson["linear_velocity"]["x"],
                    bodyJson["linear_velocity"]["y"],
                    bodyJson["linear_velocity"]["z"]
                );
            }
            
            // Angular velocity if present
            if (bodyJson.contains("angular_velocity")) {
                body->omega = Eigen::Vector3f(
                    bodyJson["angular_velocity"]["x"],
                    bodyJson["angular_velocity"]["y"],
                    bodyJson["angular_velocity"]["z"]
                );
            }
            
            // Visual properties
            if (bodyJson.contains("visual") && body->mesh) {
                const auto& visual = bodyJson["visual"];
                
                // Color
                if (visual.contains("color")) {
                    body->mesh->setSurfaceColor({
                        visual["color"]["r"],
                        visual["color"]["g"],
                        visual["color"]["b"]
                    });
                }
                
                // Transparency
                if (visual.contains("transparency")) {
                    body->mesh->setTransparency(visual["transparency"]);
                }
                
                // Smooth shading
                if (visual.contains("smooth_shade")) {
                    body->mesh->setSmoothShade(visual["smooth_shade"]);
                }
                
                // Edge width
                if (visual.contains("edge_width")) {
                    body->mesh->setEdgeWidth(visual["edge_width"]);
                }
            }
            
            // Add to system and map
            system.addBody(body);
            if (bodyJson.contains("id")) {
                bodyMap[bodyJson["id"]] = body;
            }
        }
        
        // Load joints if present
        if (scenario.contains("joints")) {
            for (const auto& jointJson : scenario["joints"]) {
                // Get bodies
                int body0Id = jointJson["body0_id"];
                int body1Id = jointJson["body1_id"];
                
                if (bodyMap.find(body0Id) == bodyMap.end() || bodyMap.find(body1Id) == bodyMap.end()) {
                    std::cerr << "Invalid body IDs in joint definition" << std::endl;
                    continue;
                }
                
                std::string type = jointJson["type"];
                Joint* joint = nullptr;
                
                if (type == "hinge") {
                    Eigen::Vector3f r0(
                        jointJson["body0_offset"]["x"],
                        jointJson["body0_offset"]["y"],
                        jointJson["body0_offset"]["z"]
                    );
                    
                    Eigen::Vector3f r1(
                        jointJson["body1_offset"]["x"],
                        jointJson["body1_offset"]["y"],
                        jointJson["body1_offset"]["z"]
                    );
                    
                    Eigen::Quaternionf q0(
                        jointJson["body0_orientation"]["w"],
                        jointJson["body0_orientation"]["x"],
                        jointJson["body0_orientation"]["y"],
                        jointJson["body0_orientation"]["z"]
                    );
                    
                    Eigen::Quaternionf q1(
                        jointJson["body1_orientation"]["w"],
                        jointJson["body1_orientation"]["x"],
                        jointJson["body1_orientation"]["y"],
                        jointJson["body1_orientation"]["z"]
                    );
                    
                    joint = new Hinge(bodyMap[body0Id], bodyMap[body1Id], r0, q0, r1, q1);
                }
                else if (type == "spherical") {
                    Eigen::Vector3f r0(
                        jointJson["body0_offset"]["x"],
                        jointJson["body0_offset"]["y"],
                        jointJson["body0_offset"]["z"]
                    );
                    
                    Eigen::Vector3f r1(
                        jointJson["body1_offset"]["x"],
                        jointJson["body1_offset"]["y"],
                        jointJson["body1_offset"]["z"]
                    );
                    
                    joint = new Spherical(bodyMap[body0Id], bodyMap[body1Id], r0, r1);
                }
                
                if (joint) {
                    system.addJoint(joint);
                }
            }
        }
        
        // Apply physics settings if available
        if (scenario.contains("physics")) {
            const auto& physics = scenario["physics"];
            
            if (physics.contains("solver_iterations")) {
                system.solverIter = physics["solver_iterations"];
            }
        }
        
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error parsing scenario JSON: " << e.what() << std::endl;
        return false;
    }
}