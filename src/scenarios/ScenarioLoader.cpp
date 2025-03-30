#include "scenarios/ScenarioLoader.h"
#include "collision/Geometry.h"
#include "joint/Joint.h"
#include "joint/Hinge.h"
#include "joint/Spherical.h"
#include "utils/MeshAssets.h"

#include <polyscope/polyscope.h>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <filesystem>

namespace slrbs {

std::filesystem::path ScenarioLoader::getScenariosPath() {
    auto resourcesPath = JsonLoader::getResourcesPath();
    auto scenariosPath = resourcesPath / "scenarios";
    
    if (!std::filesystem::exists(scenariosPath)) {
        // Try to create the directory if it doesn't exist
        std::filesystem::create_directories(scenariosPath);
    }
    
    return scenariosPath;
}

std::string ScenarioLoader::resolveFilePath(const std::string& filename) {
    std::filesystem::path filePath(filename);
    
    // If it's already an absolute path or explicitly relative, use it directly
    if (filePath.is_absolute() || filename.starts_with("./") || filename.starts_with("../")) {
        return filename;
    }
    
    // Otherwise, treat it as relative to scenarios directory
    auto resolvedPath = getScenariosPath() / filename;
    
    // Check if file exists
    if (!std::filesystem::exists(resolvedPath)) {
        // Try adding .json extension if not present
        if (resolvedPath.extension() != ".json") {
            auto withExtension = resolvedPath;
            withExtension.replace_extension(".json");
            if (std::filesystem::exists(withExtension)) {
                return withExtension.string();
            }
        }
        
        std::cerr << "Warning: File not found: " << resolvedPath.string() << std::endl;
    }
    
    return resolvedPath.string();
}

std::vector<std::string> ScenarioLoader::listAvailableScenarios() {
    std::vector<std::string> scenarios;
    
    try {
        auto scenariosPath = getScenariosPath();
        if (!std::filesystem::exists(scenariosPath)) {
            return scenarios;
        }
        
        // List all JSON files in scenarios directory
        for (const auto& entry : std::filesystem::directory_iterator(scenariosPath)) {
            if (entry.is_regular_file() && entry.path().extension() == ".json") {
                // Add the filename without extension
                scenarios.push_back(entry.path().stem().string());
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error listing scenarios: " << e.what() << std::endl;
    }
    
    return scenarios;
}

bool ScenarioLoader::loadScenarioFromFile(RigidBodySystem& system, const std::string& filename) {
    try {
        // Resolve the file path
        std::string resolvedPath = resolveFilePath(filename);
        
        // Check if file exists
        if (!std::filesystem::exists(resolvedPath)) {
            std::cerr << "File not found: " << resolvedPath << std::endl;
            return false;
        }

        // Clear existing system
        system.clear();
        polyscope::removeAllStructures();
        
        // Load and parse JSON
        nlohmann::json scenario = JsonLoader::loadFromFile(resolvedPath);
        
        return loadScenario(system, scenario);
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading scenario from JSON: " << e.what() << std::endl;
        return false;
    }
}

bool ScenarioLoader::saveScenarioToFile(const RigidBodySystem& system, const std::string& name, const std::string& filename) {
    try {
        // Determine if we should save to scenarios directory
        std::string resolvedPath = filename;
        if (!std::filesystem::path(filename).is_absolute() && 
            !filename.starts_with("./") && 
            !filename.starts_with("../")) {
            resolvedPath = (getScenariosPath() / filename).string();
        }
        
        nlohmann::json scenario = saveScenario(system, name);
        JsonLoader::saveToFile(scenario, resolvedPath);
        std::cout << "Successfully saved scenario to " << resolvedPath << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error saving scenario to JSON: " << e.what() << std::endl;
        return false;
    }
}

nlohmann::json ScenarioLoader::saveScenario(const RigidBodySystem& system, const std::string& name) {
    nlohmann::json scenario;
    scenario["name"] = name;
    scenario["description"] = "SLRBS Scenario";
    scenario["version"] = 1.0;
    
    // Save bodies
    nlohmann::json bodies = nlohmann::json::array();
    for (size_t i = 0; i < system.getBodies().size(); i++) {
        const RigidBody* body = system.getBodies()[i];
        nlohmann::json bodyJson;
        
        bodyJson["id"] = i;
        bodyJson["mass"] = body->mass;
        bodyJson["fixed"] = body->fixed;
        
        // Position
        bodyJson["position"] = {
            {"x", body->x.x()},
            {"y", body->x.y()},
            {"z", body->x.z()}
        };
        
        // Orientation (quaternion)
        bodyJson["orientation"] = {
            {"w", body->q.w()},
            {"x", body->q.x()},
            {"y", body->q.y()},
            {"z", body->q.z()}
        };
        
        // Linear velocity
        bodyJson["linear_velocity"] = {
            {"x", body->xdot.x()},
            {"y", body->xdot.y()},
            {"z", body->xdot.z()}
        };
        
        // Angular velocity
        bodyJson["angular_velocity"] = {
            {"x", body->omega.x()},
            {"y", body->omega.y()},
            {"z", body->omega.z()}
        };
        
        // Geometry type and parameters
        if (body->geometry) {
            if (auto sphere = dynamic_cast<const Sphere*>(body->geometry)) {
                bodyJson["geometry"] = {
                    {"type", "sphere"},
                    {"radius", sphere->radius}
                };
            }
            else if (auto box = dynamic_cast<const Box*>(body->geometry)) {
                bodyJson["geometry"] = {
                    {"type", "box"},
                    {"dimensions", {
                        {"x", box->dim.x()},
                        {"y", box->dim.y()},
                        {"z", box->dim.z()}
                    }}
                };
            }
            else if (auto cylinder = dynamic_cast<const Cylinder*>(body->geometry)) {
                bodyJson["geometry"] = {
                    {"type", "cylinder"},
                    {"height", cylinder->height},
                    {"radius", cylinder->radius}
                };
            }
            else if (auto plane = dynamic_cast<const Plane*>(body->geometry)) {
                bodyJson["geometry"] = {
                    {"type", "plane"},
                    {"point", {
                        {"x", plane->point.x()},
                        {"y", plane->point.y()},
                        {"z", plane->point.z()}
                    }},
                    {"normal", {
                        {"x", plane->normal.x()},
                        {"y", plane->normal.y()},
                        {"z", plane->normal.z()}
                    }}
                };
            }
        }
        
        // Visual properties
        if (body->mesh && body->mesh->parent) {
            nlohmann::json visualJson;
            
            // Color
            if (body->color.norm() > 0) {
                visualJson["color"] = {
                    {"r", body->color.x()},
                    {"g", body->color.y()},
                    {"b", body->color.z()}
                };
            } else {
                auto surfaceColor = body->mesh->getSurfaceColor();
                visualJson["color"] = {
                    {"r", surfaceColor[0]},
                    {"g", surfaceColor[1]},
                    {"b", surfaceColor[2]}
                };
            }
            
            // Material
            if (!body->materialName.empty()) {
                visualJson["material"] = body->materialName;
            }
            
            // Texture
            if (!body->texturePath.empty()) {
                // Store relative path if possible
                std::filesystem::path texPath(body->texturePath);
                std::filesystem::path texDir = JsonLoader::getTexturesPath();
                
                // Try to make the path relative to textures directory
                std::error_code ec;
                auto relPath = std::filesystem::relative(texPath, texDir, ec);
                if (!ec) {
                    visualJson["texture"] = relPath.string();
                } else {
                    visualJson["texture"] = body->texturePath;
                }
            }
            
            visualJson["transparency"] = body->mesh->getTransparency();
            visualJson["smooth_shade"] = body->mesh->getSmoothShade();
            visualJson["edge_width"] = body->mesh->getEdgeWidth();
            
            bodyJson["visual"] = visualJson;
        }
        
        bodies.push_back(bodyJson);
    }
    scenario["bodies"] = bodies;
    
    // Save joints
    nlohmann::json joints = nlohmann::json::array();
    const auto& systemJoints = system.getJoints();
    const auto& systemBodies = system.getBodies();
    
    for (size_t i = 0; i < systemJoints.size(); i++) {
        const Joint* joint = systemJoints[i];
        nlohmann::json jointJson;
        
        jointJson["id"] = i;
        
        // Find body indices
        auto body0Index = std::distance(systemBodies.begin(), 
            std::find(systemBodies.begin(), systemBodies.end(), joint->body0));
        auto body1Index = std::distance(systemBodies.begin(), 
            std::find(systemBodies.begin(), systemBodies.end(), joint->body1));
        
        jointJson["body0_id"] = body0Index;
        jointJson["body1_id"] = body1Index;
        
        // Determine joint type and save specific parameters
        if (auto hinge = dynamic_cast<const Hinge*>(joint)) {
            jointJson["type"] = "hinge";
            
            // Local offsets and orientations
            jointJson["body0_offset"] = {
                {"x", hinge->r0.x()},
                {"y", hinge->r0.y()},
                {"z", hinge->r0.z()}
            };
            
            jointJson["body1_offset"] = {
                {"x", hinge->r1.x()},
                {"y", hinge->r1.y()},
                {"z", hinge->r1.z()}
            };
            
            jointJson["body0_orientation"] = {
                {"w", hinge->q0.w()},
                {"x", hinge->q0.x()},
                {"y", hinge->q0.y()},
                {"z", hinge->q0.z()}
            };
            
            jointJson["body1_orientation"] = {
                {"w", hinge->q1.w()},
                {"x", hinge->q1.x()},
                {"y", hinge->q1.y()},
                {"z", hinge->q1.z()}
            };
        }
        else if (auto spherical = dynamic_cast<const Spherical*>(joint)) {
            jointJson["type"] = "spherical";
            
            // Local offsets
            jointJson["body0_offset"] = {
                {"x", spherical->r0.x()},
                {"y", spherical->r0.y()},
                {"z", spherical->r0.z()}
            };
            
            jointJson["body1_offset"] = {
                {"x", spherical->r1.x()},
                {"y", spherical->r1.y()},
                {"z", spherical->r1.z()}
            };
        }
        
        joints.push_back(jointJson);
    }
    scenario["joints"] = joints;
    
    // Physics settings
    scenario["physics"] = {
        {"gravity", {
            {"x", 0.0},
            {"y", -9.81},
            {"z", 0.0}
        }},
        {"time_step", 0.01},
        {"solver_iterations", system.solverIter}
    };
    
    return scenario;
}

bool ScenarioLoader::loadScenario(RigidBodySystem& system, const nlohmann::json& scenario) {
    try {
        // Clear existing system
        system.clear();
        polyscope::removeAllStructures();
        
        std::string name = scenario["name"].get<std::string>();
        std::cout << "Loading scenario: " << name << std::endl;
        
        // Load bodies
        std::vector<RigidBody*> bodies;
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
                else {
                    std::cerr << "Unknown geometry type: " << type << std::endl;
                    continue;
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
                
                // Set color
                if (visual.contains("color")) {
                    // Handle color format variations
                    Eigen::Vector3f color;
                    
                    if (visual["color"].is_object() && 
                        visual["color"].contains("r") && 
                        visual["color"].contains("g") && 
                        visual["color"].contains("b")) {
                        // RGB object format
                        color = Eigen::Vector3f(
                            visual["color"]["r"],
                            visual["color"]["g"],
                            visual["color"]["b"]
                        );
                    } else if (visual["color"].is_array() && visual["color"].size() >= 3) {
                        // Array format [r,g,b]
                        color = Eigen::Vector3f(
                            visual["color"][0],
                            visual["color"][1],
                            visual["color"][2]
                        );
                    } else {
                        std::cerr << "Invalid color format for body" << std::endl;
                        color = Eigen::Vector3f(0.5f, 0.5f, 0.5f); // Default grey
                    }
                    
                    body->color = color;
                    body->mesh->setSurfaceColor({color.x(), color.y(), color.z()});
                }
                
                // Set material
                if (visual.contains("material") && visual["material"].is_string()) {
                    body->materialName = visual["material"].get<std::string>();
                }
                
                // Set texture
                if (visual.contains("texture") && visual["texture"].is_string()) {
                    std::string texPath = visual["texture"].get<std::string>();
                    // Resolve texture path
                    body->texturePath = JsonLoader::resolveTexturePath(texPath);
                }
                
                // Set transparency
                if (visual.contains("transparency")) {
                    body->mesh->setTransparency(visual["transparency"]);
                }
                
                // Set smooth shading
                if (visual.contains("smooth_shade")) {
                    body->mesh->setSmoothShade(visual["smooth_shade"]);
                }
                
                // Set edge width
                if (visual.contains("edge_width")) {
                    body->mesh->setEdgeWidth(visual["edge_width"]);
                }
            }
            
            // Add body to the system
            system.addBody(body);
            bodies.push_back(body);
        }
        
        // Load joints
        if (scenario.contains("joints")) {
            for (const auto& jointJson : scenario["joints"]) {
                int body0Id = jointJson["body0_id"];
                int body1Id = jointJson["body1_id"];
                
                if (body0Id < 0 || body0Id >= static_cast<int>(bodies.size()) || 
                    body1Id < 0 || body1Id >= static_cast<int>(bodies.size())) {
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
                    
                    joint = new Hinge(bodies[body0Id], bodies[body1Id], r0, q0, r1, q1);
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
                    
                    joint = new Spherical(bodies[body0Id], bodies[body1Id], r0, r1);
                }
                
                if (joint) {
                    system.addJoint(joint);
                }
            }
        }
        
        // Apply physics settings if available
        if (scenario.contains("physics")) {
            const auto& physics = scenario["physics"];
            
            if (physics.contains("solver_iterations") && physics["solver_iterations"].is_number()) {
                system.solverIter = physics["solver_iterations"];
            }
        }
        
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading scenario from JSON: " << e.what() << std::endl;
        return false;
    }
}

} // namespace slrbs
