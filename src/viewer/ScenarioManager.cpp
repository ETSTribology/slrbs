#include "viewer/ScenarioManager.h"
#include "viewer/SimViewer.h"
#include "utils/JsonLoader.h"
#include "rigidbody/RigidBodySystem.h"
#include "scenarios/Scenarios.h"

#include <iostream>
#include <filesystem>

namespace slrbs {

ScenarioManager::ScenarioManager(SimViewer* viewer) : viewer(viewer) {
}

ScenarioManager::~ScenarioManager() {
}

void ScenarioManager::initialize() {
    // Register built-in scenarios
    registerDefaultScenarios();
    
    // Try to load additional scenarios from JSON
    loadScenariosFromJson();
    
    std::cout << "ScenarioManager: Initialized with " << scenarios.size() << " scenarios" << std::endl;
}

void ScenarioManager::registerDefaultScenarios() {
    // Register the default scenarios provided by the Scenarios class
    scenarios.push_back({
        "Sphere on Box", 
        "A simple sphere resting on a box", 
        [](RigidBodySystem& system) { Scenarios::createSphereOnBox(system); }
    });
    
    scenarios.push_back({
        "Stack", 
        "A stack of boxes", 
        [](RigidBodySystem& system) { Scenarios::createStack(system); }
    });
    
    scenarios.push_back({
        "Marble Box", 
        "Box filled with marbles", 
        [](RigidBodySystem& system) { Scenarios::createMarbleBox(system); }
    });
    
    scenarios.push_back({
        "Swinging Box", 
        "Box swinging on a hinge joint", 
        [](RigidBodySystem& system) { Scenarios::createSwingingBoxes(system); }
    });
    
    scenarios.push_back({
        "Cylinder on Plane", 
        "A cylinder rolling on a plane", 
        [](RigidBodySystem& system) { Scenarios::createCylinderOnPlane(system); }
    });
    
    scenarios.push_back({
        "Car Scene", 
        "A car with suspension", 
        [](RigidBodySystem& system) { Scenarios::createCarScene(system); }
    });
    
    scenarios.push_back({
        "Rope Bridge", 
        "A bridge made of rope and planks", 
        [](RigidBodySystem& system) { Scenarios::createRopeBridgeScene(system); }
    });
}

void ScenarioManager::loadScenariosFromJson() {
    try {
        // Check if scenarios directory exists
        std::filesystem::path scenariosPath = JsonLoader::getResourcesPath() / "scenarios";
        if (!std::filesystem::exists(scenariosPath)) {
            std::cout << "ScenarioManager: No scenarios directory found at " << scenariosPath.string() << std::endl;
            return;
        }
        
        // Look for scenario index file first
        std::filesystem::path indexPath = scenariosPath / "index.json";
        if (std::filesystem::exists(indexPath)) {
            nlohmann::json index = JsonLoader::loadFromFile(indexPath.string());
            
            if (index.contains("scenarios") && index["scenarios"].is_array()) {
                for (const auto& entry : index["scenarios"]) {
                    if (entry.contains("name") && entry.contains("file")) {
                        std::string name = entry["name"];
                        std::string description = entry.value("description", "");
                        std::string filename = entry["file"];
                        
                        std::filesystem::path scenarioPath = scenariosPath / filename;
                        if (std::filesystem::exists(scenarioPath)) {
                            // Add a scenario that will load the specified file
                            scenarios.push_back({
                                name,
                                description,
                                [this, scenarioPath](RigidBodySystem& system) { 
                                    loadScenarioFromFile(system, scenarioPath.string());
                                }
                            });
                            
                            std::cout << "ScenarioManager: Added scenario '" << name << "' from " << scenarioPath.string() << std::endl;
                        }
                    }
                }
            }
        } else {
            // No index file, scan for individual scenario files
            for (const auto& entry : std::filesystem::directory_iterator(scenariosPath)) {
                if (entry.path().extension() == ".json") {
                    try {
                        nlohmann::json scenarioData = JsonLoader::loadFromFile(entry.path().string());
                        if (scenarioData.contains("name")) {
                            std::string name = scenarioData["name"];
                            std::string description = scenarioData.value("description", "");
                            
                            scenarios.push_back({
                                name,
                                description,
                                [this, path = entry.path().string()](RigidBodySystem& system) { 
                                    loadScenarioFromFile(system, path);
                                }
                            });
                            
                            std::cout << "ScenarioManager: Added scenario '" << name << "' from " << entry.path().string() << std::endl;
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "ScenarioManager: Failed to load scenario from " << entry.path().string() << ": " << e.what() << std::endl;
                    }
                }
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "ScenarioManager: Failed to load scenarios: " << e.what() << std::endl;
    }
}

void ScenarioManager::loadScenarioFromFile(RigidBodySystem& system, const std::string& filename) {
    try {
        nlohmann::json scenarioData = JsonLoader::loadFromFile(filename);
        
        // Clear the current system
        system.clear();
        
        // Check if we should apply gravity
        if (scenarioData.contains("gravity") && scenarioData["gravity"].is_array() && scenarioData["gravity"].size() == 3) {
            auto& gravity = scenarioData["gravity"];
            Eigen::Vector3f gravityVec(
                gravity[0].get<float>(),
                gravity[1].get<float>(),
                gravity[2].get<float>()
            );
            system.setGravity(gravityVec);
        }
        
        // Create a map to store body pointers by ID
        std::map<int, RigidBody*> bodyMap;
        
        // Load bodies
        if (scenarioData.contains("bodies") && scenarioData["bodies"].is_array()) {
            int bodyIdx = 0;
            for (const auto& bodyData : scenarioData["bodies"]) {
                // Extract basic properties
                std::string name = bodyData.value("name", "unnamed");
                bool fixed = bodyData.value("fixed", false);
                float mass = bodyData.value("mass", 1.0f);
                
                // Create the body
                RigidBody* body = new RigidBody();
                body->name = name;
                body->fixed = fixed;
                body->mass = mass;
                
                // Set position
                if (bodyData.contains("position") && bodyData["position"].size() == 3) {
                    auto& pos = bodyData["position"];
                    body->x = Eigen::Vector3f(
                        pos[0].get<float>(),
                        pos[1].get<float>(),
                        pos[2].get<float>()
                    );
                }
                
                // Set velocity
                if (bodyData.contains("velocity") && bodyData["velocity"].size() == 3) {
                    auto& vel = bodyData["velocity"];
                    body->xdot = Eigen::Vector3f(
                        vel[0].get<float>(),
                        vel[1].get<float>(),
                        vel[2].get<float>()
                    );
                }
                
                // Set rotation (as quaternion)
                if (bodyData.contains("rotation") && bodyData["rotation"].size() == 4) {
                    auto& rot = bodyData["rotation"];
                    body->q = Eigen::Quaternionf(
                        rot[0].get<float>(),  // w
                        rot[1].get<float>(),  // x
                        rot[2].get<float>(),  // y
                        rot[3].get<float>()   // z
                    );
                }
                
                // Set angular velocity
                if (bodyData.contains("angularVelocity") && bodyData["angularVelocity"].size() == 3) {
                    auto& angVel = bodyData["angularVelocity"];
                    body->omega = Eigen::Vector3f(
                        angVel[0].get<float>(),
                        angVel[1].get<float>(),
                        angVel[2].get<float>()
                    );
                }
                
                // Set geometry
                if (bodyData.contains("geometry")) {
                    auto& geo = bodyData["geometry"];
                    std::string type = geo.value("type", "box");
                    
                    if (type == "box") {
                        if (geo.contains("dimensions") && geo["dimensions"].size() == 3) {
                            auto& dims = geo["dimensions"];
                            body->initializeBox(
                                dims[0].get<float>(),
                                dims[1].get<float>(),
                                dims[2].get<float>()
                            );
                        }
                    } else if (type == "sphere") {
                        if (geo.contains("radius")) {
                            float radius = geo["radius"].get<float>();
                            body->initializeSphere(radius);
                        }
                    } else if (type == "cylinder") {
                        if (geo.contains("radius") && geo.contains("height")) {
                            float radius = geo["radius"].get<float>();
                            float height = geo["height"].get<float>();
                            body->initializeCylinder(radius, height);
                        }
                    } else if (type == "mesh") {
                        if (geo.contains("file")) {
                            std::string meshFile = geo["file"].get<std::string>();
                            // TODO: Implement mesh loading from file
                        }
                    }
                }
                
                // Set color
                if (bodyData.contains("color") && bodyData["color"].size() == 3) {
                    auto& color = bodyData["color"];
                    body->color = Eigen::Vector3f(
                        color[0].get<float>(),
                        color[1].get<float>(),
                        color[2].get<float>()
                    );
                }
                
                // Set material name
                if (bodyData.contains("materialName")) {
                    body->materialName = bodyData["materialName"].get<std::string>();
                }
                
                // Set texture path
                if (bodyData.contains("texturePath")) {
                    body->texturePath = bodyData["texturePath"].get<std::string>();
                }
                
                // Add to system
                system.addBody(body);
                
                // Store in map
                int id = bodyData.value("id", bodyIdx);
                bodyMap[id] = body;
                bodyIdx++;
            }
        }
        
        // Create joints
        if (scenarioData.contains("joints") && scenarioData["joints"].is_array()) {
            for (const auto& jointData : scenarioData["joints"]) {
                if (!jointData.contains("type") || 
                    !jointData.contains("body0") || 
                    !jointData.contains("body1")) {
                    continue;
                }
                
                std::string type = jointData["type"];
                int body0Id = jointData["body0"].get<int>();
                int body1Id = jointData["body1"].get<int>();
                
                // Make sure body IDs are valid
                if (bodyMap.find(body0Id) == bodyMap.end() ||
                    bodyMap.find(body1Id) == bodyMap.end()) {
                    continue;
                }
                
                RigidBody* body0 = bodyMap[body0Id];
                RigidBody* body1 = bodyMap[body1Id];
                
                // Create appropriate joint type
                Joint* joint = nullptr;
                
                if (type == "hinge") {
                    // TODO: Create hinge joint
                } else if (type == "spherical") {
                    // TODO: Create spherical joint
                } else if (type == "distance") {
                    // TODO: Create distance joint
                } else if (type == "prismatic") {
                    // TODO: Create prismatic joint
                }
                
                // Add joint to system if created
                if (joint) {
                    system.addJoint(joint);
                }
            }
        }
        
        std::cout << "ScenarioManager: Loaded scenario from " << filename << std::endl;
        std::cout << "  - Bodies: " << system.getBodies().size() << std::endl;
        std::cout << "  - Joints: " << system.getJoints().size() << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "ScenarioManager: Failed to load scenario from file: " << e.what() << std::endl;
    }
}

void ScenarioManager::loadScenario(const std::string& name) {
    for (size_t i = 0; i < scenarios.size(); i++) {
        if (scenarios[i].name == name) {
            loadScenario(i);
            return;
        }
    }
    std::cerr << "ScenarioManager: Scenario '" << name << "' not found" << std::endl;
}

void ScenarioManager::loadScenario(int index) {
    if (index >= 0 && index < scenarios.size()) {
        std::cout << "ScenarioManager: Loading scenario '" << scenarios[index].name << "'" << std::endl;
        
        // Clear existing simulation
        viewer->getRigidBodySystem().clear();
        
        // Create the scenario
        scenarios[index].createFunction(viewer->getRigidBodySystem());
        
        // Apply visual properties to bodies
        for (auto* body : viewer->getRigidBodySystem().getBodies()) {
            // Apply color if defined
            if (body->color.norm() > 0) {
                viewer->getRenderer().setBodyColor(*body, 
                    {body->color.x(), body->color.y(), body->color.z()});
            }
            
            // Apply material if defined
            if (!body->materialName.empty()) {
                viewer->getRenderer().assignMaterial(*body, body->materialName);
            }
            
            // Apply texture if defined
            if (!body->texturePath.empty()) {
                viewer->getRenderer().setBodyTexture(*body, body->texturePath);
            }
        }
        
        // Save initial state for reset
        viewer->getSimulation().save();
        
        // Update visuals
        viewer->getRenderer().updateAll();
    } else {
        std::cerr << "ScenarioManager: Invalid scenario index: " << index << std::endl;
    }
}

} // namespace slrbs
