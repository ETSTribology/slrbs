#pragma once

#include "rigidbody/RigidBodySystem.h"
#include <string>
#include <vector>

/**
 * @file ScenarioLoader.h
 *
 * @brief Utility for loading scenarios from JSON files.
 */

class ScenarioLoader {
public:
    // Load a scenario from a JSON file
    static bool loadFromFile(RigidBodySystem& system, const std::string& filename);
    
    // List all available JSON scenario files in the scenarios directory
    static std::vector<std::string> listAvailableScenarios();
    
private:
    // Get path to scenarios directory
    static std::string getScenariosPath();
    
    // Parse JSON and apply to rigid body system
    static bool parseScenario(RigidBodySystem& system, const std::string& jsonContent);
};
