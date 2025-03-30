#pragma once

#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"
#include "utils/JsonLoader.h"
#include <nlohmann/json.hpp>
#include <string>
#include <filesystem>
#include <vector>

namespace slrbs {

/**
 * @brief Utility class for loading and saving scenarios in JSON format
 */
class ScenarioLoader {
public:
    /**
     * @brief Get the path to the scenarios directory
     * @return Path to scenarios directory
     */
    static std::filesystem::path getScenariosPath();

    /**
     * @brief Load a scenario from a JSON file
     * @param system The rigid body system to load into
     * @param filename Path to the JSON file (absolute or relative to scenarios dir)
     * @return true if loading was successful, false otherwise
     */
    static bool loadScenarioFromFile(RigidBodySystem& system, const std::string& filename);

    /**
     * @brief Save the current scenario to a JSON file
     * @param system The rigid body system to save
     * @param name Name of the scenario
     * @param filename Path to save the JSON file
     * @return true if saving was successful, false otherwise
     */
    static bool saveScenarioToFile(const RigidBodySystem& system, const std::string& name, const std::string& filename);

    /**
     * @brief Get a JSON representation of the current scenario
     * @param system The rigid body system to serialize
     * @param name Name of the scenario
     * @return JSON object representing the scenario
     */
    static nlohmann::json saveScenario(const RigidBodySystem& system, const std::string& name);

    /**
     * @brief Load a scenario from a JSON object
     * @param system The rigid body system to load into
     * @param scenario JSON object containing the scenario data
     * @return true if loading was successful, false otherwise
     */
    static bool loadScenario(RigidBodySystem& system, const nlohmann::json& scenario);

    /**
     * @brief Get a list of all available scenario files
     * @return Vector of scenario names (without extension)
     */
    static std::vector<std::string> listAvailableScenarios();

private:
    /**
     * @brief Resolve a filename to an absolute path
     * @param filename Filename (absolute or relative to scenarios dir)
     * @return Absolute path to the file
     */
    static std::string resolveFilePath(const std::string& filename);
};

} // namespace slrbs
