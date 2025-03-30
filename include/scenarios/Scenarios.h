#pragma once

#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"
#include <string>
#include <memory>
#include <map>
#include <functional>

class RigidBodySystem;

/**
 * @brief Scenario interface - uses Strategy pattern for different scenario types
 */
class ScenarioStrategy {
public:
    virtual ~ScenarioStrategy() = default;
    
    /**
     * @brief Create the scenario in the given rigid body system
     * @param rigidBodySystem System to populate with scenario objects
     */
    virtual void create(RigidBodySystem& rigidBodySystem) = 0;
    
    /**
     * @brief Get the name of this scenario
     * @return Scenario name
     */
    virtual std::string getName() const = 0;
    
    /**
     * @brief Get the description of this scenario
     * @return Scenario description
     */
    virtual std::string getDescription() const = 0;
};

/**
 * @brief Factory for creating or loading scenarios
 */
class ScenarioFactory {
public:
    /**
     * @brief Get the singleton instance of the factory
     * @return Reference to the factory instance
     */
    static ScenarioFactory& getInstance();
    
    /**
     * @brief Register a scenario strategy with the factory
     * @param name Unique name for the scenario
     * @param strategy Shared pointer to the scenario strategy
     */
    void registerScenario(const std::string& name, std::shared_ptr<ScenarioStrategy> strategy);
    
    /**
     * @brief Create a scenario by name
     * @param name Name of the registered scenario
     * @param rigidBodySystem System to populate with the scenario
     * @return True if scenario was found and created, false otherwise
     */
    bool createScenario(const std::string& name, RigidBodySystem& rigidBodySystem);
    
    /**
     * @brief Get a list of all available scenario names
     * @return Vector of scenario names
     */
    std::vector<std::string> getScenarioNames() const;
    
    /**
     * @brief Load a scenario from a JSON file
     * @param filename Path to the JSON file
     * @param rigidBodySystem System to populate with the scenario
     * @return True if the file was loaded successfully, false otherwise
     */
    bool loadFromFile(const std::string& filename, RigidBodySystem& rigidBodySystem);
    
    /**
     * @brief Save the current state as a scenario to a JSON file
     * @param name Name for the scenario
     * @param rigidBodySystem System to save
     * @param filename Path where to save the JSON file
     * @return True if the file was saved successfully, false otherwise
     */
    bool saveToFile(const std::string& name, const RigidBodySystem& rigidBodySystem, const std::string& filename);
    
private:
    ScenarioFactory();
    std::map<std::string, std::shared_ptr<ScenarioStrategy>> m_scenarios;
};

/**
 * @brief Convenience class for accessing predefined scenarios
 */
class Scenarios {
public:
    /**
     * @brief Initialize and register all built-in scenarios
     */
    static void registerBuiltInScenarios();
    
    /**
     * @brief Create marble box scenario
     * @param rigidBodySystem System to populate with the scenario
     */
    static void createMarbleBox(RigidBodySystem& rigidBodySystem);
    
    /**
     * @brief Create sphere on box scenario
     * @param rigidBodySystem System to populate with the scenario
     */
    static void createSphereOnBox(RigidBodySystem& rigidBodySystem);
    
    /**
     * @brief Create swinging boxes scenario
     * @param rigidBodySystem System to populate with the scenario
     */
    static void createSwingingBoxes(RigidBodySystem& rigidBodySystem);
    
    /**
     * @brief Create cylinder on plane scenario
     * @param rigidBodySystem System to populate with the scenario
     */
    static void createCylinderOnPlane(RigidBodySystem& rigidBodySystem);
    
    /**
     * @brief Create car scene scenario
     * @param rigidBodySystem System to populate with the scenario
     */
    static void createCarScene(RigidBodySystem& rigidBodySystem);
    
    /**
     * @brief Create rope bridge scenario
     * @param rigidBodySystem System to populate with the scenario
     */
    static void createRopeBridgeScene(RigidBodySystem& rigidBodySystem);
};
