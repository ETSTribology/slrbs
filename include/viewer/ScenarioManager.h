#pragma once

#include <string>
#include <vector>
#include <functional>

namespace slrbs {

class SimViewer;
class RigidBodySystem;

/**
 * @class ScenarioManager
 * @brief Manages different simulation scenarios.
 */
class ScenarioManager {
public:
    struct ScenarioInfo {
        std::string name;
        std::string description;
        std::function<void(RigidBodySystem&)> createFunction;
    };
    
    ScenarioManager(SimViewer* viewer);
    ~ScenarioManager();
    
    // Scenario management
    void initialize();
    void loadScenario(const std::string& name);
    void loadScenario(int index);
    
    // Scenario access
    const std::vector<ScenarioInfo>& getScenarios() const { return scenarios; }
    
private:
    // Register default scenarios
    void registerDefaultScenarios();
    
    // Member variables
    SimViewer* viewer;
    std::vector<ScenarioInfo> scenarios;
};

} // namespace slrbs
