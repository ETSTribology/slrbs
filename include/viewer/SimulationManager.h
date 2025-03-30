#pragma once

#include <memory>

namespace slrbs {

class SimViewer;
class RigidBodySystem;
class RigidBodySystemState;
class RigidBodyRenderer;

/**
 * @class SimulationManager
 * @brief Manages the simulation state and parameters.
 */
class SimulationManager {
public:
    SimulationManager(SimViewer* viewer);
    ~SimulationManager();
    
    // Simulation control
    void initialize();
    void update(float deltaTime);
    void step();
    void reset();
    void save();
    
    // Simulation parameters
    void setTimeScale(float scale) { timeScale = scale; }
    float getTimeScale() const { return timeScale; }
    
    void setTimeStep(float step) { timeStep = step; }
    float getTimeStep() const { return timeStep; }
    
    void setSolverIterations(int iterations);
    int getSolverIterations() const;
    
    void setSubsteps(int steps) { substeps = steps; }
    int getSubsteps() const { return substeps; }
    
    void setAdaptiveTimesteps(bool adaptive) { adaptiveTimesteps = adaptive; }
    bool getAdaptiveTimesteps() const { return adaptiveTimesteps; }
    
    void setAlpha(float a) { alpha = a; }
    float getAlpha() const { return alpha; }
    
    void setGeometricStiffnessDamping(bool enable);
    bool getGeometricStiffnessDamping() const { return geometricStiffnessDamping; }
    
    void setEnableScreenshots(bool enable) { enableScreenshots = enable; }
    bool getEnableScreenshots() const { return enableScreenshots; }
    
    void setEnableLogging(bool enable) { enableLogging = enable; }
    bool getEnableLogging() const { return enableLogging; }
    
    // Simulation state
    void setPaused(bool isPaused) { paused = isPaused; }
    void pause() { paused = true; }
    void resume() { paused = false; }
    void togglePause() { paused = !paused; }
    bool isPaused() const { return paused; }
    bool getStepOnce() const { return stepOnce; }
    
    // Performance metrics
    float getDynamicsTime() const { return dynamicsTime; }
    float getKineticEnergy() const { return kineticEnergy; }
    float getConstraintError() const { return constraintError; }
    int getFrameCount() const { return frameCount; }

private:
    // Helper methods
    void updateAdaptiveTimesteps();
    void computeKineticEnergy();
    void computeConstraintError();
    void setPreStepGeometricStiffnessDamping(bool enable);
    
    // Member variables
    SimViewer* viewer;
    std::unique_ptr<RigidBodySystemState> resetState;
    
    // Simulation parameters
    float timeScale;
    float timeStep;
    int substeps;
    bool adaptiveTimesteps;
    float alpha;
    bool geometricStiffnessDamping;
    bool enableScreenshots;
    bool enableLogging;
    
    // Simulation state
    bool paused;
    bool stepOnce;
    
    // Performance metrics
    float dynamicsTime;
    float kineticEnergy;
    float constraintError;
    int frameCount;
};

} // namespace slrbs
