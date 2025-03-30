#pragma once
#include "solvers/Solver.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <unordered_map>

namespace slrbs {
    class SimDataLogger;
}

class RigidBodySystem;
class Contact;

/**
 * SolverProximal - Implements a Proximal Gradient Solver for rigid body contact simulation
 * 
 * Uses a binary file format for efficient data export and analysis
 */
class SolverProximal : public Solver {
public:
    SolverProximal(RigidBodySystem* _rigidBodySystem);
    virtual ~SolverProximal();
    
    virtual void solve(float h) override;
    
    // Configure debug data export
    void enableDataExport(bool enable);
    void setExportPath(const std::string& basePath);
    
    // Log types that can be enabled/disabled individually
    enum class LogType {
        LCP_ERROR,
        RESIDUAL,
        MATRIX_R,
        ACTIVE_CONSTRAINTS,
        PERFORMANCE,
        MATRICES     // Includes A, J, etc.
    };
    
    // Enable/disable specific log types
    void setLogEnabled(LogType type, bool enabled);
    
protected:
    // Data export configuration
    bool m_exportEnabled = false;
    std::string m_exportBasePath = "./";
    std::unique_ptr<slrbs::SimDataLogger> m_dataLogger;
    
    // Flags for specific log types
    std::unordered_map<LogType, bool> m_logEnabled = {
        {LogType::LCP_ERROR, true},
        {LogType::RESIDUAL, true},
        {LogType::MATRIX_R, false},
        {LogType::ACTIVE_CONSTRAINTS, true},
        {LogType::PERFORMANCE, true},
        {LogType::MATRICES, false}
    };
    
    // Utility methods for writing data
    void initializeDataLogger(const std::string& exampleName);
    void exportMatrices(const std::vector<Eigen::Matrix3f>& A, const std::vector<Contact*>& contacts);
    void exportActiveConstraints(const std::vector<Eigen::Vector3f>& lambdaCandidate, float mu);
    void exportResidual(float residualNorm);
    void exportLCPError(float lcpError);
    void exportRValues(const std::vector<Eigen::MatrixXf>& R);
    void exportPerformanceData(double solveTimeMs, int iterations);
};

