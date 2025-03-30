#include "solvers/SolverBoxBPP.h"
#include "solvers/SolverUtils.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include "contact/Contact.h"
#include "joint/Joint.h"

#include <Eigen/Dense>
#include <iostream>
#include <limits>
#include <algorithm>

// Initialize static members
const std::string SolverBoxBPP::m_name = "Box-constrained Block Principal Pivoting";

SolverBoxBPP::SolverBoxBPP(RigidBodySystem* rigidBodySystem)
    : Solver(rigidBodySystem),
      m_useSmoothFriction(true),
      m_frictionVelocityThreshold(0.1f)
{
    // Set the solver name and default parameters
}

const std::string& SolverBoxBPP::getName() const
{
    return m_name;
}

void SolverBoxBPP::solve(float h)
{
    // Start performance timing if enabled
    startTiming();
    
    // Get the rigid body system data
    auto& joints = m_rigidBodySystem->getJoints();
    auto& contacts = m_rigidBodySystem->getContacts();
    
    // Count dimensions and assign indices
    unsigned int dim = 0;
    
    // Assign indices to joints
    for (Joint* joint : joints) {
        joint->idx = dim;
        dim += joint->dim;
    }
    
    // Assign indices to contacts
    for (Contact* contact : contacts) {
        contact->idx = dim;
        dim += contact->dim;
    }
    
    // Skip if no constraints
    if (dim == 0) {
        endTiming();
        return;
    }
    
    // Create system matrices and vectors
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(dim, dim);
    Eigen::VectorXf b = Eigen::VectorXf::Zero(dim);
    Eigen::VectorXf x = Eigen::VectorXf::Zero(dim);
    
    // Initialize box bounds
    Eigen::VectorXf lower, upper;
    SolverUtils::initializeBounds(dim, lower, upper);
    
    // Update bounds for contacts
    SolverUtils::updateContactBounds(contacts, lower, upper, m_useSmoothFriction, m_frictionVelocityThreshold);
    
    // Initialize index set (all variables start as 'free')
    Eigen::VectorXi idx = Eigen::VectorXi::Constant(dim, kFree);
    
    // Ignore variables where lower and upper are both zero
    #ifdef USE_OPENMP
    #pragma omp parallel for if(m_parallelEnabled)
    #endif
    for (int i = 0; i < dim; ++i) {
        static const float tol = 1e-5f;
        if (std::abs(upper(i)) < tol && std::abs(lower(i)) < tol) {
            idx(i) = kIgnore;
        }
    }
    
    // Build system matrix and RHS
    SolverUtils::buildSchurComplementMatrix(joints, contacts, h, A);
    
    // Get gamma value from the rigid body system (default to 0.3 if not set)
    float gamma = 0.3f;
    if (m_rigidBodySystem->hasProperty("gamma")) {
        gamma = m_rigidBodySystem->getFloatProperty("gamma");
    }
    
    SolverUtils::buildSchurComplementRHS(joints, contacts, h, b, gamma);
    
    // Perform initial solve to update friction bounds
    solvePrincipalSubproblem(A, b, idx, lower, upper, x);
    SolverUtils::updateJointsAndContacts(x, joints, contacts);
    SolverUtils::updateContactBounds(contacts, lower, upper, m_useSmoothFriction, m_frictionVelocityThreshold);
    
    // Reset index set for main iteration
    idx.setConstant(kFree);
    
    // Main block pivoting loop
    for (unsigned int iter = 0; iter < m_maxIter; ++iter) {
        // Solve the principal subproblem
        solvePrincipalSubproblem(A, b, idx, lower, upper, x);
        
        // Compute residual
        const Eigen::VectorXf residual = A * x - b;
        
        // Pivot variables between sets
        const int numPivots = pivotVariables(idx, x, lower, upper, residual);
        
        // Break if converged
        if (numPivots == 0) {
            break;
        }
    }
    
    // Final update of joints and contacts
    SolverUtils::updateJointsAndContacts(x, joints, contacts);
    SolverUtils::updateContactBounds(contacts, lower, upper, m_useSmoothFriction, m_frictionVelocityThreshold);
    
    // End performance timing
    endTiming();
}

unsigned int SolverBoxBPP::pivotVariables(Eigen::VectorXi& idx, 
                                        Eigen::VectorXf& x, 
                                        const Eigen::VectorXf& lower, 
                                        const Eigen::VectorXf& upper, 
                                        const Eigen::VectorXf& residual)
{
    static const float tol = 1e-5f;
    unsigned int numPivots = 0;
    const unsigned int n = idx.size();
    
    #ifdef USE_OPENMP
    int threadCount = getEffectiveThreadCount();
    std::vector<unsigned int> threadPivots(threadCount, 0);
    
    #pragma omp parallel for if(m_parallelEnabled) num_threads(threadCount)
    #endif
    for (int j = 0; j < n; ++j) {
        // Skip ignored variables
        if (idx[j] == kIgnore) continue;
        
        // Default: assume variable stays in current set
        int newIdx = idx[j];
        
        // Check for constraint violations
        if (idx[j] == kFree && x[j] <= lower[j]) {
            // Free variable hitting lower bound
            newIdx = kLower;
        } else if (idx[j] == kFree && x[j] >= upper[j]) {
            // Free variable hitting upper bound
            newIdx = kUpper;
        } else if (idx[j] == kLower && residual[j] > -tol) {
            // Lower bound variable with positive residual - should be free
            newIdx = kFree;
        } else if (idx[j] == kUpper && residual[j] < tol) {
            // Upper bound variable with negative residual - should be free
            newIdx = kFree;
        }
        
        // If status changed, count as pivot
        if (newIdx != idx[j]) {
            #ifdef USE_OPENMP
            if (m_parallelEnabled) {
                int threadId = omp_get_thread_num();
                threadPivots[threadId]++;
            } else {
                numPivots++;
            }
            #else
            numPivots++;
            #endif
            
            idx[j] = newIdx;
        }
    }
    
    #ifdef USE_OPENMP
    if (m_parallelEnabled) {
        for (unsigned int i = 0; i < threadPivots.size(); i++) {
            numPivots += threadPivots[i];
        }
    }
    #endif
    
    return numPivots;
}

unsigned int SolverBoxBPP::findTightIndices(const Eigen::VectorXi& idx, std::vector<int>& tightIdx) const
{
    const unsigned int n = idx.size();
    unsigned int numTight = 0;
    tightIdx.clear();
    
    for (unsigned int i = 0; i < n; ++i) {
        if (idx[i] == kLower || idx[i] == kUpper) {
            tightIdx.push_back(i);
            numTight++;
        }
    }
    
    return numTight;
}

unsigned int SolverBoxBPP::findFreeIndices(const Eigen::VectorXi& idx, std::vector<int>& freeIdx) const
{
    const unsigned int n = idx.size();
    unsigned int numFree = 0;
    freeIdx.clear();
    
    for (unsigned int i = 0; i < n; ++i) {
        if (idx[i] == kFree) {
            freeIdx.push_back(i);
            numFree++;
        }
    }
    
    return numFree;
}

void SolverBoxBPP::solvePrincipalSubproblem(const Eigen::MatrixXf& A, 
                                          const Eigen::VectorXf& b, 
                                          const Eigen::VectorXi& idx, 
                                          const Eigen::VectorXf& lower, 
                                          const Eigen::VectorXf& upper, 
                                          Eigen::VectorXf& x)
{
    std::vector<int> freeIdx, tightIdx;
    const unsigned int numTight = findTightIndices(idx, tightIdx);
    const unsigned int numFree = findFreeIndices(idx, freeIdx);
    
    if (numFree > 0) {
        // Build matrix and RHS for free variables
        Eigen::MatrixXf Aff(numFree, numFree);
        Eigen::VectorXf bf(numFree);
        
        // Extract submatrix for free variables
        #ifdef USE_OPENMP
        #pragma omp parallel for if(m_parallelEnabled)
        #endif
        for (unsigned int j = 0; j < numFree; ++j) {
            for (unsigned int i = 0; i < numFree; ++i) {
                Aff(i, j) = A(freeIdx[i], freeIdx[j]);
            }
        }
        
        // Extract RHS for free variables
        #ifdef USE_OPENMP
        #pragma omp parallel for if(m_parallelEnabled)
        #endif
        for (unsigned int i = 0; i < numFree; ++i) {
            bf(i) = b(freeIdx[i]);
        }
        
        // Update RHS with tight variable contributions
        for (unsigned int j = 0; j < numTight; ++j) {
            for (unsigned int i = 0; i < numFree; ++i) {
                if (idx[tightIdx[j]] == kLower) {
                    bf(i) -= A(freeIdx[i], tightIdx[j]) * lower(tightIdx[j]);
                } else if (idx[tightIdx[j]] == kUpper) {
                    bf(i) -= A(freeIdx[i], tightIdx[j]) * upper(tightIdx[j]);
                }
            }
        }
        
        // Solve free variable system
        Eigen::LDLT<Eigen::MatrixXf> ldlt(Aff);
        const Eigen::VectorXf xf = ldlt.solve(bf);
        
        // Update free variables
        #ifdef USE_OPENMP
        #pragma omp parallel for if(m_parallelEnabled)
        #endif
        for (unsigned int i = 0; i < numFree; ++i) {
            x(freeIdx[i]) = xf(i);
        }
        
        // Update tight variables
        #ifdef USE_OPENMP
        #pragma omp parallel for if(m_parallelEnabled)
        #endif
        for (unsigned int i = 0; i < numTight; ++i) {
            if (idx[tightIdx[i]] == kLower) {
                x(tightIdx[i]) = lower(tightIdx[i]);
            } else if (idx[tightIdx[i]] == kUpper) {
                x(tightIdx[i]) = upper(tightIdx[i]);
            }
        }
    }
}