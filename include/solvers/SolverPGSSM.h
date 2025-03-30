#pragma once

#include "solvers/Solver.h"
#include <vector>
#include <Eigen/Dense>

// Forward declaration of Joint and Contact classes
class Joint;
class Contact;

class SolverPGSSM : public Solver
{
public:
    SolverPGSSM(RigidBodySystem* _rigidBodySystem);

    // Implement PGS method that solves for the constraint impulses in @a m_rigidBodySystem.
    virtual void solve(float h) override;

private:
    std::vector<Eigen::MatrixXf> Ajoint;
    std::vector<Eigen::Matrix3f> Acontact;
    std::vector<Eigen::Vector3f> bcontact;
    std::vector<Eigen::VectorXf> bjoint;

    void solveJoints(std::vector<Joint*>& joints, int numJoints);
    void solveContacts(std::vector<Contact>& contacts, int numContacts);
    void updateIndexSets(std::vector<Contact>& contacts, std::vector<int>& L, std::vector<int>& U, std::vector<int>& A);
    void solveActiveJoints(std::vector<Joint*>& joints, std::vector<int>& A);
    void solveActiveContacts(std::vector<Contact>& contacts, std::vector<int>& A);
};
