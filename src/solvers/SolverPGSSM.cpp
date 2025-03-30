#include "solvers/SolverPGSSM.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"
#include "contact/Contact.h"
#include "joint/Joint.h"
#include <Eigen/Dense>
#include <iostream>
#include <omp.h>

namespace
{
    // Apply linear matrix-vector operations
    static inline void applyLinearMatrixVectorOperations(const Eigen::MatrixXf& G, const Eigen::VectorXf& x, const float a, Eigen::VectorXf& b)
    {
#pragma omp simd
        for (int i = 0; i < G.rows(); ++i)
        {
            b(i) -= a * (G.row(i).head<3>().dot(x.head<3>()));
        }
    }

    // Apply angular matrix-vector operations
    static inline void applyAngularMatrixVectorOperations(const Eigen::MatrixXf& G, const Eigen::VectorXf& y, const float a, Eigen::VectorXf& b)
    {
#pragma omp simd
        for (int i = 0; i < G.rows(); ++i)
        {
            b(i) -= a * (G.row(i).tail<3>().dot(y.tail<3>()));
        }
    }

    // Build RHS vector for constraints
    template <typename T, typename V>
    inline void buildRHS(T* obj, float h, V& b, float gamma)
    {
        b = -gamma * obj->phi / h; // b = -gamma * phi / h

        Eigen::VectorXf temp_b = b.cast<float>(); // Common for both Contact and Joint

        applyLinearMatrixVectorOperations(obj->J0, obj->body0->xdot, 1.0f, temp_b); // J0 * xdot0
        applyAngularMatrixVectorOperations(obj->J0, obj->body0->omega, 1.0f, temp_b); // J0 * omega0

        applyLinearMatrixVectorOperations(obj->J1, obj->body1->xdot, 1.0f, temp_b); // J1 * xdot1
        applyAngularMatrixVectorOperations(obj->J1, obj->body1->omega, 1.0f, temp_b); // J1 * omega1

        if (!obj->body0->fixed)
        {
            applyLinearMatrixVectorOperations(obj->J0Minv, obj->body0->f, h, temp_b); // J0Minv * f0
            applyAngularMatrixVectorOperations(obj->J0Minv, obj->body0->tau, h, temp_b); // J0Minv * tau0
        }
        if (!obj->body1->fixed)
        {
            applyLinearMatrixVectorOperations(obj->J1Minv, obj->body1->f, h, temp_b); // J1Minv * f1
            applyAngularMatrixVectorOperations(obj->J1Minv, obj->body1->tau, h, temp_b); // J1Minv * tau1
        }

        b = temp_b.cast<typename V::Scalar>();
    }

    static inline void solveContact(const Eigen::Matrix3f& A, const Eigen::Vector3f& b, Eigen::Vector3f& x, const float mu)
    {
        x(0) = std::max(0.0f, (b(0) - A(0, 1) * x(1) - A(0, 2) * x(2)) / A(0, 0));

        x(1) = std::max(-mu * x(0), std::min(mu * x(0), (b(1) - A(1, 0) * x(0) - A(1, 2) * x(2)) / A(1, 1)));
        x(2) = std::max(-mu * x(0), std::min(mu * x(0), (b(2) - A(2, 0) * x(0) - A(2, 1) * x(1)) / A(2, 2)));
    }

    template <typename VectorBlockType>
    static inline void solveJoint(const Eigen::MatrixXf& A, const Eigen::VectorXf& b, VectorBlockType& x)
    {
        x = A.ldlt().solve(b);
    }

    template<typename T>
    void buildDiagonalMatrix(const std::vector<T*>& objects, std::vector<Eigen::MatrixXf>& A)
    {
#pragma omp parallel for
        for (int i = 0; i < static_cast<int>(objects.size()); ++i)
        {
            T* obj = objects[i];
            const int dim = obj->dim;
            Eigen::MatrixXf matrix = Eigen::MatrixXf::Identity(dim, dim);
            if (!obj->body0->fixed)
            {
                matrix += obj->J0Minv.block(0, 0, dim, 6) * obj->J0.block(0, 0, dim, 6).transpose();
            }
            if (!obj->body1->fixed)
            {
                matrix += obj->J1Minv.block(0, 0, dim, 6) * obj->J1.block(0, 0, dim, 6).transpose();
            }
            A[i] = matrix;
        }
    }

    void buildDiagonalMatrixContacts(const std::vector<Contact*>& objects, std::vector<Eigen::Matrix3f>& A)
    {
#pragma omp parallel for
        for (int i = 0; i < static_cast<int>(objects.size()); ++i)
        {
            Contact* obj = objects[i];
            Eigen::Matrix3f matrix = Eigen::Matrix3f::Zero();
            if (!obj->body0->fixed)
            {
                matrix += obj->J0Minv * obj->J0.transpose();
            }
            if (!obj->body1->fixed)
            {
                matrix += obj->J1Minv * obj->J1.transpose();
            }
            A[i] = matrix;
        }
    }

    template<typename T>
    void computeRHSVector(std::vector<T*>& objects, std::vector<Eigen::VectorXf>& b, float h, float gamma)
    {
#pragma omp parallel for
        for (int i = 0; i < static_cast<int>(objects.size()); ++i)
        {
            T* obj = objects[i];
            b[i] = Eigen::VectorXf::Zero(obj->dim);
            buildRHS(obj, h, b[i], gamma);
            obj->lambda.setZero();
        }
    }

    void computeRHSVectorContacts(std::vector<Contact*>& objects, std::vector<Eigen::Vector3f>& b, float h, float gamma)
    {
#pragma omp parallel for
        for (int i = 0; i < static_cast<int>(objects.size()); ++i)
        {
            Contact* obj = objects[i];
            b[i] = Eigen::Vector3f::Zero();
            buildRHS(obj, h, b[i], gamma);
            obj->lambda.setZero();
        }
    }
}

SolverPGSSM::SolverPGSSM(RigidBodySystem* _rigidBodySystem) : Solver(_rigidBodySystem)
{
}

void SolverPGSSM::solveActiveJoints(std::vector<Joint*>& joints, std::vector<int>& A)
{
#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(A.size()); ++i)
    {
        int idx = A[i];
        if (idx >= static_cast<int>(joints.size()))
            continue;
        Joint* j = joints[idx];
        solveJoint(Ajoint[idx], bjoint[idx], j->lambda);
    }
}

void SolverPGSSM::solveActiveContacts(std::vector<Contact>& contacts, std::vector<int>& A)
{
#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(A.size()); ++i)
    {
        int idx = A[i];
        if (idx >= static_cast<int>(contacts.size()))
            continue;
        Contact* c = &contacts[idx];
        solveContact(Acontact[idx], bcontact[idx], c->lambda, Contact::mu);
    }
}

void SolverPGSSM::solveJoints(std::vector<Joint*>& joints, int numJoints)
{
#pragma omp parallel for
    for (int i = 0; i < numJoints; ++i)
    {
        Joint* j = joints[i];
        solveJoint(Ajoint[i], bjoint[i], j->lambda);
    }
}

void SolverPGSSM::solveContacts(std::vector<Contact>& contacts, int numContacts)
{
#pragma omp parallel for
    for (int i = 0; i < numContacts; ++i)
    {
        Contact* c = &contacts[i];
        solveContact(Acontact[i], bcontact[i], c->lambda, Contact::mu);
    }
}

void SolverPGSSM::updateIndexSets(std::vector<Contact>& contacts, std::vector<int>& L, std::vector<int>& U, std::vector<int>& A)
{
    L.clear();
    U.clear();
    A.clear();

    for (int i = 0; i < static_cast<int>(contacts.size()); ++i)
    {
        if (contacts[i].lambda(0) == 0.0f)
        {
            L.push_back(i);
        }
        else if (contacts[i].lambda(0) == Contact::mu)
        {
            U.push_back(i);
        }
        else
        {
            A.push_back(i);
        }
    }
}

void SolverPGSSM::solve(float h)
{
    std::vector<Contact>& contacts = m_rigidBodySystem->getContacts();
    std::vector<Joint*>& joints = m_rigidBodySystem->getJoints();
    const int numContacts = contacts.size();
    const int numJoints = joints.size();

    float gamma;
    gamma = m_rigidBodySystem->gamma;
    m_rigidBodySystem->gamma = gamma;

    if (numJoints > 0)
    {
        Ajoint.resize(numJoints);
        buildDiagonalMatrix(joints, Ajoint);
    }

    if (numContacts > 0)
    {
        Acontact.resize(numContacts);
        std::vector<Contact*> contactPtrs(numContacts);
        for (int i = 0; i < numContacts; ++i)
            contactPtrs[i] = &contacts[i];
        buildDiagonalMatrixContacts(contactPtrs, Acontact);
    }

    bcontact.resize(numContacts);
    std::vector<Contact*> contactPtrs(numContacts);
    for (int i = 0; i < numContacts; ++i)
        contactPtrs[i] = &contacts[i];
    computeRHSVectorContacts(contactPtrs, bcontact, h, gamma);

    bjoint.resize(numJoints);
    std::vector<Joint*> jointPtrs(numJoints);
    for (int i = 0; i < numJoints; ++i)
        jointPtrs[i] = joints[i];
    computeRHSVector(jointPtrs, bjoint, h, gamma);

    std::vector<int> L, U, A;

    // convergence loop
    for (int iter = 0; iter < m_maxIter; ++iter)
    {
        if (iter < 1)
        {
            solveJoints(joints, numJoints);
            solveContacts(contacts, numContacts);
            updateIndexSets(contacts, L, U, A);
        }
        for (int k = 0; k < m_subIter; ++k)
        {
            solveActiveJoints(joints, A);
            solveActiveContacts(contacts, A);
        }
    }
}
