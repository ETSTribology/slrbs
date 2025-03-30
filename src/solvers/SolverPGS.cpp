#include "solvers/SolverPGS.h"
#include "contact/Contact.h"
#include "joint/Joint.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"
#include <Eigen/Dense>
#include <iostream>
#include <omp.h>

namespace
{
    static inline void applyLinearMatrixVectorOperations(const Eigen::MatrixXf &G, const Eigen::VectorXf &x, const float a, Eigen::VectorXf &b)
    {
        for (int i = 0; i < G.rows(); ++i)
        {
            b(i) -= a * (G.row(i).head<3>().dot(x.head<3>()));
        }
    }

    static inline void applyAngularMatrixVectorOperations(const Eigen::MatrixXf &G, const Eigen::VectorXf &y, const float a, Eigen::VectorXf &b)
    {
        for (int i = 0; i < G.rows(); ++i)
        {
            b(i) -= a * (G.row(i).tail<3>().dot(y.tail<3>()));
        }
    }

    // Build RHS vector for constraints
    template <typename T, typename V>
    inline void buildRHS(T *obj, float h, V &b, float gamma)
    {
        b = -gamma * obj->phi / h; // b = -gamma * phi / h

        Eigen::VectorXf temp_b = b.cast<float>(); // Common for both Contact and Joint

        applyLinearMatrixVectorOperations(obj->J0, obj->body0->xdot, 1.0f, temp_b);   // J0 * xdot0
        applyAngularMatrixVectorOperations(obj->J0, obj->body0->omega, 1.0f, temp_b); // J0 * omega0

        applyLinearMatrixVectorOperations(obj->J1, obj->body1->xdot, 1.0f, temp_b);   // J1 * xdot1
        applyAngularMatrixVectorOperations(obj->J1, obj->body1->omega, 1.0f, temp_b); // J1 * omega1

        if (!obj->body0->fixed)
        {
            applyLinearMatrixVectorOperations(obj->J0Minv, obj->body0->f, h, temp_b);    // J0Minv * f0
            applyAngularMatrixVectorOperations(obj->J0Minv, obj->body0->tau, h, temp_b); // J0Minv * tau0
        }
        if (!obj->body1->fixed)
        {
            applyLinearMatrixVectorOperations(obj->J1Minv, obj->body1->f, h, temp_b);    // J1Minv * f1
            applyAngularMatrixVectorOperations(obj->J1Minv, obj->body1->tau, h, temp_b); // J1Minv * tau1
        }

        b = temp_b.cast<typename V::Scalar>();
    }

    static inline void solveContact(const Eigen::Matrix3f &A, const Eigen::Vector3f &b, Eigen::Vector3f &x, const float mu)
    {
        Eigen::Vector3f solve = A.ldlt().solve(b);
        x(0) = std::max(0.0f, solve(0));
        x(1) = std::max(-mu * x(0), std::min(mu * x(0), solve(1)));
        x(2) = std::max(-mu * x(0), std::min(mu * x(0), solve(2)));
    }

    template <typename VectorBlockType>
    static inline void solveJoint(const Eigen::MatrixXf &A, const Eigen::VectorXf &b, VectorBlockType &x)
    {
        x = A.ldlt().solve(b);
    }
}

SolverPGS::SolverPGS(RigidBodySystem *_rigidBodySystem) : Solver(_rigidBodySystem)
{
}

void SolverPGS::solve(float h)
{
    std::vector<Contact> &contacts = m_rigidBodySystem->getContacts();
    std::vector<Joint *> &joints = m_rigidBodySystem->getJoints();
    const int numContacts = contacts.size();
    const int numJoints = joints.size();

    float gamma = m_rigidBodySystem->gamma;
    m_rigidBodySystem->gamma = gamma;

    // Build diagonal matrices for joints
    if (numJoints > 0)
    {
        Ajoint.resize(numJoints);

        for (int i = 0; i < numJoints; ++i)
        {
            Joint *j = joints[i];
            const int dim = j->dim;
            Eigen::MatrixXf A = Eigen::MatrixXf::Identity(dim, dim);
            if (!j->body0->fixed)
            {
                A += j->J0Minv.block(0, 0, dim, 6) * j->J0.block(0, 0, dim, 6).transpose();
            }
            if (!j->body1->fixed)
            {
                A += j->J1Minv.block(0, 0, dim, 6) * j->J1.block(0, 0, dim, 6).transpose();
            }
            Ajoint[i] = A;
        }
    }

    // Build diagonal matrices for contacts
    if (numContacts > 0)
    {
        Acontact.resize(numContacts);

        for (int i = 0; i < numContacts; ++i)
        {
            Contact *c = &contacts[i];
            Acontact[i].setZero();

            if (!c->body0->fixed)
            {
                Acontact[i] += c->J0Minv * c->J0.transpose();
            }
            if (!c->body1->fixed)
            {
                Acontact[i] += c->J1Minv * c->J1.transpose();
            }
        }
    }

    // Compute the right-hand side vector for contacts
    bcontact.resize(numContacts);

    for (int i = 0; i < numContacts; ++i)
    {
        Contact *c = &contacts[i];
        c->lambda.setZero();
        bcontact[i] = Eigen::Vector3f::Zero();
        buildRHS<Contact>(c, h, bcontact[i], gamma);
    }

    // Compute the right-hand side vector for joints
    bjoint.resize(numJoints);

    for (int i = 0; i < numJoints; ++i)
    {
        Joint *j = joints[i];
        j->lambda.setZero();
        bjoint[i] = Eigen::VectorXf::Zero(j->dim);
        buildRHS<Joint>(j, h, bjoint[i], gamma);
    }

    // PGS main loop with Schur complement
    for (int iter = 0; iter < m_maxIter; ++iter)
    {
        // Solve for joint impulses

        for (int i = 0; i < numJoints; ++i)
        {
            Joint *j = joints[i];
            solveJoint(Ajoint[i], bjoint[i], j->lambda);
        }

        // Solve for contact impulses directly

        for (int i = 0; i < numContacts; ++i)
        {
            Contact *c = &contacts[i];
            solveContact(Acontact[i], bcontact[i], c->lambda, Contact::mu);
        }
    }
}
