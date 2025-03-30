#include "solvers/SolverBoxPGS.h"

#include "contact/Contact.h"
#include "joint/Joint.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <Eigen/Dense>
#include <iostream>

#ifdef USE_OPENMP
#include <omp.h>
#endif

namespace
{
    static inline void applyLinearMatrixVectorOperations(const Eigen::MatrixXf &G, const Eigen::Vector3f &x, const float a, Eigen::VectorXf &b)
    {
        for (int i = 0; i < G.rows(); ++i)
        {
            b(i) -= a * (G.row(i).head<3>().dot(x));
        }
    }

    static inline void applyAngularMatrixVectorOperations(const Eigen::MatrixXf &G, const Eigen::Vector3f &y, const float a, Eigen::VectorXf &b)
    {
        for (int i = 0; i < G.rows(); ++i)
        {
            b(i) -= a * (G.row(i).tail<3>().dot(y));
        }
    }

    // Build RHS vector for constraints
    template <typename T>
    inline void buildRHS(T *obj, float h, Eigen::VectorXf &b, float gamma)
    {
        const float hinv = 1.0f / h;
        b = -hinv * gamma * obj->phi;

        if (!obj->body0->fixed)
        {
            applyLinearMatrixVectorOperations(obj->J0Minv, obj->body0->f, h, b);
            applyAngularMatrixVectorOperations(obj->J0Minv, obj->body0->tau, h, b);
            applyLinearMatrixVectorOperations(obj->J0, obj->body0->xdot, 1.0f, b);
            applyAngularMatrixVectorOperations(obj->J0, obj->body0->omega, 1.0f, b);
        }
        if (!obj->body1->fixed)
        {
            applyLinearMatrixVectorOperations(obj->J1Minv, obj->body1->f, h, b);
            applyAngularMatrixVectorOperations(obj->J1Minv, obj->body1->tau, h, b);
            applyLinearMatrixVectorOperations(obj->J1, obj->body1->xdot, 1.0f, b);
            applyAngularMatrixVectorOperations(obj->J1, obj->body1->omega, 1.0f, b);
        }
    }

    // Loop over all other contacts for a body and compute modifications to the rhs vector b
    static inline void accumulateCoupledContactsAndJoints(Joint* j, const JBlock& JMinv, RigidBody* body, Eigen::VectorXf& b)
    {
        if (body->fixed)
            return;

        for (Contact* cc : body->contacts)
        {
            if (cc != j)
            {
                if (body == cc->body0)
                    b -= JMinv * (cc->J0.transpose() * cc->lambda);
                else
                    b -= JMinv * (cc->J1.transpose() * cc->lambda);
            }
        }

        for (Joint* jj : body->joints)
        {
            if (jj != j)
            {
                if (body == jj->body0)
                {
                    b -= JMinv * (jj->J0.transpose() * jj->lambda);
                }
                else
                {
                    b -= JMinv * (jj->J1.transpose() * jj->lambda);
                }
            }
        }
    }

    // Solve the Boxed LCP problem for a single contact and isotropic Coulomb friction.
    static inline void solveContact(const Eigen::Matrix3f& A, const Eigen::VectorXf& b, Eigen::VectorXf& x, const float mu)
    {
        // Normal impulse is projected to [0, inf]
        x(0) = (b(0) - A(0, 1) * x(1) - A(0, 2) * x(2)) / (A(0, 0) + 1e-3f);
        if (x(0) < 0.0f) x(0) = 0.0f;

        // Next, friction impulses are projected to [-mu * x(0), mu * x(1)]
        const float lowerx = -mu * x(0);
        const float upperx = mu * x(0);
        
        x(1) = (b(1) - A(1, 0) * x(0) - A(1, 2) * x(2)) / A(1, 1);
        if (x(1) < lowerx) x(1) = lowerx;
        else if (x(1) > upperx) x(1) = upperx;

        x(2) = (b(2) - A(2, 0) * x(0) - A(2, 1) * x(1)) / A(2, 2);
        if (x(2) < lowerx) x(2) = lowerx;
        else if (x(2) > upperx) x(2) = upperx;
    }

    static inline void solveJoint(const Eigen::LDLT<Eigen::MatrixXf>& LLT, const Eigen::VectorXf& b, Eigen::VectorXf& x)
    {
        x = LLT.solve(b);
    }
}

SolverBoxPGS::SolverBoxPGS(RigidBodySystem* _rigidBodySystem) : Solver(_rigidBodySystem)
{
}

void SolverBoxPGS::solve(float h)
{
    std::vector<Contact*>& contacts = m_rigidBodySystem->getContacts();
    std::vector<Joint*>& joints = m_rigidBodySystem->getJoints();
    const int numContacts = contacts.size();
    const int numJoints = joints.size();
    const int N = numJoints + numContacts;
    
    // Get gamma value from the system
    float gamma = m_rigidBodySystem->gamma;

    // Build diagonal matrices of bilateral joints
    std::vector<Eigen::LDLT<Eigen::MatrixXf>> LLTjointii;
    if (numJoints > 0)
    {
        // Build diagonal matrices
        LLTjointii.resize(numJoints);
        
        #ifdef USE_OPENMP
        #pragma omp parallel for
        #endif
        for (int i = 0; i < numJoints; ++i)
        {
            Joint* j = joints[i];
            const int dim = j->lambda.rows();
            const float eps = 1e-5f;

            // Compute the diagonal term : Aii = J0*Minv0*J0^T + J1*Minv1*J1^T
            Eigen::MatrixXf A = eps * Eigen::MatrixXf::Identity(dim, dim);

            if (!j->body0->fixed)
            {
                JBlockTranspose JT = j->J0.transpose();
                A += j->J0Minv * JT;
            }
            if (!j->body1->fixed)
            {
                JBlockTranspose JT = j->J1.transpose();
                A += j->J1Minv * JT;
            }

            LLTjointii[i] = A.ldlt();
        }
    }

    // Build array of 3x3 diagonal matrices, one for each contact.
    std::vector<Eigen::Matrix3f> Acontactii;
    if (numContacts > 0)
    {
        // Build diagonal matrices
        Acontactii.resize(numContacts);
        
        #ifdef USE_OPENMP
        #pragma omp parallel for
        #endif
        for (int i = 0; i < numContacts; ++i)
        {
            Contact* c = contacts[i];

            // Compute the diagonal term : Aii = J0*Minv0*J0^T + J1*Minv1*J1^T
            Acontactii[i].setZero(3, 3);

            if (!c->body0->fixed)
            {
                Acontactii[i] += c->J0Minv * c->J0.transpose();
            }
            if (!c->body1->fixed)
            {
                Acontactii[i] += c->J1Minv * c->J1.transpose();
            }
        }
    }

    if (N > 0)
    {
        std::vector<Eigen::VectorXf> b;
        b.resize(N);

        // Compute the right-hand side vector for all constraints
        #ifdef USE_OPENMP
        #pragma omp parallel for
        #endif
        for (int i = 0; i < numJoints; ++i)
        {
            Joint* j = joints[i];
            b[i].resize(j->lambda.rows());
            buildRHS<Joint>(j, h, b[i], gamma);
        }

        #ifdef USE_OPENMP
        #pragma omp parallel for
        #endif
        for (int i = 0; i < numContacts; ++i)
        {
            Contact* c = contacts[i];
            b[i+numJoints].resize(3);
            buildRHS<Contact>(c, h, b[i+numJoints], gamma);
            c->lambda.setZero();
        }

        // PGS main loop with convergence tracking
        float residual = std::numeric_limits<float>::max();
        float tolerance = 1e-6f;
        int iter = 0;
        
        for (; iter < m_maxIter && residual > tolerance; ++iter)
        {
            residual = 0.0f;
            
            // For each joint, compute an updated value of lambda
            for (int i = 0; i < numJoints; ++i)
            {
                Joint* j = joints[i];
                const int dim = j->lambda.rows();
                
                // Store old lambda for convergence check
                Eigen::VectorXf old_lambda = j->lambda;

                // Initialize current solution as x = b[i]
                Eigen::VectorXf x = b[i];

                accumulateCoupledContactsAndJoints(j, j->J0Minv, j->body0, x);
                accumulateCoupledContactsAndJoints(j, j->J1Minv, j->body1, x);
                solveJoint(LLTjointii[i], x, j->lambda);
                
                // Update residual for convergence check
                residual += (j->lambda - old_lambda).squaredNorm();
            }

            // For each contact, compute an updated value of lambda
            for (int i = 0; i < numContacts; ++i)
            {
                Contact* c = contacts[i];
                
                // Store old lambda for convergence check
                Eigen::VectorXf old_lambda = c->lambda;

                // Initialize current solution as x = b[i]
                Eigen::VectorXf x = b[i+numJoints];

                accumulateCoupledContactsAndJoints(c, c->J0Minv, c->body0, x);
                accumulateCoupledContactsAndJoints(c, c->J1Minv, c->body1, x);
                solveContact(Acontactii[i], x, c->lambda, c->mu);
                
                // Update residual for convergence check
                residual += (c->lambda - old_lambda).squaredNorm();
            }
        }
        
        // Report convergence information for debugging
        if (iter < m_maxIter) {
            // std::cout << "PGS converged in " << iter << " iterations with residual " << residual << std::endl;
        }
    }
}


