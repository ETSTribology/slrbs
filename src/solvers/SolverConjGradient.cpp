#include "solvers/SolverConjGradient.h"

#include "contact/Contact.h"
#include "joint/Joint.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <Eigen/Dense>
#include <limits>

namespace
{
    static inline void multAndSub(const JBlock &G, const Eigen::Vector3f &x, const Eigen::Vector3f &y,
                                  float a, Eigen::VectorBlock<Eigen::VectorXf> &b)
    {
        b -= a * G.col(0) * x(0);
        b -= a * G.col(1) * x(1);
        b -= a * G.col(2) * x(2);
        b -= a * G.col(3) * y(0);
        b -= a * G.col(4) * y(1);
        b -= a * G.col(5) * y(2);
    }

    // Computes the right‑hand side vector of the Schur complement system:
    //      b = -phi/h - J*vel - h*JMinv*force
    static inline void buildRHS(const std::vector<Joint *> &joints, float h, Eigen::VectorXf &b)
    {
        const float hinv  = 1.0f / h;
        const float gamma = 0.3f;

        for (Joint *j : joints)
        {
            b.segment(j->idx, j->dim) = -hinv * gamma * j->phi;

            if (!j->body0->fixed)
            {
                auto seg = b.segment(j->idx, j->dim);
                multAndSub(j->J0Minv, j->body0->f,   j->body0->tau,  h,   seg);
                multAndSub(j->J0,      j->body0->xdot, j->body0->omega, 1.0f, seg);
            }
            if (!j->body1->fixed)
            {
                auto seg = b.segment(j->idx, j->dim);
                multAndSub(j->J1Minv, j->body1->f,   j->body1->tau,  h,   seg);
                multAndSub(j->J1,      j->body1->xdot, j->body1->omega, 1.0f, seg);
            }
        }
    }

    static inline void computeAx(const std::vector<Joint *> &joints, const Eigen::VectorXf &x, Eigen::VectorXf &Ax)
    {
        Ax.setZero();
        for (Joint *j : joints)
        {
            const RigidBody *body0 = j->body0;
            const RigidBody *body1 = j->body1;

            if (!body0->fixed)
            {
                Ax.segment(j->idx, j->dim) += j->J0Minv * (j->J0.transpose() * x.segment(j->idx, j->dim));

                for (Joint *jj : body0->joints)
                {
                    if (jj == j) continue;
                    const auto segOther = x.segment(jj->idx, jj->dim);
                    Ax.segment(j->idx, j->dim) += j->J0Minv * ((body0 == jj->body0 ? jj->J0 : jj->J1).transpose() * segOther);
                }
            }
            if (!body1->fixed)
            {
                Ax.segment(j->idx, j->dim) += j->J1Minv * (j->J1.transpose() * x.segment(j->idx, j->dim));

                for (Joint *jj : body1->joints)
                {
                    if (jj == j) continue;
                    const auto segOther = x.segment(jj->idx, jj->dim);
                    Ax.segment(j->idx, j->dim) += j->J1Minv * ((body1 == jj->body0 ? jj->J0 : jj->J1).transpose() * segOther);
                }
            }
        }
    }
} 

SolverConjGradient::SolverConjGradient(RigidBodySystem *system) : Solver(system) {}

void SolverConjGradient::solve(float h)
{
    const auto &joints = m_rigidBodySystem->getJoints();

    // Map each joint into the big vector
    unsigned int idx = 0;
    for (Joint *j : joints) { j->idx = idx; idx += j->dim; }

    Eigen::VectorXf x(idx), r(idx), p(idx), b(idx), Ax(idx), Ap(idx);
    x.setZero();
    buildRHS(joints, h, b);

    computeAx(joints, x, Ax);
    r = b - Ax;
    p = r;

    float rTr = r.dot(r);
    computeAx(joints, p, Ap);
    float pAp = p.dot(Ap);

    for (int iter = 0; iter < m_maxIter && rTr > 1e-12f && pAp > 1e-12f; ++iter)
    {
        const float alpha = rTr / pAp;
        x += alpha * p;
        r -= alpha * Ap;

        const float rTrNext = r.dot(r);
        const float beta    = rTrNext / rTr;
        p  = r + beta * p;
        computeAx(joints, p, Ap);
        pAp = Ap.dot(Ap);
        rTr = rTrNext;
    }

    for (Joint *j : joints)
        j->lambda = x.segment(j->idx, j->dim);
}