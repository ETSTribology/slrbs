#include "solvers/SolverConjResidual.h"

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

    static inline void accumulateCoupled(const Joint *j, const JBlock &JMinv, const RigidBody *body,
                                         const Eigen::VectorXf &x, Eigen::VectorXf &Ax)
    {
        for (Joint *jj : body->joints)
        {
            if (jj == j) continue;
            const auto segOther = x.segment(jj->idx, jj->dim);
            Ax.segment(j->idx, j->dim) += JMinv * ((body == jj->body0 ? jj->J0 : jj->J1).transpose() * segOther);
        }
    }

    static inline void computeAx(const std::vector<Joint *> &joints, const Eigen::VectorXf &x, Eigen::VectorXf &Ax)
    {
        constexpr float eps = 1e-9f;   // to keep A positive‑definite
        Ax.setZero();
        for (Joint *j : joints)
        {
            Ax.segment(j->idx, j->dim).noalias() += eps * x.segment(j->idx, j->dim);

            const RigidBody *body0 = j->body0;
            const RigidBody *body1 = j->body1;

            if (!body0->fixed)
            {
                Ax.segment(j->idx, j->dim) += j->J0Minv * (j->J0.transpose() * x.segment(j->idx, j->dim));
                accumulateCoupled(j, j->J0Minv, body0, x, Ax);
            }
            if (!body1->fixed)
            {
                Ax.segment(j->idx, j->dim) += j->J1Minv * (j->J1.transpose() * x.segment(j->idx, j->dim));
                accumulateCoupled(j, j->J1Minv, body1, x, Ax);
            }
        }
    }
} // anonymous namespace

SolverConjResidual::SolverConjResidual(RigidBodySystem *system) : Solver(system) {}

void SolverConjResidual::solve(float h)
{
    const auto &joints = m_rigidBodySystem->getJoints();

    unsigned int idx = 0;
    for (Joint *j : joints) { j->idx = idx; idx += j->dim; }

    Eigen::VectorXf x(idx), r(idx), p(idx), b(idx), Ax(idx), Ap(idx), Ar(idx), hi(idx), lo(idx);

    x.setZero();
    hi.setConstant(std::numeric_limits<float>::max());
    lo.setConstant(-std::numeric_limits<float>::max());
    buildRHS(joints, h, b);

    computeAx(joints, x, Ax);
    r = b - Ax;
    p = r;
    computeAx(joints, p, Ap);
    computeAx(joints, r, Ar);

    float rAr   = r.dot(Ar);
    float pATAp = Ap.dot(Ap);

    for (int iter = 0; iter < m_maxIter && rAr > 1e-12f && pATAp > 1e-12f; ++iter)
    {
        const float alpha = rAr / pATAp;
        x += alpha * p;
        r -= alpha * Ap;

        computeAx(joints, r, Ar);
        const float rArNext = r.dot(Ar);

        const float beta = rArNext / rAr;
        p  = r + beta * p;
        Ap = Ar + beta * Ap;

        rAr   = rArNext;
        pATAp = Ap.dot(Ap);
    }

    for (Joint *j : joints)
        j->lambda = x.segment(j->idx, j->dim);
}
