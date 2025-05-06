#include "rigidbody/RigidBodySystem.h"
#include "collision/CollisionDetect.h"
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "solvers/SolverBoxPGS.h"
#include "solvers/SolverConjGradient.h"
#include "solvers/SolverConjResidual.h"
#include "solvers/SolverPGSSM.h"

#ifdef USE_OPENMP
# include <omp.h>
#endif

namespace {
    static Solver* s_solvers[4] = { nullptr, nullptr, nullptr, nullptr };
}

RigidBodySystem::RigidBodySystem()
 : m_collisionsEnabled(true)
 , m_gravity(0.0f, -9.81f, 0.0f)
 , m_solverType(SolverType::PGS)
 , m_solverIter(10)
 , m_integrationMethod(IntegrationMethod::EXPLICIT_EULER)
{
    m_collisionDetect = std::make_unique<CollisionDetect>(this);
    s_solvers[0] = new SolverBoxPGS(this);
    s_solvers[1] = new SolverConjGradient(this);
    s_solvers[2] = new SolverConjResidual(this);
    s_solvers[3] = new SolverPGSSM(this);
}

RigidBodySystem::~RigidBodySystem() {
    clear();
    for (auto& sol : s_solvers) {
        delete sol;
        sol = nullptr;
    }
}

void RigidBodySystem::setSolverType(SolverType type) {
    m_solverType = type;
}
SolverType RigidBodySystem::getSolverType() const {
    return m_solverType;
}

void RigidBodySystem::setIntegrationMethod(IntegrationMethod method) {
    m_integrationMethod = method;
}
IntegrationMethod RigidBodySystem::getIntegrationMethod() const {
    return m_integrationMethod;
}

void RigidBodySystem::addBody(RigidBody* b) {
    m_bodies.push_back(b);
}
void RigidBodySystem::addJoint(Joint* _j) {
    m_joints.push_back(_j);
    if (_j->body0) _j->body0->joints.push_back(_j);
    if (_j->body1) _j->body1->joints.push_back(_j);
}

void RigidBodySystem::step(float dt)
{
#ifdef USE_OPENMP
    #pragma omp parallel for if(m_bodies.size() > 16)
#endif
    for(size_t i = 0; i < m_bodies.size(); ++i) {
        auto b = m_bodies[i];
        b->f    = b->mass * m_gravity;
        b->tau.setZero();
        b->fc.setZero();
        b->tauc.setZero();
        b->contacts.clear();
    }

    computeInertias();
    if(m_preStepFunc) m_preStepFunc(m_bodies);

    m_collisionDetect->clear();
    if (m_collisionsEnabled) {
        m_collisionDetect->detectCollisions();
        m_collisionDetect->computeContactJacobians();
    }

#ifdef USE_OPENMP
    #pragma omp parallel for if(m_joints.size() > 16)
#endif
    for (size_t i = 0; i < m_joints.size(); ++i)
        m_joints[i]->computeJacobian();

#ifdef USE_OPENMP
    #pragma omp parallel for if(m_bodies.size() > 16)
#endif
    for(size_t i = 0; i < m_bodies.size(); ++i) {
        auto b = m_bodies[i];
        b->fc.setZero();
        b->tauc.setZero();
    }

    calcConstraintForces(dt);

    switch (m_integrationMethod) {
        case IntegrationMethod::EXPLICIT_EULER:
            integrateExplicitEuler(dt);   break;
        case IntegrationMethod::SYMPLECTIC_EULER:
            integrateSymplecticEuler(dt); break;
        case IntegrationMethod::VERLET:
            integrateVerlet(dt);          break;
        case IntegrationMethod::RK4:
            integrateRK4(dt);             break;
        case IntegrationMethod::IMPLICIT_EULER:
            integrateImplicitEuler(dt);   break;
        default:
            integrateExplicitEuler(dt);   break;
    }
}

// ——— Explicit Euler —————————————————————————————————————————————————————
void RigidBodySystem::integrateExplicitEuler(float dt) {
#ifdef USE_OPENMP
    #pragma omp parallel for if(m_bodies.size() > 16)
#endif
    for (auto b : m_bodies) {
        if (!b->fixed) {
            b->xdot  += dt * (1.0f/b->mass) * (b->f + b->fc);
            b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

            b->x += dt * b->xdot;

            Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
            Eigen::Quaternionf qDot = omegaQ * b->q;
            qDot.coeffs() *= 0.5f;
            b->q.coeffs() += dt * qDot.coeffs();
            b->q.normalize();
        } else {
            b->xdot.setZero();
            b->omega.setZero();
        }
    }
}

// ——— Symplectic Euler ——————————————————————————————————————————————————
void RigidBodySystem::integrateSymplecticEuler(float dt) {
#ifdef USE_OPENMP
    #pragma omp parallel for if(m_bodies.size() > 16)
#endif
    for (auto b : m_bodies) {
        if (!b->fixed) {
            b->xdot  += dt * (1.0f/b->mass) * (b->f + b->fc);
            b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

            b->x += dt * b->xdot;

            Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
            Eigen::Quaternionf qDot = omegaQ * b->q;
            qDot.coeffs() *= 0.5f;
            b->q.coeffs() += dt * qDot.coeffs();
            b->q.normalize();
        } else {
            b->xdot.setZero();
            b->omega.setZero();
        }
    }
}

// ——— Verlet ——————————————————————————————————————————————————————————
void RigidBodySystem::integrateVerlet(float dt){
    static bool first = true;
    static std::vector<Eigen::Vector3f> prev;
    if (first) {
        prev.resize(m_bodies.size());
        for (size_t i = 0; i < m_bodies.size(); ++i)
            prev[i] = m_bodies[i]->x - dt * m_bodies[i]->xdot;
        first = false;
    }

#ifdef USE_OPENMP
    #pragma omp parallel for if(m_bodies.size() > 16)
#endif
    for (size_t i = 0; i < m_bodies.size(); ++i) {
        auto b = m_bodies[i];
        if (!b->fixed) {
            Eigen::Vector3f a = (1.0f/b->mass) * (b->f + b->fc);
            Eigen::Vector3f curr = b->x;
            b->x = 2.0f*curr - prev[i] + dt*dt*a;
            b->xdot = (b->x - prev[i])/(2.0f*dt);
            prev[i] = curr;

            b->omega += dt * b->Iinv * (b->tau + b->tauc - b->omega.cross(b->I * b->omega));

            Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
            Eigen::Quaternionf qDot = omegaQ * b->q;
            qDot.coeffs() *= 0.5f;
            b->q.coeffs() += dt * qDot.coeffs();
            b->q.normalize();
        } else {
            b->xdot.setZero();
            b->omega.setZero();
            prev[i] = b->x;
        }
    }
}

// ——— Runge–Kutta 4 —————————————————————————————————————————————————————
// Note: signature is no longer const, so we can call computeInertias() & preStep
void RigidBodySystem::integrateRK4(float dt) {
    size_t N = m_bodies.size();
    std::vector<Eigen::Vector3f> x0(N), v0(N), omega0(N);
    std::vector<Eigen::Quaternionf> q0(N);
    std::vector<Eigen::Vector3f> k1x(N), k1v(N), k1w(N),
                               k2x(N), k2v(N), k2w(N),
                               k3x(N), k3v(N), k3w(N),
                               k4x(N), k4v(N), k4w(N);

    // save state
    for (size_t i = 0; i < N; ++i) {
        auto b = m_bodies[i];
        x0[i]     = b->x;
        v0[i]     = b->xdot;
        q0[i]     = b->q;
        omega0[i] = b->omega;
    }

    // --- k1 ---
    for (size_t i = 0; i < N; ++i) {
        auto b = m_bodies[i];
        if (!b->fixed) {
            k1x[i] = dt * b->xdot;
            Eigen::Vector3f a = (1.0f/b->mass)*(b->f + b->fc);
            k1v[i] = dt * a;
            Eigen::Vector3f alpha = b->Iinv*(b->tau + b->tauc - b->omega.cross(b->I*b->omega));
            k1w[i] = dt * alpha;
        } else {
            k1x[i].setZero();
            k1v[i].setZero();
            k1w[i].setZero();
        }
    }

    // apply k1 → compute k2
    for (size_t i = 0; i < N; ++i) {
        auto b = m_bodies[i];
        if (!b->fixed) {
            b->x    = x0[i] + 0.5f * k1x[i];
            b->xdot = v0[i] + 0.5f * k1v[i];
            b->omega = omega0[i] + 0.5f * k1w[i];

            Eigen::Quaternionf omegaQ(0,
                k1w[i].x(),
                k1w[i].y(),
                k1w[i].z());
            Eigen::Quaternionf qDot = omegaQ * q0[i];
            qDot.coeffs() *= 0.5f;
            Eigen::Vector4f cq = q0[i].coeffs() + dt * qDot.coeffs();
            b->q.coeffs() = cq;
            b->q.normalize();
        }
    }

    computeInertias();
    if (m_preStepFunc) m_preStepFunc(m_bodies);

    for (size_t i = 0; i < N; ++i) {
        auto b = m_bodies[i];
        if (!b->fixed) {
            k2x[i] = dt * b->xdot;
            Eigen::Vector3f a = (1.0f/b->mass)*(b->f + b->fc);
            k2v[i] = dt * a;
            Eigen::Vector3f alpha = b->Iinv*(b->tau + b->tauc - b->omega.cross(b->I*b->omega));
            k2w[i] = dt * alpha;
        } else {
            k2x[i].setZero();
            k2v[i].setZero();
            k2w[i].setZero();
        }
    }

    // apply k2 → compute k3
    for (size_t i = 0; i < N; ++i) {
        auto b = m_bodies[i];
        if (!b->fixed) {
            b->x    = x0[i] + 0.5f * k2x[i];
            b->xdot = v0[i] + 0.5f * k2v[i];
            b->omega = omega0[i] + 0.5f * k2w[i];

            Eigen::Quaternionf omegaQ(0,
                k2w[i].x(),
                k2w[i].y(),
                k2w[i].z());
            Eigen::Quaternionf qDot = omegaQ * q0[i];
            qDot.coeffs() *= 0.5f;
            Eigen::Vector4f cq = q0[i].coeffs() + dt * qDot.coeffs();
            b->q.coeffs() = cq;
            b->q.normalize();
        }
    }

    computeInertias();
    if (m_preStepFunc) m_preStepFunc(m_bodies);

    for (size_t i = 0; i < N; ++i) {
        auto b = m_bodies[i];
        if (!b->fixed) {
            k3x[i] = dt * b->xdot;
            Eigen::Vector3f a = (1.0f/b->mass)*(b->f + b->fc);
            k3v[i] = dt * a;
            Eigen::Vector3f alpha = b->Iinv*(b->tau + b->tauc - b->omega.cross(b->I*b->omega));
            k3w[i] = dt * alpha;
        } else {
            k3x[i].setZero();
            k3v[i].setZero();
            k3w[i].setZero();
        }
    }

    // apply k3 → compute k4
    for (size_t i = 0; i < N; ++i) {
        auto b = m_bodies[i];
        if (!b->fixed) {
            b->x    = x0[i] + k3x[i];
            b->xdot = v0[i] + k3v[i];
            b->omega = omega0[i] + k3w[i];

            Eigen::Quaternionf omegaQ(0,
                k3w[i].x(),
                k3w[i].y(),
                k3w[i].z());
            Eigen::Quaternionf qDot = omegaQ * q0[i];
            qDot.coeffs() *= 0.5f;
            Eigen::Vector4f cq = q0[i].coeffs() + dt * qDot.coeffs();
            b->q.coeffs() = cq;
            b->q.normalize();
        }
    }

    computeInertias();
    if (m_preStepFunc) m_preStepFunc(m_bodies);

    for (size_t i = 0; i < N; ++i) {
        auto b = m_bodies[i];
        if (!b->fixed) {
            k4x[i] = dt * b->xdot;
            Eigen::Vector3f a = (1.0f/b->mass)*(b->f + b->fc);
            k4v[i] = dt * a;
            Eigen::Vector3f alpha = b->Iinv*(b->tau + b->tauc - b->omega.cross(b->I*b->omega));
            k4w[i] = dt * alpha;
        } else {
            k4x[i].setZero();
            k4v[i].setZero();
            k4w[i].setZero();
        }
    }

    // final aggregate
    for (size_t i = 0; i < N; ++i) {
        auto b = m_bodies[i];
        if (!b->fixed) {
            b->x     = x0[i] + (k1x[i] + 2*k2x[i] + 2*k3x[i] + k4x[i]) / 6.0f;
            b->xdot  = v0[i] + (k1v[i] + 2*k2v[i] + 2*k3v[i] + k4v[i]) / 6.0f;
            b->omega = omega0[i] + (k1w[i] + 2*k2w[i] + 2*k3w[i] + k4w[i]) / 6.0f;

            Eigen::Quaternionf omegaQ(0,
                b->omega.x(),
                b->omega.y(),
                b->omega.z());
            Eigen::Quaternionf qDot = omegaQ * q0[i];
            qDot.coeffs() *= 0.5f;
            Eigen::Vector4f cq = q0[i].coeffs() + dt * qDot.coeffs();
            b->q.coeffs() = cq;
            b->q.normalize();
        }
    }
}

// ——— Implicit Euler ————————————————————————————————————————————————————
void RigidBodySystem::integrateImplicitEuler(float dt) {
    size_t N = m_bodies.size();
    std::vector<Eigen::Vector3f> acc(N), angAcc(N);

    for (size_t i = 0; i < N; ++i) {
        auto b = m_bodies[i];
        if (!b->fixed) {
            acc[i]    = (1.0f/b->mass)*(b->f + b->fc);
            angAcc[i] = b->Iinv*(b->tau + b->tauc - b->omega.cross(b->I*b->omega));
        }
    }
    for (size_t i = 0; i < N; ++i) {
        auto b = m_bodies[i];
        if (!b->fixed) {
            b->xdot  += dt * acc[i];
            b->omega += dt * angAcc[i];
        }
    }
    for (auto b : m_bodies) {
        if (!b->fixed) {
            b->x += dt * b->xdot;
            Eigen::Quaternionf omegaQ(0, b->omega.x(), b->omega.y(), b->omega.z());
            Eigen::Quaternionf qDot = omegaQ * b->q;
            qDot.coeffs() *= 0.5f;
            b->q.coeffs() += dt * qDot.coeffs();
            b->q.normalize();
        }
    }
}

void RigidBodySystem::clear() {
    if (m_resetFunc) m_resetFunc();
    m_collisionDetect->clear();
    for (auto j : m_joints) delete j;
    m_joints.clear();
    for (auto b : m_bodies) delete b;
    m_bodies.clear();
}

void RigidBodySystem::computeInertias() {
#ifdef USE_OPENMP
    #pragma omp parallel for if(m_bodies.size() > 16)
#endif
    for (size_t i = 0; i < m_bodies.size(); ++i)
        m_bodies[i]->updateInertiaMatrix();
}

const std::vector<Contact*>& RigidBodySystem::getContacts() const {
    return m_collisionDetect->getContacts();
}
std::vector<Contact*>& RigidBodySystem::getContacts() {
    return m_collisionDetect->getContacts();
}

void RigidBodySystem::calcConstraintForces(float dt) {
    int idx = static_cast<int>(m_solverType);
    s_solvers[idx]->setMaxIter(m_solverIter);
    s_solvers[idx]->solve(dt);

#ifdef USE_OPENMP
    #pragma omp parallel for if(m_joints.size() > 16)
#endif
    for (auto j : m_joints) {
        Eigen::Vector6f f0 = j->J0.transpose() * j->lambda / dt;
        Eigen::Vector6f f1 = j->J1.transpose() * j->lambda / dt;
        #pragma omp critical
        {
            j->body0->fc  += f0.head<3>();
            j->body0->tauc+= f0.tail<3>();
            j->body1->fc  += f1.head<3>();
            j->body1->tauc+= f1.tail<3>();
        }
    }

    auto contacts = m_collisionDetect->getContacts();
#ifdef USE_OPENMP
    #pragma omp parallel for if(contacts.size() > 16)
#endif
    for (auto c : contacts) {
        Eigen::Vector6f f0 = c->J0.transpose() * c->lambda / dt;
        Eigen::Vector6f f1 = c->J1.transpose() * c->lambda / dt;
        if (!c->body0->fixed) {
            #pragma omp critical
            {
                c->body0->fc   += f0.head<3>();
                c->body0->tauc += f0.tail<3>();
            }
        }
        if (!c->body1->fixed) {
            #pragma omp critical
            {
                c->body1->fc   += f1.head<3>();
                c->body1->tauc += f1.tail<3>();
            }
        }
    }
}
