#include "collision/CollisionDetect.h"
#include "collision/ImprovedCollisionRegistry.h"
#include "collision/BroadPhase.h"
#include "collision/CollisionHandler.h"
#include "collision/CollisionPair.h"
#include "collision/CollisionFactory.h"
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <algorithm>

CollisionDetect::CollisionDetect(RigidBodySystem* rigidBodySystem, bool useBroadPhase) 
    : m_rigidBodySystem(rigidBodySystem)
    , m_contacts()
    , m_broadPhase(useBroadPhase ? BroadPhaseFactory::create("sap") : nullptr)
    , m_registry(&ImprovedCollisionRegistry::getInstance())
    , m_useBroadPhase(useBroadPhase)
{
    // Initialize the collision registry
    m_registry->initialize();
}

CollisionDetect::~CollisionDetect() {
    clear();
}

void CollisionDetect::detectCollisions() {
    // Clear previous contacts
    clear();
    
    // Get the bodies
    auto& bodies = m_rigidBodySystem->getBodies();
    
    if (m_useBroadPhase && m_broadPhase) {
        // Update the broad phase
        m_broadPhase->update(m_rigidBodySystem);
        
        // Generate potential collision pairs
        auto pairs = m_broadPhase->generatePairs(m_rigidBodySystem);
        
        // Process each pair
        for (const auto& pair : pairs) {
            if (pair.isValid()) {
                processCollisionPair(bodies[pair.bodyA], bodies[pair.bodyB]);
            }
        }
    } else {
        // No broad phase - check all pairs (N^2 complexity)
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                RigidBody* bodyA = bodies[i];
                RigidBody* bodyB = bodies[j];
                
                // Skip if both bodies are fixed
                if (bodyA->fixed && bodyB->fixed) {
                    continue;
                }
                
                processCollisionPair(bodyA, bodyB);
            }
        }
    }
}

void CollisionDetect::processCollisionPair(RigidBody* bodyA, RigidBody* bodyB) {
    // Get geometry types
    GeometryType typeA = static_cast<GeometryType>(bodyA->geometry->getType());
    GeometryType typeB = static_cast<GeometryType>(bodyB->geometry->getType());
    
    // Get the appropriate collision handler
    auto handler = m_registry->getHandler(typeA, typeB);
    
    if (handler) {
        // Ensure correct order based on geometry types
        if (typeA <= typeB) {
            handler->detectCollision(bodyA, bodyB, this);
        } else {
            handler->detectCollision(bodyB, bodyA, this);
        }
    }
}

void CollisionDetect::computeContactJacobians() {
    for (auto c : m_contacts) {
        c->computeContactFrame();
        c->computeJacobian();
    }
}

void CollisionDetect::clear() {
    // Delete all contacts
    for (auto c : m_contacts) {
        delete c;
    }
    m_contacts.clear();

    // Clear contacts from bodies
    auto bodies = m_rigidBodySystem->getBodies();
    for (auto b : bodies) {
        b->contacts.clear();
    }
}