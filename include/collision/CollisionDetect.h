#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "geometry/GeometryType.h"

// Forward declarations
class Contact;
class RigidBody;
class RigidBodySystem;
class BroadPhase;
class ImprovedCollisionRegistry;
class CollisionHandler;

/**
 * The main collision detection class.
 * Tests the geometries of each pair of rigid bodies
 * and populates an array with contacts.
 */
class CollisionDetect
{
public:
    /**
     * Constructor.
     * @param rigidBodySystem The rigid body system to detect collisions in
     * @param useBroadPhase If true, use broad phase collision detection
     */
    explicit CollisionDetect(RigidBodySystem* rigidBodySystem, bool useBroadPhase = true);

    /**
     * Destructor.
     */
    ~CollisionDetect();

    /**
     * Tests for collisions between all pairs of bodies in the rigid body system
     * and generates contacts for any intersecting geometries.
     */
    void detectCollisions();

    /**
     * Clear all contacts from previous detection.
     */
    void clear();

    /**
     * Compute the Jacobians for all contacts.
     */
    void computeContactJacobians();

    /**
     * Returns all contacts following the current collision detection pass (read-only).
     */
    const std::vector<Contact*>& getContacts() const { return m_contacts; }

    /**
     * Returns all contacts following the current collision detection pass.
     */
    std::vector<Contact*>& getContacts() { return m_contacts; }

    /**
     * Add a contact to the collection.
     */
    void addContact(Contact* contact) { m_contacts.push_back(contact); }

private:
    /**
     * Process a collision pair.
     * @param bodyA First rigid body
     * @param bodyB Second rigid body
     */
    void processCollisionPair(RigidBody* bodyA, RigidBody* bodyB);

private:
    RigidBodySystem* m_rigidBodySystem;           // The rigid body system
    std::vector<Contact*> m_contacts;             // Cached array of contacts
    std::unique_ptr<BroadPhase> m_broadPhase;     // Broad phase algorithm
    ImprovedCollisionRegistry* m_registry;        // Collision handler registry
    bool m_useBroadPhase;                         // Whether to use broad phase
};