#pragma once

#include "joint/Joint.h"
#include <Eigen/Dense>
#include <unordered_map>
#include <string>

// Forward declarations
class RigidBody;
class MaterialPair;

/**
 * Contact constraint with box friction.
 * 
 * This class stores the contact normal @a n and contact point @a p,
 * as well as the Jacobians @a J0 and @a J1 for each body,
 * which are computed by computeJacobian().
 * 
 * Each contact manages its own friction coefficients based on the materials
 * of the contacting bodies. The transition between static and kinetic
 * friction is handled smoothly using a mollifier function.
 */
class Contact : public Joint
{
public: 
    /** Default static friction coefficient (used if material-specific values not set) */
    static float defaultStaticFriction;
    
    /** Default kinetic friction coefficient (used if material-specific values not set) */
    static float defaultKineticFriction;
    
    /** Velocity threshold for static/kinetic friction transition */
    static float frictionVelocityThreshold;

public:
    /**
     * Constructor with all parameters.
     * 
     * @param body0 First rigid body in contact
     * @param body1 Second rigid body in contact
     * @param p Contact point in world space
     * @param n Contact normal pointing from body0 to body1
     * @param penetration Penetration depth (negative if bodies are penetrating)
     */
    Contact(RigidBody* body0, RigidBody* body1, const Eigen::Vector3f& p, const Eigen::Vector3f& n, float penetration);

    /**
     * Destructor.
     */
    virtual ~Contact();

    /**
     * Get the constraint type.
     */
    virtual eConstraintType getType() const override { return kContact; }

    /**
     * Compute the Jacobian for this contact.
     */
    virtual void computeJacobian() override;

    /**
     * Compute a contact frame (normal and tangent vectors).
     */
    void computeContactFrame();
    
    /**
     * Calculate relative tangential velocity at the contact point.
     * 
     * @return The relative tangential velocity magnitude
     */
    float calculateRelativeTangentialVelocity() const;
    
    /**
     * Get the smooth friction coefficient based on the relative velocity.
     * 
     * @param relTangentialVelocity Magnitude of the relative tangential velocity
     * @return The interpolated friction coefficient
     */
    float getSmoothFrictionCoefficient(float relTangentialVelocity) const;
    
    /**
     * Calculate the friction mollifier function f0(x).
     * 
     * @param x Input value (typically velocity magnitude)
     * @return Mollifier value
     */
    float frictionMollifierF0(float x) const;
    
    /**
     * Calculate the derivative of the friction mollifier f1(x).
     * 
     * @param x Input value (typically velocity magnitude)
     * @return Derivative value
     */
    float frictionMollifierF1(float x) const;
    
    /**
     * Integrated friction-coefficient mollifier function.
     * Used for energy-consistent friction simulation.
     * 
     * @param x Input value (typically velocity magnitude)
     * @return Integrated friction-coefficient value
     */
    float frictionCoefficientIntegral(float x) const;
    
    /**
     * Update the friction coefficients based on the material properties of the bodies.
     * 
     * @param materialName0 Material name of first body
     * @param materialName1 Material name of second body
     */
    void updateFrictionCoefficients(const std::string& materialName0, const std::string& materialName1);

public:
    Eigen::Vector3f p;          // The contact point in world space
    Eigen::Vector3f n;          // The contact normal (body0 -> body1)
    Eigen::Vector3f t, b;       // Tangent directions for friction
    float pene;                 // Penetration depth
    
    // Friction properties for this specific contact
    float staticFriction;       // Static friction coefficient (μ_s)
    float kineticFriction;      // Kinetic friction coefficient (μ_k)

protected:
    // Default constructor (hidden)
    explicit Contact();
    
    /**
     * Register the contact with the two bodies.
     */
    void registerWithBodies();
};

/**
 * Material pair data structure for friction lookup.
 */
class MaterialPair {
public:
    std::string material1;
    std::string material2;
    float staticFriction;
    float kineticFriction;
    
    MaterialPair(const std::string& mat1, const std::string& mat2, float static_mu, float kinetic_mu)
        : material1(mat1), material2(mat2), staticFriction(static_mu), kineticFriction(kinetic_mu) {}
    
    // Hash function for material pairs
    struct Hash {
        size_t operator()(const MaterialPair& pair) const {
            return std::hash<std::string>()(pair.material1 + "-" + pair.material2);
        }
    };
    
    // Equality operator for material pairs
    struct Equal {
        bool operator()(const MaterialPair& lhs, const MaterialPair& rhs) const {
            return (lhs.material1 == rhs.material1 && lhs.material2 == rhs.material2) ||
                   (lhs.material1 == rhs.material2 && lhs.material2 == rhs.material1);
        }
    };
};

/**
 * Global registry of material pair friction coefficients.
 */
class FrictionRegistry {
public:
    static FrictionRegistry& getInstance() {
        static FrictionRegistry instance;
        return instance;
    }
    
    /**
     * Register friction coefficients for a pair of materials.
     */
    void registerMaterialPair(const std::string& mat1, const std::string& mat2, float static_mu, float kinetic_mu) {
        MaterialPair pair(mat1, mat2, static_mu, kinetic_mu);
        materialPairs[pair] = pair;
    }
    
    /**
     * Get friction coefficients for a pair of materials.
     * 
     * @param mat1 First material name
     * @param mat2 Second material name
     * @param staticMu Output static friction coefficient
     * @param kineticMu Output kinetic friction coefficient
     * @return True if material pair was found, false otherwise
     */
    bool getFrictionCoefficients(const std::string& mat1, const std::string& mat2, 
                                float& staticMu, float& kineticMu) const {
        MaterialPair key(mat1, mat2, 0, 0);
        auto it = materialPairs.find(key);
        if (it != materialPairs.end()) {
            staticMu = it->second.staticFriction;
            kineticMu = it->second.kineticFriction;
            return true;
        }
        return false;
    }

private:
    FrictionRegistry() {}  // Private constructor for singleton
    std::unordered_map<MaterialPair, MaterialPair, MaterialPair::Hash, MaterialPair::Equal> materialPairs;
};