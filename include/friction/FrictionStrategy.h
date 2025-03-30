#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include "contact/Contact.h"
#include "friction/FrictionRegistry.h"

/**
 * Base strategy for implementing different friction models.
 * 
 * This class defines the interface for all friction strategies
 * and enables various friction models to be used interchangeably.
 */
class FrictionStrategy {
public:
    virtual ~FrictionStrategy() = default;
    
    /**
     * Get the name of this friction strategy.
     * 
     * @return Strategy name
     */
    virtual std::string getName() const = 0;
    
    /**
     * Apply friction constraints to the given lambda vector.
     * 
     * @param lambda Contact force/impulse vector to apply constraints to
     * @param A Contact constraint matrix
     * @param b RHS vector
     * @param contact The contact information
     * @param velThreshold Velocity threshold for smooth friction models
     */
    virtual void applyFrictionConstraints(
        Eigen::Vector3f& lambda,
        const Eigen::Matrix3f& A,
        const Eigen::Vector3f& b,
        const Contact* contact,
        float velThreshold = 0.1f) const = 0;
        
    /**
     * Compute contact bounds for the friction model.
     *
     * @param contact The contact to compute bounds for
     * @param lower Lower bounds vector to update
     * @param upper Upper bounds vector to update
     * @param index Index in the bounds vectors for this contact
     */
    virtual void computeBounds(
        const Contact* contact,
        Eigen::VectorXf& lower,
        Eigen::VectorXf& upper,
        int index) const = 0;
        
    /**
     * Create a clone of this strategy.
     * 
     * @return Unique pointer to a new instance of this strategy
     */
    virtual std::unique_ptr<FrictionStrategy> clone() const = 0;
};