#ifndef FRICTION_MODEL_H
#define FRICTION_MODEL_H

namespace friction {
    // Compute a mollifier function for static friction transition.
    float mollifierF0(float x);
    // Compute a mollifier function for kinetic friction transition.
    float mollifierF1(float x);
    // Compute the integral of the friction coefficient transition.
    float coefficientIntegral(float x);
    // Compute a smooth friction coefficient given the relative tangential velocity.
    // staticFriction: static friction coefficient.
    // kineticFriction: kinetic friction coefficient.
    // threshold: velocity threshold for transition.
    float getSmoothFrictionCoefficient(float relTangentialVelocity,
                                       float staticFriction,
                                       float kineticFriction,
                                       float threshold);
}

#endif // FRICTION_MODEL_H
