#pragma once

#include <Eigen/Dense>
#include <vector>

// Forward declarations
class Joint;
class Contact;
class RigidBody;

/**
 * Utility functions shared across different solver implementations.
 * These functions handle common operations like building system matrices,
 * computing right-hand sides, and updating velocities.
 */
namespace SolverUtils {

/**
 * Apply linear matrix-vector operations for the Jacobian.
 * 
 * @param G Jacobian matrix
 * @param x Linear velocity or force vector
 * @param a Scaling factor
 * @param b Result vector to modify
 */
void applyLinearMatrixVectorOperations(const Eigen::MatrixXf& G, 
                                      const Eigen::Vector3f& x, 
                                      float a, 
                                      Eigen::Ref<Eigen::VectorXf> b);

/**
 * Apply angular matrix-vector operations for the Jacobian.
 * 
 * @param G Jacobian matrix
 * @param y Angular velocity or torque vector
 * @param a Scaling factor
 * @param b Result vector to modify
 */
void applyAngularMatrixVectorOperations(const Eigen::MatrixXf& G, 
                                       const Eigen::Vector3f& y, 
                                       float a, 
                                       Eigen::Ref<Eigen::VectorXf> b);

/**
 * Build the right-hand side vector for joint constraints.
 * 
 * @param joint Joint to build RHS for
 * @param h Time step
 * @param b Output RHS vector
 * @param gamma Stabilization factor
 */
void buildJointRHS(Joint* joint, float h, Eigen::VectorXf& b, float gamma);

/**
 * Build the right-hand side vector for contact constraints.
 * 
 * @param contact Contact to build RHS for
 * @param h Time step
 * @param b Output RHS vector
 * @param gamma Stabilization factor
 */
void buildContactRHS(Contact* contact, float h, Eigen::Vector3f& b, float gamma);

/**
 * Build diagonal matrices for joints.
 * 
 * @param joints Vector of joints
 * @param A Output vector of matrices
 * @param eps Regularization parameter
 */
void buildJointDiagonalMatrices(const std::vector<Joint*>& joints, 
                               std::vector<Eigen::MatrixXf>& A,
                               float eps = 1e-5f);

/**
 * Build diagonal matrices for contacts.
 * 
 * @param contacts Vector of contacts
 * @param A Output vector of matrices
 * @param eps Regularization parameter
 */
void buildContactDiagonalMatrices(const std::vector<Contact*>& contacts, 
                                 std::vector<Eigen::Matrix3f>& A,
                                 float eps = 1e-5f);

/**
 * Solve a contact constraint using box constraints.
 * 
 * @param A Contact constraint matrix
 * @param b RHS vector
 * @param x Current solution (will be updated)
 * @param mu Friction coefficient
 * @param staticMu Static friction coefficient (for smooth friction model)
 * @param kineticMu Kinetic friction coefficient (for smooth friction model)
 * @param useSmoothFriction Whether to use smooth friction transition
 * @param relVel Relative tangential velocity (for smooth friction model)
 * @param velThreshold Velocity threshold for friction transition
 */
void solveBoxConstraintContact(const Eigen::Matrix3f& A, 
                              const Eigen::Vector3f& b, 
                              Eigen::Vector3f& x,
                              float mu,
                              float staticMu = 0.0f,
                              float kineticMu = 0.0f,
                              bool useSmoothFriction = false,
                              float relVel = 0.0f,
                              float velThreshold = 0.1f);

/**
 * Solve a joint constraint using LDLT decomposition.
 * 
 * @param A Joint constraint matrix
 * @param b RHS vector
 * @param x Output solution vector
 */
void solveJoint(const Eigen::MatrixXf& A, 
               const Eigen::VectorXf& b, 
               Eigen::VectorXf& x);

/**
 * Compute a smooth friction coefficient between static and kinetic.
 * 
 * @param relVel Relative tangential velocity
 * @param staticMu Static friction coefficient
 * @param kineticMu Kinetic friction coefficient
 * @param velThreshold Velocity threshold for transition
 * @return Interpolated friction coefficient
 */
float computeSmoothFrictionCoefficient(float relVel,
                                      float staticMu,
                                      float kineticMu,
                                      float velThreshold);

/**
 * Initialize the bounds for box constraints.
 * 
 * @param dim Total dimension of the problem
 * @param lower Output lower bounds vector
 * @param upper Output upper bounds vector
 */
void initializeBounds(int dim, 
                     Eigen::VectorXf& lower, 
                     Eigen::VectorXf& upper);

/**
 * Update box bounds for contacts with friction.
 * 
 * @param contacts Vector of contacts
 * @param lower Lower bounds vector
 * @param upper Upper bounds vector
 * @param useSmoothFriction Whether to use smooth friction transition
 * @param velThreshold Velocity threshold for friction transition
 */
void updateContactBounds(const std::vector<Contact*>& contacts, 
                        Eigen::VectorXf& lower, 
                        Eigen::VectorXf& upper,
                        bool useSmoothFriction = false,
                        float velThreshold = 0.1f);

/**
 * Build the full system matrix for the Schur complement.
 * 
 * @param joints Vector of joints
 * @param contacts Vector of contacts
 * @param h Time step
 * @param A Output system matrix
 */
void buildSchurComplementMatrix(const std::vector<Joint*>& joints,
                               const std::vector<Contact*>& contacts,
                               float h,
                               Eigen::MatrixXf& A);

/**
 * Build the right-hand side vector for the Schur complement.
 * 
 * @param joints Vector of joints
 * @param contacts Vector of contacts
 * @param h Time step
 * @param b Output RHS vector
 * @param gamma Stabilization factor
 */
void buildSchurComplementRHS(const std::vector<Joint*>& joints,
                            const std::vector<Contact*>& contacts,
                            float h,
                            Eigen::VectorXf& b,
                            float gamma);

/**
 * Update joints and contacts with the computed impulses.
 * 
 * @param x Solution vector
 * @param joints Vector of joints
 * @param contacts Vector of contacts
 */
void updateJointsAndContacts(const Eigen::VectorXf& x,
                            std::vector<Joint*>& joints,
                            std::vector<Contact*>& contacts);

} // namespace SolverUtils