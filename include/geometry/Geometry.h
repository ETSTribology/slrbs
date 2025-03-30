#pragma once

#include "util/Types.h"
#include <Eigen/Dense>
#include <memory>

/**
 * Enumeration of supported geometry types
 */
enum GeometryType {
    kSphere = 0,
    kBox,
    kPlane,
    kCylinder,
    kSDF      // Signed Distance Field - for complex meshes
    kGeometryTypeCount // Total number of geometry types
};

/**
 * Base class for all geometry types in the physics system.
 * Provides common interface for physical properties.
 */
class Geometry {
public:
    /**
     * Default constructor.
     */
    Geometry() : m_I() {}
    
    /**
     * Virtual destructor to ensure proper cleanup of derived classes.
     */
    virtual ~Geometry() = default;
    
    /**
     * Computes the inertia tensor for the geometry given a mass.
     * @param mass The mass of the object.
     * @return The 3x3 inertia tensor.
     */
    virtual Eigen::Matrix3f computeInertia(float mass) = 0;
    
    /**
     * Returns the type of geometry.
     * @return The geometry type identifier.
     */
    virtual GeometryType getType() const = 0;
    
    /**
     * Factory method to create new geometries of specified types.
     * Provides a consistent way to create geometry objects.
     * @tparam GeomType The specific geometry class type
     * @tparam Args Types of arguments to pass to the constructor
     * @param args Arguments to pass to the constructor
     * @return A unique_ptr to the newly created geometry
     */
    template<typename GeomType, typename... Args>
    static std::unique_ptr<GeomType> create(Args&&... args) {
        return std::make_unique<GeomType>(std::forward<Args>(args)...);
    }

protected:
    Eigen::Matrix3f m_I;  // Inertia tensor (cached for internal use)
};

/**
 * Sphere geometry represented by a radius.
 */
class Sphere : public Geometry {
public:
    float radius;  // Sphere radius
    
    /**
     * Constructor.
     * @param radius The radius of the sphere.
     */
    explicit Sphere(float radius) : radius(radius) {}
    
    /**
     * Computes the inertia tensor for a sphere.
     * For a solid sphere: I = 2/5 * m * r^2 for each diagonal element.
     */
    Eigen::Matrix3f computeInertia(float mass) override {
        m_I.setZero();
        const float inertiaValue = (2.0f/5.0f) * mass * radius * radius;
        m_I(0,0) = m_I(1,1) = m_I(2,2) = inertiaValue;
        return m_I;
    }
    
    /**
     * Returns the type of geometry.
     */
    GeometryType getType() const override { return kSphere; }
};

/**
 * Box geometry represented by its dimensions.
 */
class Box : public Geometry {
public:
    Eigen::Vector3f dim;  // Box dimensions (width, height, depth)
    
    /**
     * Constructor.
     * @param dimensions The dimensions of the box (x, y, z).
     */
    explicit Box(const Eigen::Vector3f& dimensions) : dim(dimensions) {}
    
    /**
     * Computes the inertia tensor for a box.
     * For a solid box with uniform density, the inertia tensor has diagonal elements:
     * I_xx = m/12 * (y^2 + z^2)
     * I_yy = m/12 * (x^2 + z^2)
     * I_zz = m/12 * (x^2 + y^2)
     */
    Eigen::Matrix3f computeInertia(float mass) override {
        m_I.setZero();
        m_I(0,0) = (1.0f/12.0f) * mass * (dim.y()*dim.y() + dim.z()*dim.z());
        m_I(1,1) = (1.0f/12.0f) * mass * (dim.x()*dim.x() + dim.z()*dim.z());
        m_I(2,2) = (1.0f/12.0f) * mass * (dim.x()*dim.x() + dim.y()*dim.y());
        return m_I;
    }
    
    /**
     * Returns the type of geometry.
     */
    GeometryType getType() const override { return kBox; }
};

/**
 * Cylinder geometry represented by height and radius.
 */
class Cylinder : public Geometry {
public:
    float height;  // Cylinder height
    float radius;  // Cylinder radius
    
    /**
     * Constructor.
     * @param height The height of the cylinder.
     * @param radius The radius of the cylinder.
     */
    Cylinder(float height, float radius) : height(height), radius(radius) {}
    
    /**
     * Computes the inertia tensor for a cylinder.
     * For a solid cylinder with the y-axis as the principal axis:
     * I_xx = I_zz = m/12 * (3r^2 + h^2)
     * I_yy = m/2 * r^2
     */
    Eigen::Matrix3f computeInertia(float mass) override {
        static const float s = 1.0f / 12.0f;
        const float h2 = height * height;
        const float r2 = radius * radius;
        const float r2_h2_3 = (3.0f * r2 + h2);
        
        m_I.setZero();
        m_I(0, 0) = s * mass * r2_h2_3;
        m_I(1, 1) = 0.5f * mass * r2;
        m_I(2, 2) = s * mass * r2_h2_3;
        return m_I;
    }
    
    /**
     * Returns the type of geometry.
     */
    GeometryType getType() const override { return kCylinder; }
};

/**
 * Plane geometry represented by a point and normal.
 * Used primarily for static collision objects.
 */
class Plane : public Geometry {
public:
    Eigen::Vector3f p;  // Point on the plane
    Eigen::Vector3f n;  // Normal of the plane
    
    /**
     * Constructor.
     * @param point A point on the plane.
     * @param normal The normal vector of the plane.
     */
    Plane(const Eigen::Vector3f& point, const Eigen::Vector3f& normal) 
        : p(point), n(normal.normalized()) {}
    
    /**
     * Computes the inertia tensor for a plane.
     * Note: Planes are typically used for static objects, so this is a simplified
     * implementation that returns a unit inertia scaled by mass.
     */
    Eigen::Matrix3f computeInertia(float mass) override {
        m_I.setIdentity();  // Identity for static objects
        return mass * m_I;
    }
    
    /**
     * Returns the type of geometry.
     */
    GeometryType getType() const override { return kPlane; }
};