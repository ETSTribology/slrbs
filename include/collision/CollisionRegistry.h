#pragma once

#include <memory>
#include <array>
#include <unordered_map>
#include <typeindex>
#include <utility>
#include <functional>
#include "collision/CollisionHandler.h"

// Forward declaration
enum GeometryType;

/**
 * A registry for collision handlers.
 * 
 * This class uses templates and type traits to efficiently store and retrieve
 * the appropriate collision handler for a given pair of geometry types.
 */
class CollisionRegistry {
public:
    /**
     * Get the singleton instance of the registry.
     */
    static CollisionRegistry& getInstance();

    /**
     * Register a collision handler for a specific pair of geometry types.
     * 
     * @tparam T The type of the collision handler
     * @param type1 First geometry type
     * @param type2 Second geometry type
     */
    /**
     * Register a collision handler for a specific pair of geometry types.
     * 
     * @tparam T The type of the collision handler
     * @param type1 First geometry type
     * @param type2 Second geometry type
     */
    template<typename T>
    void registerHandler(GeometryType type1, GeometryType type2) {
        auto key = createKey(type1, type2);
        m_handlers[key] = []() { return std::make_unique<T>(); };
    }
    
    /**
     * Register a templated collision handler for a specific pair of geometry types.
     * 
     * @tparam Geom1 First geometry type
     * @tparam Geom2 Second geometry type
     */
    template<typename Geom1, typename Geom2>
    void registerTemplatedHandler() {
        auto type1 = getGeometryType<Geom1>();
        auto type2 = getGeometryType<Geom2>();
        auto key = createKey(type1, type2);
        m_handlers[key] = []() { return std::make_unique<CollisionHandlerFor<Geom1, Geom2>>(); };
    }
    
    /**
     * Helper template to get the GeometryType for a C++ type.
     */
    template<typename GeomType>
    static GeometryType getGeometryType();


    /**
     * Get a collision handler for a pair of geometry types.
     * 
     * @param type1 First geometry type
     * @param type2 Second geometry type
     * @return A unique pointer to the collision handler, or nullptr if not found
     */
    std::unique_ptr<CollisionHandler> getHandler(GeometryType type1, GeometryType type2);

    /**
     * Initialize the registry with default collision handlers.
     */
    void initialize();

private:
    CollisionRegistry() = default;
    
    // Disallow copying
    CollisionRegistry(const CollisionRegistry&) = delete;
    CollisionRegistry& operator=(const CollisionRegistry&) = delete;

    // Creates a unique key for a pair of geometry types
    size_t createKey(GeometryType type1, GeometryType type2) const;
    
    // Function type for creating collision handlers
    using HandlerCreator = std::function<std::unique_ptr<CollisionHandler>()>;
    
    // Map from geometry type pairs to handler creators
    std::unordered_map<size_t, HandlerCreator> m_handlers;
};

/**
 * Template for creating specialized collision handlers.
 * 
 * @tparam Geom1Type First geometry type
 * @tparam Geom2Type Second geometry type
 */
/**
 * Template for creating specialized collision handlers.
 * 
 * @tparam Geom1Type First geometry type
 * @tparam Geom2Type Second geometry type
 */
template<typename Geom1Type, typename Geom2Type>
class CollisionHandlerFor : public CollisionHandler {
public:
    void detectCollision(RigidBody* body0, RigidBody* body1, CollisionDetect* detector) override {
        // Cast the geometry objects to their specific types
        Geom1Type* geom1 = dynamic_cast<Geom1Type*>(body0->geometry.get());
        Geom2Type* geom2 = dynamic_cast<Geom2Type*>(body1->geometry.get());
        
        if (!geom1 || !geom2) {
            // Types don't match, try reverse order
            geom2 = dynamic_cast<Geom2Type*>(body0->geometry.get());
            geom1 = dynamic_cast<Geom1Type*>(body1->geometry.get());
            
            if (!geom1 || !geom2) {
                // Both attempts failed, wrong geometry types
                return;
            }
            
            // Types match in reverse order, call specialized detection with swapped bodies
            detectCollisionImpl(body1, body0, geom1, geom2, detector);
        } else {
            // Types match in original order
            detectCollisionImpl(body0, body1, geom1, geom2, detector);
        }
    }

private:
    // Specialized collision detection implementation
    void detectCollisionImpl(RigidBody* body1, RigidBody* body2, 
                            Geom1Type* geom1, Geom2Type* geom2,
                            CollisionDetect* detector);
};

// Template specializations for each type combination would be implemented elsewhere