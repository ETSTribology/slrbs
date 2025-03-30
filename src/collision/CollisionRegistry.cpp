#include "collision/CollisionRegistry.h"
#include "collision/CollisionFactory.h"
#include "geometry/Geometry.h"

CollisionRegistry::CollisionRegistry() 
    : m_handlers()
{
    // Initialize all handlers to nullptr
    for (auto& handler : m_handlers) {
        handler = nullptr;
    }
}

CollisionRegistry& CollisionRegistry::getInstance() {
    static CollisionRegistry instance;
    return instance;
}

std::unique_ptr<CollisionHandler> CollisionRegistry::getHandler(GeometryType type1, GeometryType type2) const {
    // Validate types
    if (!GeometryTypeUtils::isValid(type1) || !GeometryTypeUtils::isValid(type2)) {
        return nullptr;
    }
    
    // Get the key and check if we have a handler
    size_t key = createKey(type1, type2);
    if (key < kMaxHandlers && m_handlers[key]) {
        return m_handlers[key]();
    }
    
    return nullptr;
}

void CollisionRegistry::initialize() {
    // Register standard handlers using the factory
    CollisionFactory::registerStandardHandlers(*this);
}