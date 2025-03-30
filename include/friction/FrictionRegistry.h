#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>

// Forward declarations
class FrictionStrategy;
class Contact;

/**
 * Registry for all available friction models.
 * 
 * This class implements the Registry pattern to maintain a collection of all available
 * friction strategies. Strategies can be registered by name and then instantiated on demand.
 */
class FrictionRegistry {
public:
    // Type definition for friction strategy factory functions
    using StrategyFactoryFunc = std::function<std::unique_ptr<FrictionStrategy>()>;
    
    /**
     * Get the singleton instance of the registry.
     * 
     * @return Reference to the singleton registry
     */
    static FrictionRegistry& getInstance() {
        static FrictionRegistry instance;
        return instance;
    }
    
    /**
     * Register a new friction strategy.
     * 
     * @param name Name of the strategy
     * @param factoryFunc Factory function to create instances of this strategy
     * @return True if registration was successful, false if the name was already registered
     */
    bool registerStrategy(const std::string& name, StrategyFactoryFunc factoryFunc);
    
    /**
     * Check if a strategy is registered.
     * 
     * @param name Name of the strategy
     * @return True if the strategy is registered
     */
    bool isStrategyRegistered(const std::string& name) const;
    
    /**
     * Create a new instance of a registered strategy.
     * 
     * @param name Name of the strategy to create
     * @return Unique pointer to the created strategy, or nullptr if not registered
     */
    std::unique_ptr<FrictionStrategy> createStrategy(const std::string& name) const;
    
    /**
     * Get all registered strategy names.
     * 
     * @return Vector of registered strategy names
     */
    std::vector<std::string> getRegisteredStrategyNames() const;
    
    /**
     * Initialize the registry with standard friction strategies.
     */
    void initializeStandardStrategies();
    
private:
    // Private constructor for singleton
    FrictionRegistry() = default;
    
    // Disable copy/move
    FrictionRegistry(const FrictionRegistry&) = delete;
    FrictionRegistry& operator=(const FrictionRegistry&) = delete;
    FrictionRegistry(FrictionRegistry&&) = delete;
    FrictionRegistry& operator=(FrictionRegistry&&) = delete;
    
    // Map of strategy names to factory functions
    std::unordered_map<std::string, StrategyFactoryFunc> m_strategyFactories;
};