#pragma once

#include "log/Logger.h"
#include "log/CSVLogger.h"
#include "log/JSONLogger.h"
#include "log/BinaryLogger.h"
#include <memory>
#include <string>

namespace slrbs {

// Logger format types
enum class LoggerFormat {
    CSV,
    JSON,
    Binary
};

// Factory for creating different types of loggers
class LoggerFactory {
public:
    // Create a logger with specified format
    static std::shared_ptr<Logger> createLogger(LoggerFormat format);
    
    // Create a logger based on file extension
    static std::shared_ptr<Logger> createLoggerFromExtension(const std::string& filename);
};

} // namespace slrbs
