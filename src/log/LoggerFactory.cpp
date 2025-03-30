#include "log/LoggerFactory.h"
#include <algorithm>
#include <filesystem>

namespace slrbs {

std::shared_ptr<Logger> LoggerFactory::createLogger(LoggerFormat format) {
    switch (format) {
        case LoggerFormat::CSV:
            return std::make_shared<CSVLogger>();
        case LoggerFormat::JSON:
            return std::make_shared<JSONLogger>();
        case LoggerFormat::Binary:
            return std::make_shared<BinaryLogger>();
        default:
            // Default to CSV
            return std::make_shared<CSVLogger>();
    }
}

std::shared_ptr<Logger> LoggerFactory::createLoggerFromExtension(const std::string& filename) {
    std::string extension = std::filesystem::path(filename).extension().string();
    
    // Convert to lowercase
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    
    if (extension == ".csv") {
        return std::make_shared<CSVLogger>();
    } else if (extension == ".json") {
        return std::make_shared<JSONLogger>();
    } else if (extension == ".bin" || extension == ".dat") {
        return std::make_shared<BinaryLogger>();
    } else {
        // Default to CSV for unknown extensions
        return std::make_shared<CSVLogger>();
    }
}

} // namespace slrbs
