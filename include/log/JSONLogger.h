#pragma once

#include "log/Logger.h"
#include <fstream>

namespace slrbs {

class JSONLogger : public Logger {
public:
    JSONLogger();
    virtual ~JSONLogger();

    // Set JSON formatting options
    void setPrettyPrint(bool prettyPrint);
    
    // Save logs to JSON file
    virtual void save(const std::string& _filename) override;

private:
    // Helper to convert any value to JSON representation
    std::string anyToJSON(const std::any& value);
    
    bool m_prettyPrint;
};

} // namespace slrbs
