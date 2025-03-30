#pragma once

#include "log/Logger.h"
#include "utils/Serializer.h"

namespace slrbs {

class BinaryLogger : public Logger {
public:
    BinaryLogger();
    virtual ~BinaryLogger();
    
    // Save logs to binary file
    virtual void save(const std::string& _filename) override;

private:
    // Helper to write any value to binary format
    void writeAnyToBinary(BinarySerializer& serializer, const std::any& value);
};

} // namespace slrbs
