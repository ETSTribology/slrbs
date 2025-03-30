#include "log/BinaryLogger.h"
#include "utils/Serializer.h"
#include <iostream>

namespace slrbs {

BinaryLogger::BinaryLogger() : Logger() {
}

BinaryLogger::~BinaryLogger() {
}

void BinaryLogger::save(const std::string& _filename) {
    BinarySerializer serializer;
    if (!serializer.openForWrite(_filename)) {
        std::cerr << "Failed to open log file: " << _filename << std::endl;
        return;
    }
    
    // Write header information
    serializer.writeNamedString("Format", "SLRBS_LOG");
    serializer.writeNamedString("Version", "1.0");
    serializer.writeNamedString("Timestamp", getTimestampStr());
    
    // Write number of fields and field names
    serializer.writeNamedInt32("FieldCount", static_cast<int32_t>(m_fieldNames.size()));
    for (size_t i = 0; i < m_fieldNames.size(); ++i) {
        serializer.writeNamedString("FieldName_" + std::to_string(i), m_fieldNames[i]);
    }
    
    // Write field data
    for (size_t i = 0; i < m_buf.size(); ++i) {
        const auto& field = m_buf[i];
        serializer.writeNamedInt32("RecordCount_" + std::to_string(i), static_cast<int32_t>(field.size()));
        
        for (size_t j = 0; j < field.size(); ++j) {
            const auto& value = field[j];
            try {
                // Handle different data types
                if (value.type() == typeid(int)) {
                    serializer.writeNamedInt32("Field_" + std::to_string(i) + "_" + std::to_string(j), 
                                              std::any_cast<int>(value));
                } else if (value.type() == typeid(float)) {
                    serializer.writeNamedFloat("Field_" + std::to_string(i) + "_" + std::to_string(j), 
                                              std::any_cast<float>(value));
                } else if (value.type() == typeid(double)) {
                    serializer.writeNamedDouble("Field_" + std::to_string(i) + "_" + std::to_string(j), 
                                              std::any_cast<double>(value));
                } else if (value.type() == typeid(bool)) {
                    serializer.writeNamedBool("Field_" + std::to_string(i) + "_" + std::to_string(j), 
                                             std::any_cast<bool>(value));
                } else if (value.type() == typeid(std::string)) {
                    serializer.writeNamedString("Field_" + std::to_string(i) + "_" + std::to_string(j), 
                                               std::any_cast<std::string>(value));
                }
            } catch (const std::bad_any_cast&) {
                // Handle error
                serializer.writeNamedString("Field_" + std::to_string(i) + "_" + std::to_string(j) + "_Error", 
                                           "Type casting error");
            }
        }
    }
    
    // Write log records
    serializer.writeNamedInt32("LogCount", static_cast<int32_t>(m_records.size()));
    for (size_t i = 0; i < m_records.size(); ++i) {
        const auto& record = m_records[i];
        
        // Convert timestamp to string
        std::string timestampStr = [&]() {
            auto time_t = std::chrono::system_clock::to_time_t(record.timestamp);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                record.timestamp.time_since_epoch()) % 1000;
            
            std::stringstream ss;
            ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
            ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
            return ss.str();
        }();
        
        // Convert level to int
        int32_t level = static_cast<int32_t>(record.level);
        
        serializer.writeNamedString("Log_" + std::to_string(i) + "_Timestamp", timestampStr);
        serializer.writeNamedInt32("Log_" + std::to_string(i) + "_Level", level);
        serializer.writeNamedString("Log_" + std::to_string(i) + "_Message", record.message);
    }
    
    serializer.close();
}

} // namespace slrbs
