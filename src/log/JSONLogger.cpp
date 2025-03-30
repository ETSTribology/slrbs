#include "log/JSONLogger.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

namespace slrbs {

JSONLogger::JSONLogger() : Logger(), m_prettyPrint(true) {
}

JSONLogger::~JSONLogger() {
}

void JSONLogger::setPrettyPrint(bool prettyPrint) {
    m_prettyPrint = prettyPrint;
}

void JSONLogger::save(const std::string& _filename) {
    nlohmann::json root;
    
    // Add metadata
    root["metadata"] = {
        {"version", "1.0"},
        {"format", "json"},
        {"timestamp", getTimestampStr()}
    };
    
    // Add fields data
    nlohmann::json fields = nlohmann::json::array();
    for (size_t i = 0; i < m_buf[0].size(); ++i) {
        nlohmann::json record;
        for (size_t j = 0; j < m_buf.size(); ++j) {
            if (i < m_buf[j].size()) {
                // Try to convert the any value to appropriate JSON type
                try {
                    const std::any& val = m_buf[j][i];
                    if (val.type() == typeid(int)) {
                        record[m_fieldNames[j]] = std::any_cast<int>(val);
                    } else if (val.type() == typeid(float)) {
                        record[m_fieldNames[j]] = std::any_cast<float>(val);
                    } else if (val.type() == typeid(double)) {
                        record[m_fieldNames[j]] = std::any_cast<double>(val);
                    } else if (val.type() == typeid(bool)) {
                        record[m_fieldNames[j]] = std::any_cast<bool>(val);
                    } else if (val.type() == typeid(std::string)) {
                        record[m_fieldNames[j]] = std::any_cast<std::string>(val);
                    } else {
                        record[m_fieldNames[j]] = "unsupported_type";
                    }
                } catch (const std::bad_any_cast&) {
                    record[m_fieldNames[j]] = "error_casting";
                }
            }
        }
        fields.push_back(record);
    }
    root["fields"] = fields;
    
    // Add log records
    nlohmann::json logs = nlohmann::json::array();
    for (const auto& record : m_records) {
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
        
        // Convert level to string
        std::string levelStr;
        switch (record.level) {
            case LogLevel::Trace: levelStr = "TRACE"; break;
            case LogLevel::Debug: levelStr = "DEBUG"; break;
            case LogLevel::Info: levelStr = "INFO"; break;
            case LogLevel::Warning: levelStr = "WARNING"; break;
            case LogLevel::Error: levelStr = "ERROR"; break;
            case LogLevel::Critical: levelStr = "CRITICAL"; break;
            default: levelStr = "UNKNOWN";
        }
        
        nlohmann::json logEntry = {
            {"timestamp", timestampStr},
            {"level", levelStr},
            {"message", record.message}
        };
        
        logs.push_back(logEntry);
    }
    root["logs"] = logs;
    
    // Write to file
    std::ofstream file(_filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open log file: " << _filename << std::endl;
        return;
    }
    
    file << (m_prettyPrint ? root.dump(4) : root.dump());
    file.close();
}

} // namespace slrbs
