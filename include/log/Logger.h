#pragma once

#include <algorithm>
#include <any>
#include <cassert>
#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace slrbs {

// Log levels for message filtering
enum class LogLevel {
    Trace,
    Debug,
    Info,
    Warning,
    Error,
    Critical
};

// Log record for storing log entries with metadata
struct LogRecord {
    std::chrono::system_clock::time_point timestamp;
    LogLevel level;
    std::string message;
};

class Logger
{
public:
    Logger();
    virtual ~Logger();

    // Set/get the logging level
    void setLevel(LogLevel level);
    LogLevel getLevel() const;

    // Field management for structured data
    int addField(const std::string& _name);
    void clear();

    // Log messages with different severity levels
    void log(LogLevel level, const std::string& message);
    void trace(const std::string& message);
    void debug(const std::string& message);
    void info(const std::string& message);
    void warning(const std::string& message);
    void error(const std::string& message);
    void critical(const std::string& message);

    // Push values to fields
    template<typename T>
    void pushVal(int _idx, const T& _val)
    {
        assert(0 <= _idx && _idx < m_buf.size());
        m_buf[_idx].push_back(_val);
    }

    // Save log to file (to be implemented by derived classes)
    virtual void save(const std::string& _filename) { }

protected:
    // Helper methods for formatting
    std::string getTimestampStr();
    bool shouldLog(LogLevel level) const;

    // Storage for log data
    std::vector<std::vector<std::any>> m_buf;
    std::vector<std::string> m_fieldNames;
    std::vector<LogRecord> m_records;
    LogLevel m_level = LogLevel::Info;
};

}  // namespace slrbs
