#include "log/Logger.h"
#include <iostream>

namespace slrbs {

Logger::Logger() : m_buf() {
}

Logger::~Logger() {
}

void Logger::setLevel(LogLevel level) {
    m_level = level;
}

LogLevel Logger::getLevel() const {
    return m_level;
}

int Logger::addField(const std::string& _name) {
    const auto it = std::find(m_fieldNames.begin(), m_fieldNames.end(), _name);
    
    assert(it == m_fieldNames.end());
    
    const int idx = m_fieldNames.size();
    m_fieldNames.push_back(_name);
    m_buf.push_back(std::vector<std::any>());
    m_buf.back().reserve(256 * 1024);
    return idx;
}

void Logger::clear() {
    for (auto& buf : m_buf) {
        buf.clear();
    }
    m_records.clear();
}

void Logger::log(LogLevel level, const std::string& message) {
    if (!shouldLog(level)) return;
    
    LogRecord record;
    record.timestamp = std::chrono::system_clock::now();
    record.level = level;
    record.message = message;
    
    m_records.push_back(std::move(record));
}

void Logger::trace(const std::string& message) {
    log(LogLevel::Trace, message);
}

void Logger::debug(const std::string& message) {
    log(LogLevel::Debug, message);
}

void Logger::info(const std::string& message) {
    log(LogLevel::Info, message);
}

void Logger::warning(const std::string& message) {
    log(LogLevel::Warning, message);
}

void Logger::error(const std::string& message) {
    log(LogLevel::Error, message);
}

void Logger::critical(const std::string& message) {
    log(LogLevel::Critical, message);
}

std::string Logger::getTimestampStr() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    // Get milliseconds
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
        
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    
    return ss.str();
}

bool Logger::shouldLog(LogLevel level) const {
    return level >= m_level;
}

} // namespace slrbs
