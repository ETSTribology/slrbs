#include "log/CSVLogger.h"
#include <fstream>
#include <typeinfo>
#include <iostream>

namespace slrbs {

CSVLogger::CSVLogger() 
    : Logger(), m_delimiter(','), m_includeHeaders(true), m_quoteStrings(true) {
}

CSVLogger::~CSVLogger() {
}

void CSVLogger::setDelimiter(char delimiter) {
    m_delimiter = delimiter;
}

void CSVLogger::setIncludeHeaders(bool include) {
    m_includeHeaders = include;
}

void CSVLogger::setQuoteStrings(bool quote) {
    m_quoteStrings = quote;
}

void CSVLogger::save(const std::string& _filename) {
    std::ofstream fs(_filename);
    const int nrows = m_buf[0].size();
    const int ncols = m_fieldNames.size();

    for (int j = 0; j < ncols; ++j)
    {
        fs << m_fieldNames[j];
        if (j < ncols-1) fs << ",";
    }
    fs << std::endl;

    for (int i = 0; i < nrows; ++i)
    {
        for (int j = 0; j < ncols; ++j)
        {
            if (i < m_buf[j].size())
            {
                // TODO need to fix this.  Assumes all types are float. 
                fs << std::any_cast<float>(m_buf[j][i]);
            }

            if (j < ncols-1) fs << ",";
        }
        fs << std::endl;
    }

    fs.flush();
    fs.close();
}

std::string CSVLogger::anyToString(const std::any& value) {
    try {
        if (value.type() == typeid(int)) {
            return std::to_string(std::any_cast<int>(value));
        } else if (value.type() == typeid(float)) {
            return std::to_string(std::any_cast<float>(value));
        } else if (value.type() == typeid(double)) {
            return std::to_string(std::any_cast<double>(value));
        } else if (value.type() == typeid(bool)) {
            return std::any_cast<bool>(value) ? "true" : "false";
        } else if (value.type() == typeid(std::string)) {
            return std::any_cast<std::string>(value);
        } else {
            return "unsupported_type";
        }
    } catch (const std::bad_any_cast&) {
        return "error_casting";
    }
}

std::string CSVLogger::formatForCSV(const std::string& value) {
    // Check if we need to quote this string
    bool needsQuoting = m_quoteStrings && 
                        (value.find(m_delimiter) != std::string::npos || 
                         value.find('\"') != std::string::npos || 
                         value.find('\n') != std::string::npos);
    
    if (!needsQuoting) {
        return value;
    }
    
    // Escape quotes by doubling them and enclose in quotes
    std::string result = "\"";
    for (char c : value) {
        if (c == '\"') result += "\"\""; // Double quotes to escape them
        else result += c;
    }
    result += "\"";
    
    return result;
}

} // namespace slrbs
