#pragma once

#include "log/Logger.h"

namespace slrbs {

class CSVLogger : public Logger
{
public:
    CSVLogger();
    virtual ~CSVLogger();

    // Configure CSV output format
    void setDelimiter(char delimiter);
    void setIncludeHeaders(bool include);
    void setQuoteStrings(bool quote);

    // Save log to CSV file
    virtual void save(const std::string& _filename) override;

protected:
    // Helper methods for CSV formatting
    std::string anyToString(const std::any& value);
    std::string formatForCSV(const std::string& value);

private:
    char m_delimiter;
    bool m_includeHeaders;
    bool m_quoteStrings;
};

}  // namespace slrbs
