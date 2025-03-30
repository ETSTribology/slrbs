#include "utils/Serializer.h"
#include <filesystem>
#include <stdexcept>
#include <chrono>
#include <ctime>

namespace slrbs {

BinarySerializer::BinarySerializer() 
    : m_isReading(false), m_fileExists(false), m_currentFrame(0) 
{
    // Initialize the file header
    memset(&m_fileHeader, 0, sizeof(FileHeader));
    memcpy(m_fileHeader.magic, "SLRB", 4);
    m_fileHeader.version = 1;
    
    // Set timestamp
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    m_fileHeader.timeStamp = static_cast<uint64_t>(now_c);
}

BinarySerializer::~BinarySerializer() {
    if (m_file.is_open()) {
        if (!m_isReading) {
            updateFileHeader();
        }
        m_file.close();
    }
}

bool BinarySerializer::openForWrite(const std::string& filename, bool append) {
    std::ios_base::openmode mode = std::ios::binary;
    if (append) {
        mode |= std::ios::in | std::ios::out | std::ios::app;
        // Check if file exists and is valid
        std::ifstream checkFile(filename, std::ios::binary | std::ios::ate);
        if (checkFile.good() && checkFile.tellg() >= sizeof(FileHeader)) {
            m_fileExists = true;
        }
    } else {
        mode |= std::ios::out;
    }
    
    m_file.open(filename, mode);
    m_isReading = false;
    
    if (!m_file.is_open()) {
        return false;
    }
    
    if (m_fileExists) {
        readFileHeader();
        m_file.seekp(0, std::ios::end);
    } else {
        writeFileHeader();
    }
    
    return true;
}

bool BinarySerializer::openForRead(const std::string& filename) {
    m_file.open(filename, std::ios::in | std::ios::binary);
    m_isReading = true;
    
    if (!m_file.is_open()) {
        return false;
    }
    
    // Read file header
    readFileHeader();
    return true;
}

void BinarySerializer::close() {
    if (m_file.is_open()) {
        if (!m_isReading) {
            updateFileHeader();
        }
        m_file.close();
    }
}

bool BinarySerializer::isOpen() const {
    return m_file.is_open();
}

void BinarySerializer::beginFrame(int frameId) {
    m_currentFrame = frameId;
    
    // Write frame marker
    RecordHeader header;
    header.type = DataType::FrameMarker;
    header.nameLength = 10; // "FrameBegin"
    header.dimensions[0] = 0;
    header.dimensions[1] = 0;
    header.dataSize = sizeof(int32_t);
    
    writeRecordHeader(header, "FrameBegin");
    m_file.write(reinterpret_cast<const char*>(&frameId), sizeof(int32_t));
    
    m_fileHeader.frameCount++;
}

void BinarySerializer::endFrame() {
    // Write frame end marker
    RecordHeader header;
    header.type = DataType::FrameMarker;
    header.nameLength = 8; // "FrameEnd"
    header.dimensions[0] = 0;
    header.dimensions[1] = 0;
    header.dataSize = sizeof(int32_t);
    
    writeRecordHeader(header, "FrameEnd");
    m_file.write(reinterpret_cast<const char*>(&m_currentFrame), sizeof(int32_t));
    
    updateFileHeader();
}

void BinarySerializer::writeFileHeader() {
    m_file.seekp(0);
    m_file.write(reinterpret_cast<const char*>(&m_fileHeader), sizeof(FileHeader));
}

void BinarySerializer::readFileHeader() {
    m_file.seekg(0);
    m_file.read(reinterpret_cast<char*>(&m_fileHeader), sizeof(FileHeader));
    
    // Validate header
    if (memcmp(m_fileHeader.magic, "SLRB", 4) != 0) {
        throw std::runtime_error("Invalid file format: magic number mismatch");
    }
}

void BinarySerializer::updateFileHeader() {
    auto currentPos = m_file.tellp();
    m_file.seekp(0);
    m_file.write(reinterpret_cast<const char*>(&m_fileHeader), sizeof(FileHeader));
    m_file.seekp(currentPos);
}

void BinarySerializer::writeRecordHeader(const RecordHeader& header, const std::string& name) {
    m_file.write(reinterpret_cast<const char*>(&header.type), sizeof(DataType));
    m_file.write(reinterpret_cast<const char*>(&header.nameLength), sizeof(uint32_t));
    m_file.write(name.data(), header.nameLength);
    m_file.write(reinterpret_cast<const char*>(&header.dimensions), sizeof(header.dimensions));
    m_file.write(reinterpret_cast<const char*>(&header.dataSize), sizeof(uint32_t));
}

void BinarySerializer::writeDataType(DataType type) {
    m_file.write(reinterpret_cast<char*>(&type), sizeof(DataType));
}

void BinarySerializer::writeInt8(int8_t value) {
    writeNamedInt8("", value);
}

void BinarySerializer::writeInt32(int32_t value) {
    writeNamedInt32("", value);
}

void BinarySerializer::writeFloat(float value) {
    writeNamedFloat("", value);
}

void BinarySerializer::writeDouble(double value) {
    writeNamedDouble("", value);
}

void BinarySerializer::writeString(const std::string& value) {
    writeNamedString("", value);
}

void BinarySerializer::writeBool(bool value) {
    writeNamedBool("", value);
}

void BinarySerializer::writeNamedInt8(const std::string& name, int8_t value) {
    RecordHeader header;
    header.type = DataType::Int8;
    header.nameLength = static_cast<uint32_t>(name.length());
    header.dataSize = sizeof(int8_t);
    
    writeRecordHeader(header, name);
    m_file.write(reinterpret_cast<const char*>(&value), sizeof(int8_t));
    
    m_fileHeader.recordCount++;
}

void BinarySerializer::writeNamedInt32(const std::string& name, int32_t value) {
    RecordHeader header;
    header.type = DataType::Int32;
    header.nameLength = static_cast<uint32_t>(name.length());
    header.dataSize = sizeof(int32_t);
    
    writeRecordHeader(header, name);
    m_file.write(reinterpret_cast<const char*>(&value), sizeof(int32_t));
    
    m_fileHeader.recordCount++;
}

void BinarySerializer::writeNamedFloat(const std::string& name, float value) {
    RecordHeader header;
    header.type = DataType::Float;
    header.nameLength = static_cast<uint32_t>(name.length());
    header.dataSize = sizeof(float);
    
    writeRecordHeader(header, name);
    m_file.write(reinterpret_cast<const char*>(&value), sizeof(float));
    
    m_fileHeader.recordCount++;
}

void BinarySerializer::writeNamedDouble(const std::string& name, double value) {
    RecordHeader header;
    header.type = DataType::Double;
    header.nameLength = static_cast<uint32_t>(name.length());
    header.dataSize = sizeof(double);
    
    writeRecordHeader(header, name);
    m_file.write(reinterpret_cast<const char*>(&value), sizeof(double));
    
    m_fileHeader.recordCount++;
}

void BinarySerializer::writeNamedString(const std::string& name, const std::string& value) {
    RecordHeader header;
    header.type = DataType::String;
    header.nameLength = static_cast<uint32_t>(name.length());
    header.dataSize = static_cast<uint32_t>(value.length());
    
    writeRecordHeader(header, name);
    m_file.write(value.data(), value.length());
    
    m_fileHeader.recordCount++;
}

void BinarySerializer::writeNamedBool(const std::string& name, bool value) {
    RecordHeader header;
    header.type = DataType::Bool;
    header.nameLength = static_cast<uint32_t>(name.length());
    header.dataSize = sizeof(bool);
    
    writeRecordHeader(header, name);
    m_file.write(reinterpret_cast<const char*>(&value), sizeof(bool));
    
    m_fileHeader.recordCount++;
}

void BinarySerializer::writeNamedVector3f(const std::string& name, const Eigen::Vector3f& vec) {
    RecordHeader header;
    header.type = DataType::Vector3f;
    header.nameLength = static_cast<uint32_t>(name.length());
    header.dimensions[0] = 3;
    header.dimensions[1] = 1;
    header.dataSize = 3 * sizeof(float);
    
    writeRecordHeader(header, name);
    for (int i = 0; i < 3; ++i) {
        float val = vec(i);
        m_file.write(reinterpret_cast<const char*>(&val), sizeof(float));
    }
    
    m_fileHeader.recordCount++;
}

void BinarySerializer::writeNamedMatrix3f(const std::string& name, const Eigen::Matrix3f& mat) {
    RecordHeader header;
    header.type = DataType::Matrix3f;
    header.nameLength = static_cast<uint32_t>(name.length());
    header.dimensions[0] = 3;
    header.dimensions[1] = 3;
    header.dataSize = 9 * sizeof(float);
    
    writeRecordHeader(header, name);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float val = mat(i, j);
            m_file.write(reinterpret_cast<const char*>(&val), sizeof(float));
        }
    }
    
    m_fileHeader.recordCount++;
}

// Legacy read operations (maintained for backward compatibility)
DataType BinarySerializer::readDataType() {
    uint8_t typeVal;
    m_file.read(reinterpret_cast<char*>(&typeVal), sizeof(typeVal));
    return static_cast<DataType>(typeVal);
}

int8_t BinarySerializer::readInt8() {
    DataType type = readDataType();
    if (type != DataType::Int8) {
        throw std::runtime_error("Type mismatch in readInt8, expected Int8");
    }
    int8_t value;
    m_file.read(reinterpret_cast<char*>(&value), sizeof(value));
    return value;
}

int32_t BinarySerializer::readInt32() {
    DataType type = readDataType();
    if (type != DataType::Int32) {
        throw std::runtime_error("Type mismatch in readInt32, expected Int32");
    }
    int32_t value;
    m_file.read(reinterpret_cast<char*>(&value), sizeof(value));
    return value;
}

float BinarySerializer::readFloat() {
    DataType type = readDataType();
    if (type != DataType::Float) {
        throw std::runtime_error("Type mismatch in readFloat, expected Float");
    }
    float value;
    m_file.read(reinterpret_cast<char*>(&value), sizeof(value));
    return value;
}

double BinarySerializer::readDouble() {
    DataType type = readDataType();
    if (type != DataType::Double) {
        throw std::runtime_error("Type mismatch in readDouble, expected Double");
    }
    double value;
    m_file.read(reinterpret_cast<char*>(&value), sizeof(value));
    return value;
}

std::string BinarySerializer::readString() {
    DataType type = readDataType();
    if (type != DataType::String) {
        throw std::runtime_error("Type mismatch in readString, expected String");
    }
    int32_t size = readInt32();
    std::string value(size, '\0');
    m_file.read(&value[0], size);
    return value;
}

Eigen::Vector3f BinarySerializer::readVector3f() {
    DataType type = readDataType();
    if (type != DataType::Vector3f) {
        throw std::runtime_error("Type mismatch in readVector3f, expected Vector3f");
    }
    Eigen::Vector3f vec;
    for (int i = 0; i < 3; ++i) {
        vec(i) = readFloat();
    }
    return vec;
}

Eigen::Matrix3f BinarySerializer::readMatrix3f() {
    DataType type = readDataType();
    if (type != DataType::Matrix3f) {
        throw std::runtime_error("Type mismatch in readMatrix3f, expected Matrix3f");
    }
    Eigen::Matrix3f mat;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat(i, j) = readFloat();
        }
    }
    return mat;
}

Eigen::VectorXf BinarySerializer::readVectorXf() {
    DataType type = readDataType();
    if (type != DataType::VectorXf) {
        throw std::runtime_error("Type mismatch in readVectorXf, expected VectorXf");
    }
    int32_t size = readInt32();
    Eigen::VectorXf vec(size);
    for (int i = 0; i < size; ++i) {
        vec(i) = readFloat();
    }
    return vec;
}

Eigen::MatrixXf BinarySerializer::readMatrixXf() {
    DataType type = readDataType();
    if (type != DataType::MatrixXf) {
        throw std::runtime_error("Type mismatch in readMatrixXf, expected MatrixXf");
    }
    int32_t rows = readInt32();
    int32_t cols = readInt32();
    Eigen::MatrixXf mat(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            mat(i, j) = readFloat();
        }
    }
    return mat;
}

// SimDataLogger implementation
SimDataLogger& SimDataLogger::getInstance() {
    static SimDataLogger instance;
    return instance;
}

SimDataLogger::SimDataLogger() : m_frameCounter(0), m_isLogging(false) {
    m_serializer = std::make_unique<BinarySerializer>();
}

SimDataLogger::~SimDataLogger() {
    if (m_isLogging) {
        stopLogging();
    }
}

void SimDataLogger::initialize(const std::string& baseDir) {
    m_baseDir = baseDir;
    // Create directory if it doesn't exist
    std::filesystem::create_directories(m_baseDir);
}

void SimDataLogger::startLogging(const std::string& simulationName) {
    if (m_isLogging) {
        stopLogging();
    }
    
    m_simulationName = simulationName;
    m_frameCounter = 0;
    m_isLogging = true;
    
    std::string filename = m_baseDir + "/" + m_simulationName + ".bin";
    if (!m_serializer->openForWrite(filename)) {
        std::cerr << "Failed to open file for logging: " << filename << std::endl;
        m_isLogging = false;
    } else {
        // Write header
        m_serializer->writeString("SLRBS_SIM_DATA_1.0");
        m_serializer->writeString(m_simulationName);
    }
}

void SimDataLogger::stopLogging() {
    if (m_isLogging) {
        m_serializer->close();
        m_isLogging = false;
    }
}

void SimDataLogger::beginFrame() {
    if (!m_isLogging) return;
    m_serializer->beginFrame(m_frameCounter);
}

void SimDataLogger::endFrame() {
    if (!m_isLogging) return;
    m_serializer->endFrame();
    m_frameCounter++;
}

void SimDataLogger::logVector3f(const std::string& name, const Eigen::Vector3f& vec) {
    if (!m_isLogging) return;
    m_serializer->writeNamedVector3f(name, vec);
}

void SimDataLogger::logMatrix3f(const std::string& name, const Eigen::Matrix3f& mat) {
    if (!m_isLogging) return;
    m_serializer->writeNamedMatrix3f(name, mat);
}

void SimDataLogger::logVectorXf(const std::string& name, const Eigen::VectorXf& vec) {
    if (!m_isLogging) return;
    m_serializer->writeVectorXf(vec);
}

void SimDataLogger::logMatrixXf(const std::string& name, const Eigen::MatrixXf& mat) {
    if (!m_isLogging) return;
    m_serializer->writeMatrixXf(mat);
}

void SimDataLogger::logScalar(const std::string& name, float value) {
    if (!m_isLogging) return;
    m_serializer->writeNamedFloat(name, value);
}

void SimDataLogger::logInt(const std::string& name, int value) {
    if (!m_isLogging) return;
    m_serializer->writeNamedInt32(name, value);
}

void SimDataLogger::logString(const std::string& name, const std::string& value) {
    if (!m_isLogging) return;
    m_serializer->writeNamedString(name, value);
}

void SimDataLogger::logBool(const std::string& name, bool value) {
    if (!m_isLogging) return;
    m_serializer->writeNamedBool(name, value);
}

}  // namespace slrbs
