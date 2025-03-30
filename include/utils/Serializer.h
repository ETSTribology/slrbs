#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <iostream>
#include <cstring>

namespace slrbs {

// DataType enum for type safety in serialization
enum class DataType : uint8_t {
    Int8,
    Int32,
    Float,
    Double,
    String,
    VectorXf,
    MatrixXf,
    Vector3f,
    Matrix3f,
    Bool,
    Array,
    Map,
    FrameMarker
};

// File header structure for versioning and metadata
struct FileHeader {
    char magic[4] = {'S', 'L', 'R', 'B'}; // Magic identifier "SLRB"
    uint32_t version = 1;                 // Format version
    uint32_t recordCount = 0;            // Number of data records
    uint32_t frameCount = 0;             // Number of frames
    uint64_t timeStamp = 0;              // Creation timestamp
};

// Record header structure for each data entry
struct RecordHeader {
    DataType type;
    uint32_t nameLength;
    uint32_t dimensions[2] = {0, 0};  // For vectors and matrices
    uint32_t dataSize;                // Size in bytes
};

// Forward declaration
class BinarySerializer;

// Base class for serializable data
class SerializableData {
public:
    virtual ~SerializableData() {}
    virtual void serialize(BinarySerializer& serializer) const = 0;
    virtual void deserialize(BinarySerializer& serializer) = 0;
};

// Class for binary serialization
class BinarySerializer {
public:
    BinarySerializer();
    ~BinarySerializer();

    // File operations
    bool openForWrite(const std::string& filename, bool append = false);
    bool openForRead(const std::string& filename);
    void close();
    bool isOpen() const;

    // Frame operations
    void beginFrame(int frameId);
    void endFrame();
    
    // Write operations
    void writeDataType(DataType type);
    void writeInt8(int8_t value);
    void writeInt32(int32_t value);
    void writeFloat(float value);
    void writeDouble(double value);
    void writeString(const std::string& value);
    void writeBool(bool value);
    
    // Write with name (recommended)
    void writeNamedInt8(const std::string& name, int8_t value);
    void writeNamedInt32(const std::string& name, int32_t value);
    void writeNamedFloat(const std::string& name, float value);
    void writeNamedDouble(const std::string& name, double value);
    void writeNamedString(const std::string& name, const std::string& value);
    void writeNamedBool(const std::string& name, bool value);
    
    // Eigen types with names
    void writeNamedVector3f(const std::string& name, const Eigen::Vector3f& vec);
    void writeNamedMatrix3f(const std::string& name, const Eigen::Matrix3f& mat);
    
    // Template methods for Eigen types
    template<typename Derived>
    void writeNamedVector(const std::string& name, const Eigen::MatrixBase<Derived>& vector) {
        static_assert(Derived::IsVectorAtCompileTime, "Input must be a vector");
        
        // Write record header
        RecordHeader header;
        header.type = DataType::VectorXf;
        header.nameLength = static_cast<uint32_t>(name.length());
        header.dimensions[0] = vector.size();
        header.dimensions[1] = 1;
        header.dataSize = vector.size() * sizeof(float);
        
        writeRecordHeader(header, name);
        
        // Write vector data
        for (int i = 0; i < vector.size(); i++) {
            float val = static_cast<float>(vector(i));
            m_file.write(reinterpret_cast<const char*>(&val), sizeof(float));
        }
        
        m_fileHeader.recordCount++;
    }
    
    template<typename Derived>
    void writeNamedMatrix(const std::string& name, const Eigen::MatrixBase<Derived>& matrix) {
        // Write record header
        RecordHeader header;
        header.type = DataType::MatrixXf;
        header.nameLength = static_cast<uint32_t>(name.length());
        header.dimensions[0] = matrix.rows();
        header.dimensions[1] = matrix.cols();
        header.dataSize = matrix.rows() * matrix.cols() * sizeof(float);
        
        writeRecordHeader(header, name);
        
        // Write matrix data in row-major order for better cache locality
        for (int i = 0; i < matrix.rows(); i++) {
            for (int j = 0; j < matrix.cols(); j++) {
                float val = static_cast<float>(matrix(i, j));
                m_file.write(reinterpret_cast<const char*>(&val), sizeof(float));
            }
        }
        
        m_fileHeader.recordCount++;
    }
    
    // Legacy read operations (kept for backward compatibility)
    DataType readDataType();
    int8_t readInt8();
    int32_t readInt32();
    float readFloat();
    double readDouble();
    std::string readString();
    bool readBool();
    
    Eigen::Vector3f readVector3f();
    Eigen::Matrix3f readMatrix3f();
    Eigen::VectorXf readVectorXf();
    Eigen::MatrixXf readMatrixXf();

private:
    void writeFileHeader();
    void readFileHeader();
    void updateFileHeader();
    void writeRecordHeader(const RecordHeader& header, const std::string& name);

    std::fstream m_file;
    FileHeader m_fileHeader;
    bool m_isReading;
    bool m_fileExists;
    int m_currentFrame;
};

// Helper class for simulation data logging
class SimDataLogger {
public:
    static SimDataLogger& getInstance();

    void initialize(const std::string& baseDir);
    void startLogging(const std::string& simulationName, bool append = false);
    void stopLogging();

    void beginFrame();
    void endFrame();
    
    // Log data with name
    void logVector3f(const std::string& name, const Eigen::Vector3f& vec);
    void logMatrix3f(const std::string& name, const Eigen::Matrix3f& mat);
    
    template<typename Derived>
    void logVector(const std::string& name, const Eigen::MatrixBase<Derived>& vector) {
        if (!m_isLogging) return;
        m_serializer->writeNamedVector(name, vector);
    }
    
    template<typename Derived>
    void logMatrix(const std::string& name, const Eigen::MatrixBase<Derived>& matrix) {
        if (!m_isLogging) return;
        m_serializer->writeNamedMatrix(name, matrix);
    }
    
    void logScalar(const std::string& name, float value);
    void logInt(const std::string& name, int value);
    void logString(const std::string& name, const std::string& value);
    void logBool(const std::string& name, bool value);

private:
    SimDataLogger();
    ~SimDataLogger();

    std::string m_baseDir;
    std::string m_simulationName;
    int m_frameCounter;
    std::unique_ptr<BinarySerializer> m_serializer;
    bool m_isLogging;
};

}  // namespace slrbs
