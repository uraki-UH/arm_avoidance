#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace pcd_protocol {

// Data mask bits
constexpr uint8_t MASK_XYZ = 0x01;      // Always present
constexpr uint8_t MASK_RGB = 0x02;
constexpr uint8_t MASK_INTENSITY = 0x04;
constexpr uint8_t MASK_NORMALS = 0x08;

constexpr uint32_t PROTOCOL_VERSION = 1;
constexpr uint32_t MAGIC = 0x50434458; // "PCDX"
constexpr size_t MAX_SERIALIZED_PAYLOAD_BYTES = 64ULL * 1024ULL * 1024ULL;

#pragma pack(push, 1)
struct ProtocolHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t pointCount;
    uint8_t dataMask;
    uint8_t reserved[3];
    uint32_t payloadSize;
};
#pragma pack(pop)

class PointCloudMessage {
public:
    PointCloudMessage(uint32_t pointCount, uint8_t dataMask)
        : pointCount_(pointCount), dataMask_(dataMask | MASK_XYZ) {}
    
    void setPositions(const std::vector<float>& positions) {
        positions_ = positions;
    }
    
    void setColors(const std::vector<uint8_t>& colors) {
        colors_ = colors;
        dataMask_ |= MASK_RGB;
    }
    
    void setIntensities(const std::vector<float>& intensities) {
        intensities_ = intensities;
        dataMask_ |= MASK_INTENSITY;
    }
    
    std::vector<uint8_t> serialize() const {
        const size_t expectedPointCount = static_cast<size_t>(pointCount_);
        const size_t expectedPositionCount = expectedPointCount * 3;
        if (positions_.size() != expectedPositionCount) {
            throw std::invalid_argument("PointCloudMessage positions size does not match pointCount");
        }
        if ((dataMask_ & MASK_RGB) && colors_.size() != expectedPositionCount) {
            throw std::invalid_argument("PointCloudMessage colors size does not match pointCount");
        }
        if ((dataMask_ & MASK_INTENSITY) && intensities_.size() != expectedPointCount) {
            throw std::invalid_argument("PointCloudMessage intensities size does not match pointCount");
        }

        size_t payloadSize = positions_.size() * sizeof(float);
        if (dataMask_ & MASK_RGB) {
            payloadSize += colors_.size() * sizeof(uint8_t);
        }
        if (dataMask_ & MASK_INTENSITY) {
            payloadSize += intensities_.size() * sizeof(float);
        }
        if (dataMask_ & MASK_NORMALS) {
            throw std::invalid_argument("PointCloudMessage normals are not supported by this serializer");
        }
        if (payloadSize > MAX_SERIALIZED_PAYLOAD_BYTES) {
            throw std::length_error("PointCloudMessage payload exceeds safe serialization limit");
        }
        if (payloadSize > std::numeric_limits<uint32_t>::max()) {
            throw std::length_error("PointCloudMessage payload exceeds header capacity");
        }
        
        std::vector<uint8_t> buffer(sizeof(ProtocolHeader) + payloadSize);
        
        // Write header
        ProtocolHeader header;
        header.magic = MAGIC;
        header.version = PROTOCOL_VERSION;
        header.pointCount = pointCount_;
        header.dataMask = dataMask_;
        header.reserved[0] = header.reserved[1] = header.reserved[2] = 0;
        header.payloadSize = static_cast<uint32_t>(payloadSize);
        
        std::memcpy(buffer.data(), &header, sizeof(ProtocolHeader));
        
        // Write payload
        size_t offset = sizeof(ProtocolHeader);
        
        // Positions
        std::memcpy(buffer.data() + offset, positions_.data(), positions_.size() * sizeof(float));
        offset += positions_.size() * sizeof(float);
        
        // Colors
        if (dataMask_ & MASK_RGB) {
            std::memcpy(buffer.data() + offset, colors_.data(), colors_.size() * sizeof(uint8_t));
            offset += colors_.size() * sizeof(uint8_t);
        }
        
        // Intensities
        if (dataMask_ & MASK_INTENSITY) {
            std::memcpy(buffer.data() + offset, intensities_.data(), intensities_.size() * sizeof(float));
            offset += intensities_.size() * sizeof(float);
        }
        
        return buffer;
    }
    
    // Getter methods
    uint32_t getPointCount() const { return pointCount_; }
    const std::vector<float>& getPositions() const { return positions_; }
    const std::vector<uint8_t>& getColors() const { return colors_; }
    const std::vector<float>& getIntensities() const { return intensities_; }
    uint8_t getDataMask() const { return dataMask_; }
    
private:
    uint32_t pointCount_;
    uint8_t dataMask_;
    std::vector<float> positions_;
    std::vector<uint8_t> colors_;
    std::vector<float> intensities_;
};

} // namespace pcd_protocol
