#pragma once

#include <string>
#include <vector>
#include <memory>
#include <filesystem>
#include <optional>

namespace ros2_bridge {

struct PointCloudFileInfo {
    std::string path;           // Absolute path
    std::string name;           // File name
    std::string relativePath;   // Path relative to root
    std::string format;         // pcd, ply, las, landxml
    size_t fileSize;
};

class PointCloudFileManager {
public:
    PointCloudFileManager(const std::string& rootDir);
    ~PointCloudFileManager() = default;

    // Scan directory for supported point cloud files
    std::vector<PointCloudFileInfo> scanFiles();
    
    // Load a point cloud file and return raw data
    // Returns: positions (x,y,z,...), colors if available
    struct LoadResult {
        bool success;
        std::vector<float> positions;
        std::vector<uint8_t> colors;  // RGB values
        std::vector<float> intensities;
        uint32_t pointCount;
        std::string errorMessage;
    };
    
    LoadResult loadFile(const std::string& relativePath);

private:
    std::string rootDir_;
    
    // Helper to determine file format
    std::string getFileFormat(const std::filesystem::path& path);
    
    // Loaders for different formats
    LoadResult loadPCD(const std::filesystem::path& path);
    LoadResult loadPLY(const std::filesystem::path& path);
    LoadResult loadLAS(const std::filesystem::path& path);
    LoadResult loadLandXML(const std::filesystem::path& path);
};

} // namespace ros2_bridge
