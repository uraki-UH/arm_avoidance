#include "topo_fuzzy_viewer/services/pointcloud_file_manager.h"
#include "topo_fuzzy_viewer/common/pcl_converter.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

namespace ros2_bridge {

PointCloudFileManager::PointCloudFileManager(const std::string& rootDir) : rootDir_(rootDir) {
    if (!fs::exists(rootDir_)) {
        std::cerr << "PointCloud root directory does not exist: " << rootDir_ << std::endl;
    }
}

std::string PointCloudFileManager::getFileFormat(const fs::path& path) {
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    
    if (ext == ".pcd") return "pcd";
    if (ext == ".ply") return "ply";
    if (ext == ".las" || ext == ".laz") return "las";
    if (ext == ".xml" || ext == ".landxml" || ext == ".landxm") return "landxml";
    return "";
}

std::vector<PointCloudFileInfo> PointCloudFileManager::scanFiles() {
    std::vector<PointCloudFileInfo> files;
    if (!fs::exists(rootDir_)) return files;

    try {
        for (const auto& entry : fs::recursive_directory_iterator(rootDir_)) {
            if (entry.is_regular_file()) {
                std::string format = getFileFormat(entry.path());
                if (!format.empty()) {
                    PointCloudFileInfo info;
                    info.path = entry.path().string();
                    info.name = entry.path().filename().string();
                    
                    // Create relative path
                    std::string fullPath = entry.path().string();
                    if (fullPath.find(rootDir_) == 0) {
                        info.relativePath = fullPath.substr(rootDir_.length());
                        if (!info.relativePath.empty() && info.relativePath.front() == '/') {
                            info.relativePath.erase(0, 1);
                        }
                    } else {
                        info.relativePath = info.name;
                    }
                    
                    info.format = format;
                    info.fileSize = entry.file_size();
                    
                    files.push_back(info);
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error scanning files: " << e.what() << std::endl;
    }
    
    // Sort by name
    std::sort(files.begin(), files.end(), [](const PointCloudFileInfo& a, const PointCloudFileInfo& b) {
        return a.relativePath < b.relativePath;
    });

    return files;
}

PointCloudFileManager::LoadResult PointCloudFileManager::loadFile(const std::string& relativePath) {
    fs::path filePath = fs::path(rootDir_) / relativePath;
    
    if (!fs::exists(filePath)) {
        return { false, {}, {}, {}, 0, "File not found: " + filePath.string() };
    }
    
    std::string format = getFileFormat(filePath);
    
    if (format == "pcd") {
        return loadPCD(filePath);
    } else if (format == "ply") {
        return loadPLY(filePath);
    } else if (format == "las") {
        return loadLAS(filePath);
    } else if (format == "landxml") {
        return loadLandXML(filePath);
    }
    
    return { false, {}, {}, {}, 0, "Unsupported format: " + format };
}

PointCloudFileManager::LoadResult PointCloudFileManager::loadPCD(const fs::path& path) {
    LoadResult result;
    result.success = false;
    
    // Try loading as XYZRGB first
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(path.string(), *cloudRGB) == 0 && !cloudRGB->empty()) {
        auto data = utils::convertFromPclCloud(cloudRGB);
        result.positions = std::move(data.positions);
        result.colors = std::move(data.colors);
        result.pointCount = data.pointCount;
        result.success = true;
        return result;
    }
    
    // Try loading as XYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudI(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(path.string(), *cloudI) == 0 && !cloudI->empty()) {
        auto data = utils::convertFromPclCloud(cloudI);
        result.positions = std::move(data.positions);
        result.intensities = std::move(data.intensities);
        result.pointCount = data.pointCount;
        result.success = true;
        return result;
    }
    
    // Try loading as XYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path.string(), *cloudXYZ) == 0) {
        auto data = utils::convertFromPclCloud(cloudXYZ);
        result.positions = std::move(data.positions);
        result.pointCount = data.pointCount;
        result.success = true;
        return result;
    }
    
    result.errorMessage = "Failed to load PCD file";
    return result;
}

PointCloudFileManager::LoadResult PointCloudFileManager::loadPLY(const fs::path& path) {
    LoadResult result;
    result.success = false;
    
    // Try loading as XYZRGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(path.string(), *cloudRGB) == 0 && !cloudRGB->empty()) {
        auto data = utils::convertFromPclCloud(cloudRGB);
        result.positions = std::move(data.positions);
        result.colors = std::move(data.colors);
        result.pointCount = data.pointCount;
        result.success = true;
        return result;
    }
    
    // Try loading as XYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(path.string(), *cloudXYZ) == 0) {
        auto data = utils::convertFromPclCloud(cloudXYZ);
        result.positions = std::move(data.positions);
        result.pointCount = data.pointCount;
        result.success = true;
        return result;
    }
    
    result.errorMessage = "Failed to load PLY file";
    return result;
}

PointCloudFileManager::LoadResult PointCloudFileManager::loadLAS(const fs::path& path) {
    LoadResult result;

    if (path.extension() == ".laz") {
        result.success = false;
        result.errorMessage = "LAZ (compressed LAS) is not supported. Please convert to LAS.";
        return result;
    }

    std::ifstream file(path, std::ios::binary);
    if (!file) {
        result.success = false;
        result.errorMessage = "Failed to open LAS file";
        return result;
    }

    std::vector<uint8_t> header(227);
    file.read(reinterpret_cast<char*>(header.data()), header.size());
    if (file.gcount() < static_cast<std::streamsize>(header.size())) {
        result.success = false;
        result.errorMessage = "Invalid LAS header";
        return result;
    }

    auto readU16 = [](const std::vector<uint8_t>& buffer, size_t offset) -> uint16_t {
        return static_cast<uint16_t>(buffer[offset]) |
            (static_cast<uint16_t>(buffer[offset + 1]) << 8);
    };
    auto readU32 = [](const std::vector<uint8_t>& buffer, size_t offset) -> uint32_t {
        return static_cast<uint32_t>(buffer[offset]) |
            (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
            (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
            (static_cast<uint32_t>(buffer[offset + 3]) << 24);
    };
    auto readI32 = [](const uint8_t* buffer) -> int32_t {
        return static_cast<int32_t>(buffer[0]) |
            (static_cast<int32_t>(buffer[1]) << 8) |
            (static_cast<int32_t>(buffer[2]) << 16) |
            (static_cast<int32_t>(buffer[3]) << 24);
    };
    auto readF64 = [](const std::vector<uint8_t>& buffer, size_t offset) -> double {
        double value = 0.0;
        std::memcpy(&value, buffer.data() + offset, sizeof(double));
        return value;
    };

    if (!(header[0] == 'L' && header[1] == 'A' && header[2] == 'S' && header[3] == 'F')) {
        result.success = false;
        result.errorMessage = "Invalid LAS signature";
        return result;
    }

    const uint32_t pointDataOffset = readU32(header, 96);
    const uint16_t pointRecordLength = readU16(header, 105);
    uint32_t pointCount = readU32(header, 107);

    if (pointRecordLength < 12) {
        result.success = false;
        result.errorMessage = "Unsupported LAS point record length";
        return result;
    }

    const double scaleX = readF64(header, 131);
    const double scaleY = readF64(header, 139);
    const double scaleZ = readF64(header, 147);
    const double offsetX = readF64(header, 155);
    const double offsetY = readF64(header, 163);
    const double offsetZ = readF64(header, 171);

    if (pointCount == 0) {
        const auto fileSize = fs::file_size(path);
        if (fileSize > pointDataOffset && pointRecordLength > 0) {
            pointCount = static_cast<uint32_t>((fileSize - pointDataOffset) / pointRecordLength);
        }
    }

    if (pointCount == 0) {
        result.success = false;
        result.errorMessage = "No points found in LAS file";
        return result;
    }

    const bool hasIntensity = pointRecordLength >= 14;

    result.positions.resize(static_cast<size_t>(pointCount) * 3);
    if (hasIntensity) {
        result.intensities.resize(pointCount);
    }

    file.seekg(static_cast<std::streamoff>(pointDataOffset), std::ios::beg);
    if (!file) {
        result.success = false;
        result.errorMessage = "Failed to seek to LAS point data";
        return result;
    }

    const size_t recordsPerChunk = 100000;
    std::vector<uint8_t> buffer(static_cast<size_t>(pointRecordLength) * recordsPerChunk);

    uint32_t pointsRead = 0;
    while (pointsRead < pointCount && file) {
        const uint32_t remaining = pointCount - pointsRead;
        const uint32_t toRead = remaining < recordsPerChunk ? remaining : static_cast<uint32_t>(recordsPerChunk);
        const size_t bytesToRead = static_cast<size_t>(toRead) * pointRecordLength;

        file.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(bytesToRead));
        const size_t bytesRead = static_cast<size_t>(file.gcount());
        const size_t recordsRead = bytesRead / pointRecordLength;

        for (size_t i = 0; i < recordsRead; ++i) {
            const uint8_t* record = buffer.data() + i * pointRecordLength;
            const int32_t ix = readI32(record);
            const int32_t iy = readI32(record + 4);
            const int32_t iz = readI32(record + 8);

            const size_t baseIndex = static_cast<size_t>(pointsRead) * 3;
            result.positions[baseIndex] = static_cast<float>(ix * scaleX + offsetX);
            result.positions[baseIndex + 1] = static_cast<float>(iy * scaleY + offsetY);
            result.positions[baseIndex + 2] = static_cast<float>(iz * scaleZ + offsetZ);

            if (hasIntensity) {
                const uint16_t intensity = static_cast<uint16_t>(record[12]) |
                    (static_cast<uint16_t>(record[13]) << 8);
                result.intensities[pointsRead] = static_cast<float>(intensity) / 65535.0f;
            }

            pointsRead++;
        }

        if (recordsRead < toRead) {
            break;
        }
    }

    if (pointsRead == 0) {
        result.success = false;
        result.errorMessage = "No points read from LAS file";
        return result;
    }

    result.positions.resize(static_cast<size_t>(pointsRead) * 3);
    if (hasIntensity) {
        result.intensities.resize(pointsRead);
    }
    result.pointCount = pointsRead;
    result.success = true;
    return result;
}

PointCloudFileManager::LoadResult PointCloudFileManager::loadLandXML(const fs::path& path) {
    LoadResult result;
    result.success = false;

    std::ifstream file(path);
    if (!file) {
        result.errorMessage = "Failed to open LandXML file";
        return result;
    }

    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if (content.empty()) {
        result.errorMessage = "Empty LandXML file";
        return result;
    }

    std::vector<float> positions;
    positions.reserve(1024);

    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;
    size_t validCount = 0;

    size_t searchPos = 0;
    while (true) {
        const size_t tagPos = content.find("<P", searchPos);
        if (tagPos == std::string::npos) break;

        const size_t tagNamePos = tagPos + 2;
        if (tagNamePos >= content.size()) break;
        const char nextChar = content[tagNamePos];
        if (!(nextChar == '>' || std::isspace(static_cast<unsigned char>(nextChar)))) {
            searchPos = tagNamePos;
            continue;
        }

        const size_t textStart = content.find('>', tagPos);
        if (textStart == std::string::npos) break;

        const size_t textEnd = content.find("</P>", textStart);
        if (textEnd == std::string::npos) {
            searchPos = textStart + 1;
            continue;
        }

        const std::string pointText = content.substr(textStart + 1, textEnd - textStart - 1);
        std::istringstream iss(pointText);
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (iss >> x >> y >> z) {
            positions.push_back(static_cast<float>(x));
            positions.push_back(static_cast<float>(y));
            positions.push_back(static_cast<float>(z));
            sumX += x;
            sumY += y;
            sumZ += z;
            validCount++;
        }

        searchPos = textEnd + 4;
    }

    if (validCount == 0) {
        result.errorMessage = "No points found in LandXML file";
        return result;
    }

    const double centroidX = sumX / static_cast<double>(validCount);
    const double centroidY = sumY / static_cast<double>(validCount);
    const double centroidZ = sumZ / static_cast<double>(validCount);

    for (size_t i = 0; i + 2 < positions.size(); i += 3) {
        positions[i] = static_cast<float>(positions[i] - centroidX);
        positions[i + 1] = static_cast<float>(positions[i + 1] - centroidY);
        positions[i + 2] = static_cast<float>(positions[i + 2] - centroidZ);
    }

    result.positions = std::move(positions);
    result.pointCount = static_cast<uint32_t>(validCount);
    result.success = true;
    return result;
}

} // namespace ros2_bridge
