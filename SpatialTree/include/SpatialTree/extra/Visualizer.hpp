#ifndef SPATIAL_TREE_VISUALIZER_HPP
#define SPATIAL_TREE_VISUALIZER_HPP

#include <SpatialTree/GNG/GNG.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace SpatialTree {

/**
 * @brief 2D 可視化ユーティリティ。
 * 空間分割ツリーと GNG の学習プロセスを OpenCV で描画します。
 */
template <typename Scalar = DefaultScalar>
class Visualizer2D {
public:
    struct Config {
        int width = 800;
        int height = 800;
        Scalar padding = 50.0;
        int frame_rate = 30;
        bool show_window = true;
        std::string video_path = "";
    };

    Visualizer2D(const Config& config, const Point<Scalar, 2>& world_size)
        : config_(config), world_size_(world_size) {
        
        canvas_ = cv::Mat::zeros(config_.height, config_.width, CV_8UC3);
        
        if (!config_.video_path.empty()) {
            writer_.open(config_.video_path, 
                         cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 
                         config_.frame_rate, 
                         cv::Size(config_.width, config_.height));
        }

        if (config_.show_window) {
            cv::namedWindow("SpatialTree Visualization", cv::WINDOW_AUTOSIZE);
        }
    }

    ~Visualizer2D() {
        if (writer_.isOpened()) writer_.release();
        if (config_.show_window) cv::destroyWindow("SpatialTree Visualization");
    }

    template <typename GNGT>
    void recordFrame(const GNGT& gng, const Point<Scalar, 2>& current_sample = Point<Scalar, 2>::Zero()) {
        canvas_.setTo(cv::Scalar(255, 255, 255)); // White background

        // 1. Draw Tree Hierarchy
        gng.getTree().visitCells([this](const auto& cell, int depth) {
            // Draw only leaf cells or all cells with different alpha
            cv::Rect rect = worldToCanvas(cell.bounds);
            cv::Scalar color = depth % 2 == 0 ? cv::Scalar(235, 235, 235) : cv::Scalar(220, 220, 220);
            if (!cell.is_subdivided) {
                cv::rectangle(canvas_, rect, color, -1); // Filled leaf
                cv::rectangle(canvas_, rect, cv::Scalar(210, 210, 210), 1); // Border
            }
        });

        // 1.5 Draw Coordinate Axes (Origin-centered)
        cv::Point origin = pointToCanvas(Point<Scalar, 2>(0, 0));
        cv::line(canvas_, cv::Point(0, origin.y), cv::Point(config_.width, origin.y), cv::Scalar(200, 200, 255), 1); // X-axis (Red-ish)
        cv::line(canvas_, cv::Point(origin.x, 0), cv::Point(origin.x, config_.height), cv::Scalar(200, 255, 200), 1); // Y-axis (Green-ish)

        // 2. Draw Sample Point (Highlight)
        if (!current_sample.isZero()) {
            cv::circle(canvas_, pointToCanvas(current_sample), 5, cv::Scalar(255, 100, 100), -1);
        }

        // 3. Draw GNG Edges
        const auto& nodes = gng.getActiveNodes();
        for (const auto* n1 : nodes) {
            for (const auto& pair : n1->neighbors) {
                const auto* n2 = pair.first;
                if (n1->id < n2->id) { // Draw each edge once
                    cv::line(canvas_, pointToCanvas(n1->position), pointToCanvas(n2->position), 
                             cv::Scalar(100, 200, 100), 1, cv::LINE_8);
                }
            }
        }

        // 4. Draw GNG Nodes
        for (const auto* node : nodes) {
            cv::circle(canvas_, pointToCanvas(node->position), 2, cv::Scalar(50, 50, 200), -1, cv::LINE_8);
        }

        // 5. Display/Save
        if (config_.show_window) {
            cv::imshow("SpatialTree Visualization", canvas_);
            cv::waitKey(1);
        }

        if (writer_.isOpened()) {
            writer_.write(canvas_);
        }
    }

private:
    cv::Point pointToCanvas(const Point<Scalar, 2>& p) const {
        Scalar scale_x = (config_.width - 2 * config_.padding) / world_size_.x();
        Scalar scale_y = (config_.height - 2 * config_.padding) / world_size_.y();
        
        // 中心が (0,0) で、範囲が [-world_size/2, +world_size/2] であると仮定してマッピング
        int x = static_cast<int>((p.x() + world_size_.x() / 2) * scale_x + config_.padding);
        int y = static_cast<int>(config_.height - ((p.y() + world_size_.y() / 2) * scale_y + config_.padding)); // Y軸反転
        return cv::Point(x, y);
    }

    cv::Rect worldToCanvas(const BoundingBox<Scalar, 2>& box) const {
        Point<Scalar, 2> min_pt = box.center - box.half_extents;
        Point<Scalar, 2> max_pt = box.center + box.half_extents;
        
        cv::Point p1 = pointToCanvas(min_pt);
        cv::Point p2 = pointToCanvas(max_pt);
        
        return cv::Rect(p1.x, p2.y, std::abs(p2.x - p1.x), std::abs(p2.y - p1.y));
    }

    Config config_;
    Point<Scalar, 2> world_size_;
    cv::Mat canvas_;
    cv::VideoWriter writer_;
};

template <typename Scalar = DefaultScalar>
class Visualizer3D {
public:
    struct Config {
        int width = 800;
        int height = 800;
        Scalar padding = 50.0;
        int frame_rate = 30;
        bool show_window = true;
        std::string video_path = "";
        bool auto_rotate = true;
    };

    Visualizer3D(const Config& config, const Point<Scalar, 3>& world_size)
        : config_(config), world_size_(world_size), rotation_y_(0.0), rotation_x_(0.4) {
        
        canvas_ = cv::Mat::zeros(config_.height, config_.width, CV_8UC3);
        
        if (!config_.video_path.empty()) {
            writer_.open(config_.video_path, 
                         cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 
                         config_.frame_rate, 
                         cv::Size(config_.width, config_.height));
        }

        if (config_.show_window) {
            cv::namedWindow("SpatialTree 3D Visualization", cv::WINDOW_AUTOSIZE);
        }
    }

    ~Visualizer3D() {
        if (writer_.isOpened()) writer_.release();
        if (config_.show_window) cv::destroyWindow("SpatialTree 3D Visualization");
    }

    template <typename GNGT>
    void recordFrame(const GNGT& gng, const Point<Scalar, 3>& current_sample = Point<Scalar, 3>::Zero()) {
        canvas_.setTo(cv::Scalar(255, 255, 255)); // White background

        if (config_.auto_rotate) {
            rotation_y_ += 0.05; // rotate
        }

        // 1. Draw Tree Hierarchy (3D Wireframe)
        gng.getTree().visitCells([this](const auto& cell, int depth) {
            if (!cell.is_subdivided) {
                Point<Scalar, 3> c = cell.bounds.center;
                Point<Scalar, 3> h = cell.bounds.half_extents;
                
                cv::Point p000 = project3D(c + Point<Scalar,3>(-h.x(), -h.y(), -h.z()));
                cv::Point p100 = project3D(c + Point<Scalar,3>(h.x(), -h.y(), -h.z()));
                cv::Point p010 = project3D(c + Point<Scalar,3>(-h.x(), h.y(), -h.z()));
                cv::Point p110 = project3D(c + Point<Scalar,3>(h.x(), h.y(), -h.z()));
                cv::Point p001 = project3D(c + Point<Scalar,3>(-h.x(), -h.y(), h.z()));
                cv::Point p101 = project3D(c + Point<Scalar,3>(h.x(), -h.y(), h.z()));
                cv::Point p011 = project3D(c + Point<Scalar,3>(-h.x(), h.y(), h.z()));
                cv::Point p111 = project3D(c + Point<Scalar,3>(h.x(), h.y(), h.z()));

                cv::Scalar color(230, 230, 230); // Light gray wireframe
                
                // Bottom face
                cv::line(canvas_, p000, p100, color, 1, cv::LINE_8);
                cv::line(canvas_, p100, p110, color, 1, cv::LINE_8);
                cv::line(canvas_, p110, p010, color, 1, cv::LINE_8);
                cv::line(canvas_, p010, p000, color, 1, cv::LINE_8);
                
                // Top face
                cv::line(canvas_, p001, p101, color, 1, cv::LINE_8);
                cv::line(canvas_, p101, p111, color, 1, cv::LINE_8);
                cv::line(canvas_, p111, p011, color, 1, cv::LINE_8);
                cv::line(canvas_, p011, p001, color, 1, cv::LINE_8);
                
                // Vertical pillars
                cv::line(canvas_, p000, p001, color, 1, cv::LINE_8);
                cv::line(canvas_, p100, p101, color, 1, cv::LINE_8);
                cv::line(canvas_, p010, p011, color, 1, cv::LINE_8);
                cv::line(canvas_, p110, p111, color, 1, cv::LINE_8);
            }
        });

        // 2. Draw 3D axes (centered at origin)
        cv::Point c = project3D(Point<Scalar,3>(0, 0, 0));
        cv::Point px = project3D(Point<Scalar,3>(world_size_.x()*0.4, 0, 0));
        cv::Point py = project3D(Point<Scalar,3>(0, world_size_.y()*0.4, 0));
        cv::Point pz = project3D(Point<Scalar,3>(0, 0, world_size_.z()*0.4));
        cv::line(canvas_, c, px, cv::Scalar(0, 0, 255), 2, cv::LINE_8); // X Red
        cv::line(canvas_, c, py, cv::Scalar(0, 255, 0), 2, cv::LINE_8); // Y Green
        cv::line(canvas_, c, pz, cv::Scalar(255, 0, 0), 2, cv::LINE_8); // Z Blue

        // Draw Sample Point (Highlight)
        if (!current_sample.isZero()) {
            cv::circle(canvas_, project3D(current_sample), 5, cv::Scalar(255, 100, 100), -1);
        }

        // Draw GNG Edges
        const auto& nodes = gng.getActiveNodes();
        for (const auto* n1 : nodes) {
            for (const auto& pair : n1->neighbors) {
                const auto* n2 = pair.first;
                if (n1->id < n2->id) { 
                    cv::line(canvas_, project3D(n1->position), project3D(n2->position), 
                             cv::Scalar(40, 40, 40), 1, cv::LINE_8);
                }
            }
        }

        // Draw GNG Nodes
        for (const auto* node : nodes) {
            cv::circle(canvas_, project3D(node->position), 2, cv::Scalar(50, 50, 200), -1, cv::LINE_8);
        }

        if (config_.show_window) {
            cv::imshow("SpatialTree 3D Visualization", canvas_);
            cv::waitKey(1);
        }
        if (writer_.isOpened()) writer_.write(canvas_);
    }

private:
    cv::Point project3D(const Point<Scalar, 3>& p) const {
        // データがすでに原点中心 ([-world/2, +world/2]) であると仮定
        Scalar tx = p.x();
        Scalar ty = p.y();
        Scalar tz = p.z();

        Scalar rtx = tx * std::cos(rotation_y_) - tz * std::sin(rotation_y_);
        Scalar rtz = tx * std::sin(rotation_y_) + tz * std::cos(rotation_y_);
        Scalar rty = ty * std::cos(rotation_x_) - rtz * std::sin(rotation_x_);
        
        Scalar scale = (std::min(config_.width, config_.height) - 2 * config_.padding) / world_size_.x();
        int px = static_cast<int>(rtx * scale + config_.width / 2);
        int py = static_cast<int>(config_.height / 2 - rty * scale);
        
        return cv::Point(px, py);
    }

    Config config_;
    Point<Scalar, 3> world_size_;
    cv::Mat canvas_;
    cv::VideoWriter writer_;
    Scalar rotation_y_;
    Scalar rotation_x_;
};

} // namespace SpatialTree

#endif // SPATIAL_TREE_VISUALIZER_HPP
