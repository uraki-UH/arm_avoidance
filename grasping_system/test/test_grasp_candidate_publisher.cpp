#include <candidate/grasp_candidate_publisher.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <thread>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    using candidate = gng_control_msgs::msg::GraspCandidate;
    using array = gng_control_msgs::msg::GraspCandidateArray;
    auto source = std::make_shared<rclcpp::Node>("candidate_source_test",
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("reachability_map_topic", "/test/reach_map"),
        rclcpp::Parameter("reachability_voxel_size", 0.1),
        rclcpp::Parameter("reachability_publish_hz", 20.0)}));
    array notified;
    std::size_t notification_num = 0;
    grasping_system::candidate::grasp_candidate_publisher publisher(*source, "/grasp_pose_cands",
      [&](const array &msg) { notified = msg; ++notification_num; });
    auto driver = std::make_shared<rclcpp::Node>("candidate_driver_test");
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    array latest;
    std::size_t num_received = 0;
    auto sub = driver->create_subscription<array>("/grasp_pose_cands", qos,
      [&](array::ConstSharedPtr msg) { latest = *msg; ++num_received; });
    auto map_pub = driver->create_publisher<ais_gng_msgs::msg::TopologicalMap>("/test/reach_map", qos);
    tf2_ros::StaticTransformBroadcaster broadcaster(driver);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(source);
    executor.add_node(driver);
    auto require = [](bool condition) { if (!condition) throw std::runtime_error("候補配信の検証失敗"); };
    auto wait_for = [&](const std::function<bool()> &condition) {
      const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (!condition() && std::chrono::steady_clock::now() < end) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      require(condition());
    };
    array input;
    input.header.frame_id = "world";
    input.header.stamp = source->now();
    input.tcp_frame = "test_tcp";
    for (const double x : {-0.1, -0.10001, 0.09999, 0.1}) {
      candidate entry;
      entry.id = 42 + input.candidates.size();
      entry.pose.position.x = x;
      entry.pose.orientation.w = 1.0;
      input.candidates.push_back(entry);
    }
    publisher.publish(input);
    wait_for([&]() { return num_received > 0; });
    require(latest.candidates[0].state == candidate::UNKNOWN);
    require(notification_num > 0 && notified == latest);
    const auto update_id = latest.update_id;
    ais_gng_msgs::msg::TopologicalMap map;
    map.header.frame_id = "base";
    for (const float x : {-0.05F, 0.01F}) {
      ais_gng_msgs::msg::TopologicalNode entry;
      entry.pos.x = x;
      map.nodes.push_back(entry);
    }
    const auto previous_num = num_received;
    map_pub->publish(map);
    wait_for([&]() { return num_received > previous_num; });
    require(latest.candidates[0].state == candidate::UNKNOWN);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "base";
    tf.child_frame_id = "world";
    tf.transform.rotation.w = 1.0;
    broadcaster.sendTransform(tf);
    wait_for([&]() { return latest.candidates[0].state == candidate::INSIDE; });
    require(latest.update_id == update_id && latest.candidates[0].id == 42 &&
      latest.tcp_frame == "test_tcp");
    require(latest.candidates[1].state == candidate::OUTSIDE);
    require(latest.candidates[2].state == candidate::INSIDE);
    require(latest.candidates[3].state == candidate::OUTSIDE);
    require(notified == latest);
    tf.transform.translation.x = 10.0;
    broadcaster.sendTransform(tf);
    wait_for([&]() { return latest.candidates[0].state == candidate::OUTSIDE; });
    require(notified == latest);
    tf.transform.translation.x = 0.0;
    broadcaster.sendTransform(tf);
    wait_for([&]() { return latest.candidates[0].state == candidate::INSIDE; });
    map.nodes.clear();
    map_pub->publish(map);
    wait_for([&]() { return latest.candidates[0].state == candidate::OUTSIDE; });
    input.candidates[0].pose.position.x = std::numeric_limits<double>::quiet_NaN();
    input.candidates[1].pose.orientation.w = 0.0;
    publisher.publish(input);
    wait_for([&]() { return latest.update_id > update_id; });
    require(latest.candidates[0].state == candidate::UNKNOWN && latest.candidates[1].state == candidate::UNKNOWN);
    const auto previous_id = latest.update_id;
    input.candidates.clear();
    publisher.publish(input);
    wait_for([&]() { return latest.update_id > previous_id && latest.candidates.empty(); });
    require(notified == latest && notified.candidates.empty());

    // 遅着する動的TFの補間による初回配信。観測時刻と候補IDの保持
    tf2_ros::TransformBroadcaster dynamic_broadcaster(driver);
    map.nodes.resize(1);
    map.nodes[0].pos.x = 0.01F;
    map_pub->publish(map);
    const auto observation_time = source->now();
    tf.child_frame_id = "moving_sensor";
    tf.header.stamp = observation_time - rclcpp::Duration::from_seconds(0.1);
    dynamic_broadcaster.sendTransform(tf);
    input.header.frame_id = tf.child_frame_id;
    input.header.stamp = tf.header.stamp;
    input.candidates.resize(1);
    input.candidates[0].id = 77;
    input.candidates[0].pose.position.x = 0.01;
    input.candidates[0].pose.orientation.w = 1.0;
    publisher.publish(input);
    wait_for([&]() { return notified.candidates[0].state == candidate::INSIDE; });
    input.header.stamp = observation_time;
    tf.header.stamp = observation_time + rclcpp::Duration::from_seconds(0.1);
    tf.transform.translation.x = 0.2;
    const auto before_delayed_tf = notification_num;
    auto delayed_tf = std::async(std::launch::async, [&]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
      dynamic_broadcaster.sendTransform(tf);
    });
    publisher.publish(input);
    delayed_tf.get();
    require(notification_num == before_delayed_tf + 1);
    require(notified.candidates[0].state == candidate::OUTSIDE && notified.candidates[0].id == 77);
    require(notified.header == input.header);

    // 補間不能時の未評価。古い変換での到達判定や無期限待機の禁止
    input.header.stamp = observation_time + rclcpp::Duration::from_seconds(10.0);
    publisher.publish(input);
    require(notified.candidates[0].state == candidate::UNKNOWN);
    input.candidates.clear();
    publisher.publish(input);
    require(notified.candidates.empty());
    require(driver->count_publishers("/grasp_pose_cands") == 1);
    require(driver->count_publishers("/grasp_pose_cands/reachability") == 0);
    require(driver->count_publishers("/grasp_pose_cands/reachability_markers") == 0);
    std::cout << "成功: 未評価・TF追従・遅着TF補間・境界・ID保持・空候補・単一配信元\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
