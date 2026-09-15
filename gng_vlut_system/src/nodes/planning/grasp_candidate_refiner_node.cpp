#include "grasp_candidate_refinement.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include "nodes/bridge/world_point_bucket_index.hpp"

#include <gng_control_msgs/msg/grasp_candidate_array.hpp>
#include <gng_control_msgs/msg/grasp_candidate_metric_array.hpp>
#include <gng_control_msgs/msg/grasp_refinement_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <cstring>
#include <map>
#include <set>
#include <sstream>

namespace
{
using source_array=gng_control_msgs::msg::GraspCandidateArray;
using seed_array=gng_control_msgs::msg::GraspCandidateMetricArray;
using output_array=gng_control_msgs::msg::GraspRefinementArray;
using output_candidate=gng_control_msgs::msg::GraspRefinement;
using steady_clock=std::chrono::steady_clock;

Eigen::Isometry3d to_transform(const geometry_msgs::msg::Pose &pose)
{
  const auto &p=pose.position; const auto &q=pose.orientation;
  Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
  if (!rotation.coeffs().allFinite() || !std::isfinite(rotation.norm()) || rotation.norm()<1e-8 ||
    !Eigen::Vector3d(p.x,p.y,p.z).allFinite()) throw std::invalid_argument("invalid_pose");
  Eigen::Isometry3d out=Eigen::Isometry3d::Identity();
  out.linear()=rotation.normalized().toRotationMatrix();
  out.translation()=Eigen::Vector3d(p.x,p.y,p.z);
  return out;
}

geometry_msgs::msg::Point to_point(const Eigen::Vector3d &point)
{
  geometry_msgs::msg::Point out;
  out.x=point.x(); out.y=point.y(); out.z=point.z();
  return out;
}

geometry_msgs::msg::Pose to_pose(const Eigen::Isometry3d &pose)
{
  geometry_msgs::msg::Pose out;
  out.position=to_point(pose.translation());
  const Eigen::Quaterniond rotation(pose.linear());
  out.orientation.x=rotation.x(); out.orientation.y=rotation.y();
  out.orientation.z=rotation.z(); out.orientation.w=rotation.w();
  return out;
}

class grasp_candidate_refiner : public rclcpp::Node
{
public:
  grasp_candidate_refiner():Node("grasp_candidate_refiner"),buffer_(get_clock()),listener_(buffer_)
  {
    const auto input_topic=declare_parameter("candidate_topic","/grasp_pose_cands");
    const auto cloud_topic=declare_parameter("point_cloud_topic","/camera/camera/depth/color/points");
    const auto seed_topic=declare_parameter("seed_topic","/ToPoDualArm/grasp_candidate_metrics");
    const auto output_topic=declare_parameter("output_topic","/grasp_pose_refined");
    for (const auto &input:{input_topic,cloud_topic,seed_topic})
      if (get_node_topics_interface()->resolve_topic_name(input)==get_node_topics_interface()->resolve_topic_name(output_topic))
        throw std::invalid_argument("refinement output must differ from every input");
    const double update_hz=declare_parameter("update_hz",2.0);
    max_input_age_sec_=declare_parameter("max_input_age_sec",2.0);
    max_stamp_diff_sec_=declare_parameter("max_stamp_diff_sec",0.5);
    max_candidates_=positive_count("max_candidates",20);
    max_cloud_points_=positive_count("max_cloud_points",1000000);
    max_local_points_=positive_count("max_local_points",30000);
    max_ik_seeds_=positive_count("max_ik_seeds",3);
    max_ik_iter_=positive_count("max_ik_iter",120);
    max_position_error_=declare_parameter("max_position_error",0.002);
    max_orientation_error_deg_=declare_parameter("max_orientation_error_deg",3.0);
    max_seed_dist_=declare_parameter("max_seed_dist",0.15);
    max_joint_change_deg_=declare_parameter("max_joint_change_deg",25.0);
    root_frame_=declare_parameter("root_frame","ToPoDualArm/L_shoulder_mount");
    tcp_frame_=declare_parameter("tcp_frame","L_tcp");
    const auto root_link=declare_parameter("root_link","L_shoulder_mount");
    gripper_joint_=declare_parameter("gripper_joint","L_gripper_joint");
    gripper_mimic_joint_=declare_parameter("gripper_mimic_joint","L_gripper_mimic");
    closed_width_=declare_parameter("closed_width",0.0);
    enable_ik_=declare_parameter("enable_ik",true);
    const auto urdf_path=declare_parameter("urdf_path","/ros2_ws/src/dual_arm_urdf/dual_arm_robot.urdf");
    config_.min_width=declare_parameter("min_width",config_.min_width);
    config_.max_width=declare_parameter("max_width",config_.max_width);
    config_.finger_span=declare_parameter("finger_span",config_.finger_span);
    config_.finger_length=declare_parameter("finger_length",config_.finger_length);
    config_.finger_thickness=declare_parameter("finger_thickness",config_.finger_thickness);
    config_.base_span=declare_parameter("base_span",config_.base_span);
    config_.base_width=declare_parameter("base_width",config_.base_width);
    config_.min_base_depth=declare_parameter("min_base_depth",config_.min_base_depth);
    config_.max_base_depth=declare_parameter("max_base_depth",config_.max_base_depth);
    config_.opening_margin=declare_parameter("opening_margin",config_.opening_margin);
    config_.collision_margin=declare_parameter("collision_margin",config_.collision_margin);
    config_.approach_length=declare_parameter("approach_length",config_.approach_length);
    config_.contact_band=declare_parameter("contact_band",config_.contact_band);
    config_.min_contact_depth=declare_parameter("min_contact_depth",config_.min_contact_depth);
    config_.max_contact_depth=declare_parameter("max_contact_depth",config_.max_contact_depth);
    config_.min_contact_spread=declare_parameter("min_contact_spread",config_.min_contact_spread);
    config_.max_contact_variation=declare_parameter("max_contact_variation",config_.max_contact_variation);
    config_.max_contact_normal_deg=declare_parameter("max_contact_normal_deg",config_.max_contact_normal_deg);
    config_.max_center_shift=declare_parameter("max_center_shift",config_.max_center_shift);
    config_.min_contact_points=positive_count("min_contact_points",config_.min_contact_points);
    config_.tcp_rotation_x_deg=declare_parameter("tcp_rotation_x_deg",config_.tcp_rotation_x_deg);
    config_.yaw_offsets_deg=declare_parameter("yaw_offsets_deg",config_.yaw_offsets_deg);
    config_.insertion_depths=declare_parameter("insertion_depths",config_.insertion_depths);
    grasp_refinement::validate(config_);
    for (double value:{update_hz,max_input_age_sec_,max_stamp_diff_sec_,max_position_error_,
        max_orientation_error_deg_,max_seed_dist_,max_joint_change_deg_})
      if (!std::isfinite(value) || value<=0) throw std::invalid_argument("invalid refinement limit");
    if (!std::isfinite(closed_width_) || closed_width_<0 || closed_width_>config_.min_width)
      throw std::invalid_argument("invalid closed_width");
    if (enable_ik_) {
      const auto model=simulation::loadRobotFromUrdf(urdf_path);
      if (!model.getLink(root_link) || !model.getLink(tcp_frame_))
        throw std::invalid_argument("unknown refinement chain link");
      chain_=std::make_unique<kinematics::KinematicChain>(simulation::createKinematicChainFromModel(
        model,tcp_frame_,Eigen::Vector3d::Zero(),root_link));
      // root_frameのTFで移動する関節鎖。URDF祖先のゼロ姿勢オフセットの二重適用なし
      chain_->setBase(Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity());
      for (int idx=0;idx<chain_->getNumJoints();++idx) {
        if (chain_->getJointDOF(idx)==0) continue;
        if (chain_->getJointDOF(idx)!=1) throw std::invalid_argument("only single-DOF arm joints are supported");
        joint_names_.push_back(chain_->getJointName(idx));
      }
      const auto *gripper=model.getJoint(gripper_joint_);
      const auto *mimic=model.getJoint(gripper_mimic_joint_);
      if (!gripper || !mimic || gripper->type!=kinematics::JointType::Prismatic ||
        !mimic->has_mimic || mimic->mimic_joint_name!=gripper_joint_ ||
        mimic->mimic_multiplier!=-1 || mimic->mimic_offset!=0 ||
        gripper->limits.lower>0.5*(config_.min_width-closed_width_) ||
        gripper->limits.upper<0.5*(config_.max_width-closed_width_))
        throw std::invalid_argument("width configuration does not match the symmetric prismatic gripper");
    }
    const auto qos=rclcpp::QoS(1).reliable().transient_local();
    output_=create_publisher<output_array>(output_topic,qos);
    markers_=create_publisher<visualization_msgs::msg::MarkerArray>(output_topic+"/markers",qos);
    candidates_sub_=create_subscription<source_array>(input_topic,qos,[this](source_array::ConstSharedPtr value) {
      source_=std::move(value); source_received_=steady_clock::now(); has_update_=true;
    });
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic,rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr value) {
        cloud_=std::move(value); cloud_received_=steady_clock::now(); has_update_=true;
      });
    seeds_sub_=create_subscription<seed_array>(seed_topic,qos,[this](seed_array::ConstSharedPtr value) {
      seeds_=std::move(value); has_update_=true;
    });
    status_="waiting_for_candidates: "+std::string(candidates_sub_->get_topic_name());
    timer_=create_wall_timer(std::chrono::duration<double>(1.0/update_hz),[this] { update(); report_status(); });
    RCLCPP_INFO(get_logger(),"Grasp refinement: %s | %.1f Hz | IK=%s",output_topic.c_str(),update_hz,enable_ik_ ? "on":"off");
    RCLCPP_INFO(get_logger(),"Inputs: candidates=%s points=%s seeds=%s | max_local_points=%zu",
      candidates_sub_->get_topic_name(),cloud_sub_->get_topic_name(),seeds_sub_->get_topic_name(),max_local_points_);
  }

private:
  void report_status()
  {
    // 状態変化時のみ、最短5秒間隔の理由表示。入力停止後の最終状態も表示対象
    const auto now=steady_clock::now();
    if (status_==reported_status_ || (!reported_status_.empty() &&
      std::chrono::duration<double>(now-status_reported_).count()<5.0)) return;
    RCLCPP_INFO(get_logger(),"Refine: %s",status_.c_str());
    reported_status_=status_; status_reported_=now;
  }

  std::size_t positive_count(const std::string &name,std::size_t fallback)
  {
    const auto value=declare_parameter<int64_t>(name,static_cast<int64_t>(fallback));
    if (value<=0 || value>10000000) throw std::invalid_argument("invalid "+name);
    return static_cast<std::size_t>(value);
  }

  Eigen::Isometry3d transform(const std::string &target,const std::string &source,
    const builtin_interfaces::msg::Time &stamp)
  {
    if (target.empty() || source.empty()) throw std::invalid_argument("missing_frame");
    if (target==source) return Eigen::Isometry3d::Identity();
    return tf2::transformToEigen(buffer_.lookupTransform(target,source,rclcpp::Time(stamp),rclcpp::Duration::from_seconds(0.0)));
  }

  void prepare_cloud()
  {
    if (indexed_cloud_==cloud_) return;
    indexed_cloud_.reset();
    point_index_=robot_sim::bridge::world_point_bucket_index(0.05);
    if (static_cast<std::size_t>(cloud_->width)*cloud_->height>max_cloud_points_)
      throw std::runtime_error("cloud_budget");
    std::vector<std::size_t> offsets;
    for (const auto *name:{"x","y","z"}) {
      const auto field=std::find_if(cloud_->fields.begin(),cloud_->fields.end(),[&](const auto &entry){return entry.name==name;});
      if (field==cloud_->fields.end() || field->datatype!=sensor_msgs::msg::PointField::FLOAT32 ||
        field->count!=1 || field->offset+4>cloud_->point_step) throw std::runtime_error("invalid_cloud_fields");
      offsets.push_back(field->offset);
    }
    if (cloud_->is_bigendian || cloud_->row_step<static_cast<std::size_t>(cloud_->width)*cloud_->point_step ||
      cloud_->data.size()<static_cast<std::size_t>(cloud_->row_step)*cloud_->height)
      throw std::runtime_error("invalid_cloud_layout");
    point_index_.begin_frame(static_cast<std::size_t>(cloud_->width)*cloud_->height);
    for (std::size_t row=0;row<cloud_->height;++row) for (std::size_t col=0;col<cloud_->width;++col) {
      const auto *data=cloud_->data.data()+row*cloud_->row_step+col*cloud_->point_step;
      Eigen::Vector3f point;
      for (std::size_t axis=0;axis<3;++axis) std::memcpy(&point[axis],data+offsets[axis],sizeof(float));
      point_index_.add_point(point);
    }
    if (point_index_.point_num()==0) throw std::runtime_error("empty_point_cloud");
    indexed_cloud_=cloud_;
  }

  void solve_joints(output_candidate &out,const Eigen::Isometry3d &target,
    const std::vector<Eigen::Vector3d> &points)
  {
    if (!enable_ik_) { out.reason="ik_disabled"; return; }
    if (!seeds_ || seeds_->candidates.empty()) { out.reason="no_joint_seeds"; return; }
    const auto source_to_root=transform(root_frame_,source_->header.frame_id,cloud_->header.stamp);
    const auto root_target=source_to_root*target;
    struct seed { double dist; int id; std::vector<double> values; };
    std::vector<seed> candidates;
    for (const auto &entry:seeds_->candidates) {
      if (!entry.feasible) continue;
      const auto &state=entry.final_joint_state;
      if (state.name.size()!=state.position.size()) continue;
      std::map<std::string,double> by_name;
      for (std::size_t idx=0;idx<state.name.size();++idx) by_name.emplace(state.name[idx],state.position[idx]);
      if (by_name.size()!=state.name.size()) continue;
      std::vector<double> values;
      for (const auto &name:joint_names_) {
        const auto found=by_name.find(name);
        if (found==by_name.end() || !std::isfinite(found->second)) break;
        values.push_back(found->second);
      }
      if (values.size()!=joint_names_.size() || !chain_->isWithinLimits(values)) continue;
      std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> positions;
      std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> rotations;
      chain_->forwardKinematicsAt(values,positions,rotations);
      const double dist=(positions.back()-root_target.translation()).norm();
      if (dist<=max_seed_dist_) candidates.push_back({dist,entry.goal_node_id,std::move(values)});
    }
    std::sort(candidates.begin(),candidates.end(),[](const auto &a,const auto &b){return a.dist<b.dist;});
    out.reason=candidates.empty() ? "no_near_joint_seed":"ik_not_converged";
    const double max_angle=max_orientation_error_deg_*3.14159265358979323846/180;
    for (std::size_t idx=0;idx<std::min(max_ik_seeds_,candidates.size());++idx) {
      const auto &seed=candidates[idx];
      std::vector<double> values;
      if (!chain_->inverseKinematicsAt(chain_->getNumJoints()+1,root_target.translation(),
        Eigen::Quaterniond(root_target.linear()),seed.values,static_cast<int>(max_ik_iter_),
        max_position_error_,max_angle,values)) continue;
      bool has_large_change=false;
      for (std::size_t j=0;j<values.size();++j)
        if (!std::isfinite(values[j]) || std::abs(values[j]-seed.values[j])>max_joint_change_deg_*3.14159265358979323846/180) has_large_change=true;
      if (has_large_change || !chain_->isWithinLimits(values)) { out.reason="joint_change_limit"; continue; }
      std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> positions;
      std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> rotations;
      chain_->forwardKinematicsAt(values,positions,rotations);
      out.position_error=(positions.back()-root_target.translation()).norm();
      out.orientation_error_deg=rotations.back().angularDistance(Eigen::Quaterniond(root_target.linear()))*180/3.14159265358979323846;
      if (out.position_error>max_position_error_ || out.orientation_error_deg>max_orientation_error_deg_) continue;
      Eigen::Isometry3d achieved=Eigen::Isometry3d::Identity();
      achieved.translation()=positions.back(); achieved.linear()=rotations.back().toRotationMatrix();
      achieved=source_to_root.inverse()*achieved;
      // IKの残差を含む実TCP姿勢で、観測点とのグリッパ掃引を再検査
      if (grasp_refinement::observed_collision(points,achieved,out.contact_width,out.opening_width,config_)) {
        out.has_observed_collision=true; out.reason="ik_gripper_collision"; continue;
      }
      out.has_observed_collision=false;
      out.joint_pose=to_pose(achieved);
      out.has_joint_solution=true; out.seed_node_id=seed.id;
      out.joint_state.header=source_->header;
      out.joint_state.header.stamp=cloud_->header.stamp;
      out.joint_state.name=joint_names_; out.joint_state.position=std::move(values);
      // 進入時の開口を関節値へ変換。接触幅は別フィールドで保持
      out.joint_state.name.push_back(gripper_joint_);
      out.joint_state.position.push_back(0.5*(out.opening_width-closed_width_));
      out.joint_state.name.push_back(gripper_mimic_joint_);
      out.joint_state.position.push_back(-0.5*(out.opening_width-closed_width_));
      out.reason="refined_arm_path_unchecked";
      return;
    }
  }

  void update()
  {
    if (!source_) return;
    const auto begin=steady_clock::now();
    const bool is_stale=std::chrono::duration<double>(begin-source_received_).count()>max_input_age_sec_ ||
      (cloud_ && std::chrono::duration<double>(begin-cloud_received_).count()>max_input_age_sec_);
    if (!has_update_ && is_stale==is_stale_output_) return;
    has_update_=false; is_stale_output_=is_stale;
    output_array out;
    out.header=out.source_header=source_->header;
    out.source_update_id=source_->update_id; out.tcp_frame=tcp_frame_;
    if (cloud_) { out.cloud_header=cloud_->header; out.header.stamp=cloud_->header.stamp; }
    std::string unavailable;
    Eigen::Isometry3d cloud_to_source=Eigen::Isometry3d::Identity();
    if (!source_->candidates.empty()) {
      if (is_stale) unavailable="stale_input";
      else if (!cloud_) unavailable="no_point_cloud";
      else if (source_->tcp_frame!=tcp_frame_) unavailable="tcp_frame_mismatch";
      else if (std::abs((rclcpp::Time(source_->header.stamp)-rclcpp::Time(cloud_->header.stamp)).seconds())>max_stamp_diff_sec_)
        unavailable="stamp_mismatch";
      else try {
        cloud_to_source=transform(source_->header.frame_id,cloud_->header.frame_id,cloud_->header.stamp);
        prepare_cloud();
      } catch (const tf2::TransformException &) { unavailable="cloud_transform_unavailable"; }
        catch (const std::exception &error) { unavailable=error.what(); }
    }
    std::set<std::uint32_t> ids;
    bool has_duplicate_id=false;
    for (const auto &entry:source_->candidates) if (!ids.insert(entry.id).second) has_duplicate_id=true;
    for (std::size_t idx=0;idx<source_->candidates.size();++idx) {
      const auto &input=source_->candidates[idx];
      output_candidate candidate;
      candidate.source_candidate_id=input.id; candidate.source_pose=candidate.refined_pose=input.pose;
      candidate.observed_width=candidate.contact_width=candidate.opening_width=candidate.position_error=candidate.orientation_error_deg=std::numeric_limits<double>::quiet_NaN();
      candidate.reason=unavailable;
      if (has_duplicate_id) candidate.reason="duplicate_candidate_id";
      else if (idx>=max_candidates_) candidate.reason="candidate_budget";
      else if (unavailable.empty()) try {
        const auto source_pose=to_transform(input.pose);
        auto bounds=grasp_refinement::search_bounds(cloud_to_source.inverse()*source_pose,config_);
        if (enable_ik_) {
          // IKの並進・回転残差による掃引領域の拡大。箱対角長を回転半径の上界として利用
          const double margin=max_position_error_+2*(bounds.second-bounds.first).norm()*
            std::sin(std::min(180.0,max_orientation_error_deg_)*3.14159265358979323846/360);
          bounds.first.array()-=margin; bounds.second.array()+=margin;
        }
        std::vector<Eigen::Vector3d> local;
        const auto stats=point_index_.query_aabb(bounds.first,bounds.second,[&](const Eigen::Vector3f &point) {
          if (local.size()<=max_local_points_) local.push_back(cloud_to_source*point.cast<double>());
        });
        if (stats.accepted_point_num>max_local_points_) candidate.reason="local_point_budget";
        else {
          const auto result=grasp_refinement::refine(local,source_pose,config_);
          candidate.refined_pose=to_pose(result.pose);
          candidate.left_contact=to_point(result.left); candidate.right_contact=to_point(result.right);
          candidate.contact_width=result.contact_width; candidate.opening_width=result.opening_width;
          candidate.observed_width=result.observed_width;
          candidate.left_support_num=result.left_support_num; candidate.right_support_num=result.right_support_num;
          candidate.has_contact_pair=result.has_contact_pair; candidate.has_gripper_check=result.has_gripper_check;
          candidate.has_observed_collision=result.has_observed_collision; candidate.reason=result.reason;
          if (result.has_contact_pair && !result.has_observed_collision) {
            try { solve_joints(candidate,result.pose,local); }
            catch (const tf2::TransformException &) { candidate.reason="root_transform_unavailable"; }
          }
        }
      } catch (const std::exception &error) { candidate.reason=error.what(); }
      out.candidates.push_back(std::move(candidate));
    }
    out.update_ms=std::chrono::duration<double,std::milli>(steady_clock::now()-begin).count();
    output_->publish(out);
    publish_markers(out);
    std::map<std::string,std::size_t> reasons;
    std::size_t contact_num=0,joint_num=0;
    for (const auto &candidate:out.candidates) {
      ++reasons[candidate.reason];
      contact_num+=candidate.has_contact_pair; joint_num+=candidate.has_joint_solution;
    }
    std::ostringstream status;
    status<<"candidates="<<out.candidates.size()<<" contacts="<<contact_num<<" IK="<<joint_num<<" |";
    if (out.candidates.empty()) status<<" no_candidates";
    for (const auto &entry:reasons) status<<' '<<entry.first<<'='<<entry.second;
    status_=status.str();
    RCLCPP_DEBUG(get_logger(),"Refine: %.2f ms Count: source=%zu",out.update_ms,out.candidates.size());
  }

  void publish_markers(const output_array &out)
  {
    using marker=visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;
    marker clear;
    clear.header=out.header; clear.action=marker::DELETEALL;
    array.markers.push_back(clear);
    for (std::size_t idx=0;idx<out.candidates.size();++idx) {
      const auto &candidate=out.candidates[idx];
      const double width=candidate.has_contact_pair ? candidate.contact_width:candidate.observed_width;
      const bool has_width=std::isfinite(width) && width>0;
      Eigen::Isometry3d pose;
      try {
        pose=to_transform(has_width ? candidate.refined_pose:candidate.source_pose);
        if (!has_width)
          pose.linear()*=Eigen::AngleAxisd(config_.tcp_rotation_x_deg*3.14159265358979323846/180,
            Eigen::Vector3d::UnitX()).toRotationMatrix();
      } catch (const std::invalid_argument &) { continue; }
      marker line;
      line.header=out.header; line.ns="refined_gripper"; line.id=static_cast<int>(idx);
      line.type=marker::LINE_LIST; line.action=marker::ADD; line.pose.orientation.w=1;
      line.pose=to_pose(pose);
      line.scale.x=0.003; line.color.a=1;
      line.color.r=0.1F; line.color.g=0.9F; line.color.b=candidate.has_joint_solution ? 0.1F:0.9F;
      if (!candidate.has_contact_pair) { line.color.r=1.0F; line.color.g=0.65F; line.color.b=0.1F; }
      if (candidate.has_observed_collision || candidate.reason=="width_out_of_range") {
        line.color.r=1.0F; line.color.g=0.15F; line.color.b=0.1F;
      }
      if (!has_width) line.color.r=line.color.g=line.color.b=0.6F;
      if (has_width) {
        // TCPのY軸が把持幅、X軸が指内面の幅、+Zが指先から基部への方向
        const auto segment=[&](const Eigen::Vector3d &start,const Eigen::Vector3d &end) {
          const int num=candidate.has_contact_pair ? 1:12;
          for (int part=0;part<num;part+=2) {
            line.points.push_back(to_point(start+(end-start)*(static_cast<double>(part)/num)));
            line.points.push_back(to_point(start+(end-start)*(static_cast<double>(part+1)/num)));
          }
        };
        const double half_span=0.5*config_.finger_span,depth=config_.finger_length;
        for (double y:{-0.5*width,0.5*width}) {
          segment({-half_span,y,0},{half_span,y,0});
          segment({-half_span,y,depth},{half_span,y,depth});
          segment({-half_span,y,0},{-half_span,y,depth});
          segment({half_span,y,0},{half_span,y,depth});
        }
        segment({0,-0.5*width,depth},{0,0.5*width,depth});
        segment({0,-0.5*width,0},{0,0.5*width,0});
        array.markers.push_back(line);
      }
      // 実TCPの-Zへ進入する矢印。未計算時は元候補の向きだけを灰色表示
      line.ns="refined_approach"; line.type=marker::ARROW;
      line.scale.x=0.003; line.scale.y=0.008; line.scale.z=0.015;
      line.points={to_point({0,0,config_.finger_length+config_.approach_length}),to_point({0,0,0})};
      array.markers.push_back(line);
    }
    markers_->publish(array);
  }

  grasp_refinement::options config_;
  double max_input_age_sec_,max_stamp_diff_sec_,max_position_error_,max_orientation_error_deg_,max_seed_dist_,max_joint_change_deg_,closed_width_;
  std::size_t max_candidates_,max_cloud_points_,max_local_points_,max_ik_seeds_,max_ik_iter_;
  std::string root_frame_,tcp_frame_,gripper_joint_,gripper_mimic_joint_;
  std::string status_,reported_status_;
  bool enable_ik_,has_update_=false,is_stale_output_=false;
  std::unique_ptr<kinematics::KinematicChain> chain_;
  std::vector<std::string> joint_names_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  source_array::ConstSharedPtr source_;
  seed_array::ConstSharedPtr seeds_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_,indexed_cloud_;
  steady_clock::time_point source_received_,cloud_received_,status_reported_;
  robot_sim::bridge::world_point_bucket_index point_index_{0.05};
  rclcpp::Subscription<source_array>::SharedPtr candidates_sub_;
  rclcpp::Subscription<seed_array>::SharedPtr seeds_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<output_array>::SharedPtr output_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // 並列評価ノード

int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<grasp_candidate_refiner>()); }
  catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("grasp_candidate_refiner"),"%s",error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
  return 0;
}
