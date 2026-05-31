#pragma once

#include <grasping_system/core/grasp_object_model.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace grasping_system::core
{

class GraspObjectRegistry
{
public:
  bool has(const std::string &object_id) const
  {
    return objects_.find(object_id) != objects_.end();
  }

  void upsert(GraspObjectModel model)
  {
    objects_[model.object.object_id] = std::move(model);
  }

  const GraspObjectModel *find(const std::string &object_id) const
  {
    const auto it = objects_.find(object_id);
    return it == objects_.end() ? nullptr : &it->second;
  }

  GraspObjectModel *find(const std::string &object_id)
  {
    const auto it = objects_.find(object_id);
    return it == objects_.end() ? nullptr : &it->second;
  }

  void erase(const std::string &object_id)
  {
    objects_.erase(object_id);
  }

  void clear()
  {
    objects_.clear();
  }

private:
  std::unordered_map<std::string, GraspObjectModel> objects_;
};

}  // namespace grasping_system::core
