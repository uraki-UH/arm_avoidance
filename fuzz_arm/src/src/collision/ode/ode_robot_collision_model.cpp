#include "collision/ode/ode_robot_collision_model.hpp"

namespace simulation {

OdeRobotCollisionModel::OdeRobotCollisionModel(
    const RobotModel &model, dWorldID world, dSpaceID space,
    CollisionManager *collision_manager,
    const kinematics::KinematicChain &chain,
    MeshCache *mesh_cache,
    bool use_mesh_collision) // Added use_mesh_collision
    : world_(world), space_(space), collision_manager_(collision_manager),
      model_(model) {

  // ロボットを構築（干渉判定用モデル）
  OdeRobotBuilder builder(world_, space_, collision_manager_, mesh_cache);
  builder.setUseMesh(use_mesh_collision); // Apply toggle setting

  // ロボットのリンクとジョイントを作成。
  // カテゴリビット: ROBOT_PART
  // 衝突ビット: ALL (自己だけでなく環境全体との干渉を検出できるようにする)
  components_ = builder.build(model_, CollisionCategory::ROBOT_PART,
                              CollisionCategory::ALL);

  // link_names_order_ の 0 番目をルートリンクにする（KinematicChain の
  // positions[0] に対応）
  link_names_order_.push_back(model_.getRootLinkName());

  // KinematicChainのリンク順序に合わせて保持
  // chain[0] はベースの位置・姿勢、chain[1] 以降が各リンクに対応
  for (int i = 0; i < chain.getNumJoints(); ++i) {
    link_names_order_.push_back(chain.getLinkName(i));
  }
}

OdeRobotCollisionModel::~OdeRobotCollisionModel() {
  // コンポーネントのクリーンアップは基本クラスやシミュレーション全体で行う想定
}

void OdeRobotCollisionModel::updateBodyPoses(
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
    const std::vector<Eigen::Quaterniond,
                      Eigen::aligned_allocator<Eigen::Quaterniond>>
        &orientations) {
  // KinematicChainから渡される positions/orientations
  // は、各ジョイント（＝リンクの原点）のワールド姿勢。
  // ODEのボディは「重心(COM)」の位置・回転を設定する必要がある。

  // positions[i] が
  // i番目のリンクの原点姿勢に対応すると仮定（KinematicChainの実装に依存）
  // ルートリンク(0番目)から順番に適用
  for (size_t i = 0; i < std::min(positions.size(), link_names_order_.size());
       ++i) {
    const std::string &link_name = link_names_order_[i];
    auto it = components_.find(link_name);
    if (it == components_.end())
      continue;

    dBodyID body = it->second.body_id;
    if (!body)
      continue;

    const LinkProperties *props = model_.getLink(link_name);
    if (!props)
      continue;

    // リンク原点のワールド変換
    Eigen::Isometry3d link_origin_world = Eigen::Isometry3d::Identity();
    link_origin_world.translate(positions[i]);
    link_origin_world.rotate(orientations[i]);

    // ボディ(COM)のワールド変換 = link_origin_world * inertial.origin
    Eigen::Isometry3d body_com_world =
        link_origin_world * props->inertial.origin;

    // ODEに反映
    dVector3 pos;
    dMatrix3 R;
    // ode_robot_builder.cpp にある eigenIsometryToOde
    // と同等の処理（ここでは直接書くか、ヘルパーを呼ぶ）
    pos[0] = body_com_world.translation().x();
    pos[1] = body_com_world.translation().y();
    pos[2] = body_com_world.translation().z();

    Eigen::Matrix3d rot = body_com_world.rotation();
    R[0] = rot(0, 0);
    R[1] = rot(1, 0);
    R[2] = rot(2, 0);
    R[3] = 0;
    R[4] = rot(0, 1);
    R[5] = rot(1, 1);
    R[6] = rot(2, 1);
    R[7] = 0;
    R[8] = rot(0, 2);
    R[9] = rot(1, 2);
    R[10] = rot(2, 2);
    R[11] = 0;

    dBodySetPosition(body, pos[0], pos[1], pos[2]);
    dBodySetRotation(body, R);
  }
}

bool OdeRobotCollisionModel::updateCollisionStatus() {
  if (!collision_manager_)
    return false;

  collision_manager_->clearContacts();
  dJointGroupEmpty(collision_manager_->getContactGroupID());

  // ODEの空間内での衝突判定を実行
  // 接触情報は collision_manager 内部に蓄積される
  dSpaceCollide(space_, collision_manager_, &CollisionManager::nearCallback);

  // 1つでも接触があれば true を返す（詳細は hasSelfCollision /
  // hasEnvironmentCollision で確認） 厳密にチェックするのはコストがかかるので、
  // activeContacts_ が空でなければ「何かが起きている」とする
  // （ただし CollisionManager 側で activeContacts_ の空チェックAPIがないので、
  // hasSelfCollison 等で確認する設計とする）
  return true;
}

bool OdeRobotCollisionModel::checkCollision() {
  // ISelfCollisionChecker インターフェース実装: 自己干渉のみを返す
  updateCollisionStatus();
  return hasSelfCollision();
}

bool OdeRobotCollisionModel::hasSelfCollision() const {
  if (!collision_manager_)
    return false;

  // 全リンクについてチェック
  for (const auto &link_name : link_names_order_) {
    std::string geom_name =
        link_name + "_geom_0"; // 簡易実装: 第0ジオメトリのみ
    auto contacts = collision_manager_->getContacts(geom_name);

    for (const auto &contact_name : contacts) {
      // 接触相手が「自分のリンク」であれば自己干渉
      // （※CollisionManager 内で隣接リンク除外などは処理済み前提）
      // checking if contact_name exists in our components map
      if (components_.find(contact_name) != components_.end() ||
          // geom_name format check (link_name + "_geom_")
          contact_name.find("_geom_") != std::string::npos) {
        // 名前だけでの判定は弱いが、components_ キーと比較するのが確実
        // ただし _geom_X サフィックスを考慮する必要あり。
        // ここでは簡易的に「相手もロボットパーツ（components_にある名前を含んでいる）」なら自己干渉とする
        // 厳密には CollisionManager にカテゴリ情報を問い合わせるのが良いが、
        // 現状のAPIでは名前ベースで判断する。
        // components_ のキー（リンク名）が contact_name
        // に含まれていればロボットパーツとみなす
        for (const auto &comp_pair : components_) {
          const std::string &robot_link = comp_pair.first;
          if (contact_name.compare(0, robot_link.length(), robot_link) == 0) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool OdeRobotCollisionModel::hasEnvironmentCollision() const {
  if (!collision_manager_)
    return false;

  for (const auto &link_name : link_names_order_) {
    std::string geom_name = link_name + "_geom_0";
    auto contacts = collision_manager_->getContacts(geom_name);

    for (const auto &contact_name : contacts) {
      // 相手がロボットパーツでなければ環境との干渉
      // (sc_ground など)
      bool is_robot_part = false;
      for (const auto &comp_pair : components_) {
        const std::string &robot_link = comp_pair.first;
        if (contact_name.compare(0, robot_link.length(), robot_link) == 0) {
          is_robot_part = true;
          break;
        }
      }

      if (!is_robot_part) {
        // ロボット以外（地面や障害物）と接触している
        return true;
      }
    }
  }
  return false;
}

bool OdeRobotCollisionModel::isPathColliding(
    [[maybe_unused]] const Eigen::VectorXf &q1,
    [[maybe_unused]] const Eigen::VectorXf &q2, [[maybe_unused]] int steps) {
  // 未使用
  return false;
}

} // namespace simulation
