#include "simulation/robot/ode/ode_collision_manager.hpp"

namespace simulation {

CollisionManager::CollisionManager(dWorldID world, dJointGroupID contactGroup)
    : world_(world), contactGroup_(contactGroup) {}

CollisionManager::~CollisionManager() {
  // The geomDataStore_ with unique_ptrs will clean itself up.
}

void CollisionManager::registerGeom(dGeomID geom, const std::string &name,
                                    uint32_t categoryBits, uint32_t collideBits,
                                    double friction, double restitution,
                                    dTriMeshDataID visual_mesh_id,
                                    const Eigen::Vector3d &visual_mesh_center) {
  if (!geom)
    return;

  auto newData = std::make_unique<GeomData>();
  newData->name = name;
  newData->categoryBits = categoryBits;
  newData->collideBits = collideBits;
  newData->friction = friction;
  newData->restitution = restitution;
  newData->visual_mesh_id = visual_mesh_id;
  newData->visual_mesh_center = visual_mesh_center;

  // Store the raw pointer for ODE, but keep ownership in the vector
  dGeomSetData(geom, newData.get());
  geomDataNameMap_[name] = newData.get();
  geomDataStore_.push_back(std::move(newData));
}

bool CollisionManager::updateMaterial(const std::string &name, double friction,
                                      double restitution) {
  auto it = geomDataNameMap_.find(name);
  if (it != geomDataNameMap_.end()) {
    it->second->friction = friction;
    it->second->restitution = restitution;
    return true;
  }
  return false;
}

// Static callback function that redirects to the member function
void CollisionManager::nearCallback(void *data, dGeomID o1, dGeomID o2) {
  if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
    // Collide spaces or geoms in spaces
    dSpaceCollide2(o1, o2, data, &nearCallback);
  } else {
    CollisionManager *self = static_cast<CollisionManager *>(data);
    if (self) {
      self->handleCollision(o1, o2);
    }
  }
}

// The actual collision handling implementation
void CollisionManager::handleCollision(dGeomID o1, dGeomID o2) {
  GeomData *data1 = static_cast<GeomData *>(dGeomGetData(o1));
  GeomData *data2 = static_cast<GeomData *>(dGeomGetData(o2));

  // --- 1. Filtering ---
  // Ignore if either geom is not registered
  if (!data1 || !data2) {
    return;
  }

  // 衝突除外ペアをチェック
  if (shouldSkipCollision(data1->name, data2->name)) {
    return;
  }

  // Filter by category bitmask
  bool can_collide = (data1->categoryBits & data2->collideBits) &&
                     (data2->categoryBits & data1->collideBits);
  if (!can_collide) {
    return;
  }

  // Ignore collisions between bodies connected by a joint (except contact
  // joints)
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);

  if (b1 && b2) {
    // 1. 同一ボディ内のジオメトリ間の衝突を無視
    if (b1 == b2)
      return;

    // 2. ジョイントで接続されたボディ間の衝突を無視
    if (dAreConnectedExcluding(b1, b2, dJointTypeContact))
      return;
  }

  // --- 2. Contact Generation ---
  const int MAX_CONTACTS = 16;
  dContact contact[MAX_CONTACTS];

  for (int i = 0; i < MAX_CONTACTS; i++) {
    // --- 3. Set Surface Properties ---
    // Use more stable parameters, disable bounce for now
    contact[i].surface.mode =
        dContactSoftCFM | dContactSoftERP | dContactApprox1;
    contact[i].surface.mu = (data1->friction + data2->friction) * 0.5;
    if (contact[i].surface.mu < 0)
      contact[i].surface.mu = dInfinity;

    contact[i].surface.soft_cfm = 1e-6; // Even stiffer
    contact[i].surface.soft_erp = 0.4;  // Stronger correction
  }

  int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

  // --- 4. Create Contact Joints ---
  if (numc > 0) {
    for (int i = 0; i < numc; i++) {
      activeContacts_[o1].insert(o2);
      activeContacts_[o2].insert(o1);

      dJointID c = dJointCreateContact(world_, contactGroup_, &contact[i]);
      dJointAttach(c, b1, b2);
    }
  }
}

void CollisionManager::clearContacts() { activeContacts_.clear(); }

std::vector<std::string>
CollisionManager::getContacts(const std::string &name) const {
  std::set<std::string> uniqueContacts;

  // Iterate over all active contact pairs
  for (const auto &[g1, contactSet] : activeContacts_) {
    // If the first geometry's name matches our target name
    if (getGeomName(g1) == name) {
      // Add all touching geometries' names to the list
      for (dGeomID g2 : contactSet) {
        uniqueContacts.insert(getGeomName(g2));
      }
    }
  }

  return std::vector<std::string>(uniqueContacts.begin(), uniqueContacts.end());
}

std::string CollisionManager::getGeomName(dGeomID geom) const {
  void *data = dGeomGetData(geom);
  if (data) {
    GeomData *gData = static_cast<GeomData *>(data);
    return gData->name;
  }
  return "unknown";
}

void CollisionManager::addCollisionExclusion(const std::string &name1,
                                             const std::string &name2) {
  collisionExclusionPairs_.insert({name1, name2});
  collisionExclusionPairs_.insert({name2, name1});
}

bool CollisionManager::shouldSkipCollision(const std::string &name1,
                                           const std::string &name2) const {
  // 名前が完全一致するか、プレフィックスとして一致するかをチェック
  // （例: "link_1_geom_0" の "link_1" 部分をチェック）
  auto check = [&](const std::string &n1, const std::string &n2) {
    if (collisionExclusionPairs_.count({n1, n2}))
      return true;

    // プレフィックスチェック
    for (const auto &pair : collisionExclusionPairs_) {
      const std::string &ex1 = pair.first;
      const std::string &ex2 = pair.second;

      bool match1 = (n1 == ex1 || n1.rfind(ex1 + "_", 0) == 0);
      bool match2 = (n2 == ex2 || n2.rfind(ex2 + "_", 0) == 0);

      if (match1 && match2)
        return true;
    }
    return false;
  };

  return check(name1, name2);
}

} // namespace simulation
