#include <point_cloud_store.hpp>

namespace voxel_idx
{
bool point_registration_spec::operator==(const point_registration_spec &other) const
{
  return size == other.size && min_pos == other.min_pos && max_pos == other.max_pos &&
    num_cells == other.num_cells && target_frame == other.target_frame;
}

point_registration_query::point_registration_query(point_registration_spec spec)
: spec_(std::move(spec)), inverse_size_(1.f / spec_.size), max_cell_num_(1)
{
  if (!std::isfinite(spec_.size) || spec_.size <= 0 || !std::isfinite(inverse_size_) ||
    spec_.target_frame.empty()) {throw std::invalid_argument("登録格子条件の不正値");}
  std::uint64_t num_cells = 1;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const float span = (spec_.max_pos[axis] - spec_.min_pos[axis]) * inverse_size_;
    if (!std::isfinite(spec_.min_pos[axis]) || !std::isfinite(spec_.max_pos[axis]) ||
      !std::isfinite(span) || span < 1 || span >= static_cast<float>(INT32_MAX) ||
      static_cast<std::uint32_t>(span) != spec_.num_cells[axis])
    {throw std::invalid_argument("登録格子の範囲・寸法不一致");}
    if (num_cells > (UINT32_MAX - 1ULL) / spec_.num_cells[axis])
    {throw std::invalid_argument("登録格子のセル数過大");}
    num_cells *= spec_.num_cells[axis];
  }
  max_cell_num_ = static_cast<std::uint32_t>(num_cells);
}

std::shared_ptr<const point_registration> point_registration_query::read(
  const std::shared_ptr<const point_frame> &frame, std::uint32_t source_num,
  const std::vector<std::uint32_t> &source_indices, const float *xyz,
  const std::array<float, 7> &source_pose)
{
  if (!frame || (!xyz && !source_indices.empty()) || source_indices.size() > UINT32_MAX)
  {throw std::invalid_argument("登録対象snapshot・入力配列の不正値");}
  for (const auto value : source_pose) {
    if (!std::isfinite(value)) {throw std::invalid_argument("登録格子の座標変換不正");}
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const bool is_same_frame = frame_.lock() == frame;
  if (is_same_frame && source_pose_ != source_pose)
  {throw std::invalid_argument("同一snapshot・座標系の取得時TF不一致");}
  if (is_same_frame && cell_slots_.size() != source_num)
  {throw std::invalid_argument("同一snapshotの元点数不一致");}
  if (is_same_frame && has_result_ && selected_indices_ == source_indices) {return current_;}
  has_result_ = false;
  if (!is_same_frame) {
    cell_slots_.resize(source_num);
    has_cell_.assign(source_num, 0);
    num_registered_points_ = 0;
    frame_ = frame;
    source_pose_ = source_pose;
  }
  if (!current_ || !current_.unique()) {
    current_.swap(spare_);
    if (!current_ || !current_.unique()) {current_ = std::make_shared<point_registration>();}
  }
  auto &points = current_->points;
  points.clear(); points.reserve(source_indices.size());
  for (std::size_t idx = 0; idx < source_indices.size(); ++idx) {
    const auto source_idx = source_indices[idx];
    if (source_idx >= source_num) {throw std::invalid_argument("登録元点番号の範囲外");}
    if (!has_cell_[source_idx]) {
      cell_slots_[source_idx] = cell_idx(xyz + 3 * idx);
      has_cell_[source_idx] = 1;
      ++num_registered_points_;
    }
    if (cell_slots_[source_idx] != UINT32_MAX) {
      points.push_back((std::uint64_t{cell_slots_[source_idx]} << 32) | idx);
    }
  }
  // セル番号だけの4段安定基数ソート。代表元点の入力順保持、比較ソートなし
  sort_buffer_.resize(points.size());
  for (unsigned shift = 32; shift < 64; shift += 8) {
    std::array<std::size_t, 256> offsets{};
    for (const auto point : points) {++offsets[(point >> shift) & 255];}
    std::size_t offset = 0;
    for (auto &count : offsets) {const auto next = offset + count; count = offset; offset = next;}
    for (const auto point : points) {sort_buffer_[offsets[(point >> shift) & 255]++] = point;}
    points.swap(sort_buffer_);
  }
  selected_indices_ = source_indices;
  has_result_ = true;
  return current_;
}

std::size_t point_registration_query::num_registered_points() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return num_registered_points_;
}

std::shared_ptr<point_registration_query> point_frame_channel::registration_query(const point_registration_spec &spec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = registration_queries_.begin(); it != registration_queries_.end();) {
    if (const auto query = it->lock()) {
      if (query->spec() == spec) {return query;}
      ++it;
    } else {it = registration_queries_.erase(it);}
  }
  auto result = std::make_shared<point_registration_query>(spec);
  registration_queries_.push_back(result);
  return result;
}

bool point_cell_spec::operator==(const point_cell_spec &other) const
{
  return size == other.size && origin == other.origin && min_corner == other.min_corner &&
    max_corner == other.max_corner && enable_exclusion == other.enable_exclusion &&
    exclude_min == other.exclude_min && exclude_max == other.exclude_max &&
    max_dense_voxel_num == other.max_dense_voxel_num;
}

point_cell_counts::point_cell_counts(point_cell_spec spec) : spec_(std::move(spec))
{
  if (!spec_.size.allFinite() || (spec_.size.array() <= 0).any() || !spec_.origin.allFinite() ||
    !spec_.min_corner.allFinite() || !spec_.max_corner.allFinite() ||
    (spec_.min_corner.array() > spec_.max_corner.array()).any() ||
    (spec_.enable_exclusion && (!spec_.exclude_min.allFinite() || !spec_.exclude_max.allFinite() ||
    (spec_.exclude_min.array() > spec_.exclude_max.array()).any())))
  {
    throw std::invalid_argument("セル集計条件の不正値");
  }
  min_key_ = spec_.key(spec_.min_corner); max_key_ = spec_.key(spec_.max_corner);
  const auto span = [](std::int32_t lo, std::int32_t hi) {
    return static_cast<std::uint64_t>(static_cast<std::int64_t>(hi)-lo+1);
  };
  const auto nx = span(min_key_.x, max_key_.x), ny = span(min_key_.y, max_key_.y);
  const auto nz = span(min_key_.z, max_key_.z);
  const auto limit = spec_.max_dense_voxel_num;
  if (nx <= limit && ny <= limit/nx && nz <= limit/(nx*ny)) {
    num_x_ = nx; num_y_ = ny;
    dense_lookup_.resize(static_cast<std::size_t>(nx*ny*nz), 0U);
  }
}

void point_cell_counts::begin_frame()
{
  if (has_dense_lookup()) {
    for (const auto &cell : cells_) {dense_lookup_[dense_idx(cell.key)] = 0U;}
  } else {sparse_lookup_.clear();}
  cells_.clear();
}

std::size_t point_cell_counts::find(const world_bucket_key &key) const
{
  if (key.x < min_key_.x || key.x > max_key_.x || key.y < min_key_.y || key.y > max_key_.y ||
    key.z < min_key_.z || key.z > max_key_.z) {return no_cell;}
  std::uint32_t slot = 0;
  if (has_dense_lookup()) {slot = dense_lookup_[dense_idx(key)];}
  else {
    const auto found = sparse_lookup_.find(key);
    if (found != sparse_lookup_.end()) {slot = found->second;}
  }
  return slot ? static_cast<std::size_t>(slot-1) : no_cell;
}

std::shared_ptr<const point_cell_counts> point_cell_query::read(const std::shared_ptr<const point_frame> &frame)
{
  if (!frame || !frame->point_idx) {throw std::invalid_argument("集計対象snapshotなし");}
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.lock() == frame) {return current_;}
  frame_.reset();
  if (!current_ || !current_.unique()) {
    current_.swap(spare_);
    if (!current_ || !current_.unique()) {current_ = std::make_shared<point_cell_counts>(spec_);}
  }
  current_->begin_frame();
  frame->point_idx->query_aabb(spec_.min_corner, spec_.max_corner,
    [&](const Eigen::Vector3f &point) {current_->add_point(point);});
  frame_ = frame;
  return current_;
}

std::shared_ptr<point_cell_query> point_frame_channel::cell_query(const point_cell_spec &spec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = cell_queries_.begin(); it != cell_queries_.end();) {
    if (const auto query = it->lock()) {
      if (query->spec() == spec) {return query;}
      ++it;
    } else {it = cell_queries_.erase(it);}
  }
  auto result = std::make_shared<point_cell_query>(spec);
  cell_queries_.push_back(result);
  return result;
}

void point_frame_channel::claim_writer(const void *writer)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!writer || (writer_ && writer_ != writer)) {
    throw std::logic_error("共有点群channelへの複数writer登録");
  }
  writer_ = writer;
}

void point_frame_channel::release_writer(const void *writer)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (writer_ == writer) {writer_ = nullptr; frame_.reset();}
}

void point_frame_channel::publish(const void *writer, point_frame frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!writer || writer_ != writer || (!frame.point_idx && !(frame.roi_points && frame.source_owner)) ||
    frame.frame_id.empty()) {
    throw std::logic_error("共有点群のwriterまたはsnapshot不正");
  }
  frame.revision = ++revision_;
  frame_ = std::make_shared<const point_frame>(std::move(frame));
}

std::shared_ptr<const point_frame> point_frame_channel::latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_;
}

std::shared_ptr<point_frame_channel> shared_point_frames(const std::string &name)
{
  if (name.empty()) {throw std::invalid_argument("空の共有点群channel名");}
  static std::mutex mutex;
  static std::unordered_map<std::string, std::weak_ptr<point_frame_channel>> channels;
  std::lock_guard<std::mutex> lock(mutex);
  for (auto it = channels.begin(); it != channels.end();) {
    if (it->second.expired()) {it = channels.erase(it);} else {++it;}
  }
  auto result = channels[name].lock();
  if (!result) {result = std::make_shared<point_frame_channel>(); channels[name] = result;}
  return result;
}
}
