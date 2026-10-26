#include "uld_occ_observer/observer.hpp"
#include <openssl/evp.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <set>
#include <locale>
#include <cmath>

namespace uld_occ_observer {
namespace {
using Clock = std::chrono::steady_clock;
double milliseconds(Clock::time_point start, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end-start).count();
}
std::string fingerprint(const OpticalObservation& observation) {
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(17);
  const auto& id = observation.identity;
  stream << id.stamp_ns << ':' << id.epoch << ':' << std::quoted(id.optical_frame)
         << std::quoted(id.clock_domain) << std::quoted(id.geometry_hash);
  for (double value : observation.uld_from_optical.rotation) stream << ':' << value;
  const auto& t = observation.uld_from_optical.translation;
  stream << ':' << t.x << ':' << t.y << ':' << t.z << ':' << observation.uld_from_optical.stamp_ns;
  for (const auto& point : observation.points_m)
    stream << ';' << point.x << ',' << point.y << ',' << point.z;
  return sha256(stream.str());
}
CellIndex index_from_key(const octomap::OcTreeKey& key) {
  return {static_cast<int>(key[0])-32768, static_cast<int>(key[1])-32768,
          static_cast<int>(key[2])-32768};
}
octomap::OcTreeKey key_from_index(CellIndex i) {
  return {static_cast<octomap::key_type>(i.x+32768),
          static_cast<octomap::key_type>(i.y+32768),
          static_cast<octomap::key_type>(i.z+32768)};
}
}
std::string sha256(const std::string& bytes) {
  unsigned char digest[EVP_MAX_MD_SIZE]; unsigned int size{};
  if (EVP_Digest(bytes.data(), bytes.size(), digest, &size, EVP_sha256(), nullptr) != 1)
    throw Error("HASH_FAILURE");
  std::ostringstream stream;
  for (unsigned int i = 0; i < size; ++i)
    stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(digest[i]);
  return stream.str();
}
ObservationStore::ObservationStore(GeometryDescriptor descriptor, double resolution,
    DepthZConfig depth, std::string clock_domain, std::string config_digest)
    : domain_(std::make_shared<const UldDomain>(std::move(descriptor), resolution)),
      preprocessor_(depth), clock_domain_(std::move(clock_domain)),
      config_digest_(std::move(config_digest)), tree_(std::make_unique<octomap::OcTree>(resolution)) {
  if (clock_domain_.empty() || config_digest_.empty()) throw Error("INVALID_CONFIG_IDENTITY");
  configure_tree();
}
void ObservationStore::configure_tree() {
  tree_->setProbHit(.7); tree_->setProbMiss(.4); tree_->setOccupancyThres(.5);
  tree_->setClampingThresMin(.1192); tree_->setClampingThresMax(.971);
}
SnapshotIdentity ObservationStore::identity_unlocked() const {
  return {epoch_, map_revision_, observed_revision_, last_integrated_stamp_ns_,
          domain_->descriptor().geometry_hash, config_digest_, clock_domain_};
}

UpdateReceipt ObservationStore::integrate(const OpticalObservation& observation) {
  const auto start = Clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  ++received_;
  UpdateReceipt receipt;
  receipt.sensor_id = observation.identity.sensor_id;
  receipt.observation_id = observation.identity.observation_id;
  receipt.input_stamp_ns = observation.identity.stamp_ns;
  receipt.raw_points = observation.points_m.size();
  try {
    const auto& id = observation.identity;
    if (id.epoch != epoch_) throw Error("EPOCH_MISMATCH");
    if (id.geometry_hash != domain_->descriptor().geometry_hash) throw Error("GEOMETRY_MISMATCH");
    if (id.clock_domain != clock_domain_) throw Error("CLOCK_DOMAIN_MISMATCH");
    observation.uld_from_optical.validate(id.stamp_ns);
    const auto digest = fingerprint(observation);
    const auto dedup_key = std::make_pair(id.sensor_id, id.observation_id);
    const auto prior = fingerprints_.find(dedup_key);
    if (prior != fingerprints_.end()) {
      if (prior->second != digest) throw Error("CONFLICT");
      ++duplicates_; receipt.status = "ALREADY_PROCESSED";
      receipt.identity = identity_unlocked();
      receipt.total_ms = milliseconds(start, Clock::now());
      return receipt;
    }
    const auto last = sensor_stamps_.find(id.sensor_id);
    if (last != sensor_stamps_.end() && id.stamp_ns <= last->second) throw Error("OUT_OF_ORDER");
    const auto filtered = preprocessor_.prepare(observation.points_m);
    const auto frame = UldFrameAdapter::transform(id, filtered, observation.uld_from_optical);
    receipt.accepted_points = frame.points.size();
    const auto prepared_points = Clock::now();
    receipt.preprocessing_ms = milliseconds(start, prepared_points);
    if (frame.points.empty()) throw Error("NO_VALID_POINTS");

    std::set<CellIndex> hits, frees;
    const auto origin = domain_->octomap_coordinates(frame.origin);
    octomap::OcTreeKey origin_key;
    if (!tree_->coordToKeyChecked(origin.x, origin.y, origin.z, origin_key))
      throw Error("ORIGIN_OUT_OF_KEY_RANGE");
    octomap::KeyRay ray;
    for (const auto& endpoint : frame.points) {
      const auto p = domain_->octomap_coordinates(endpoint);
      octomap::OcTreeKey endpoint_key;
      if (!tree_->coordToKeyChecked(p.x, p.y, p.z, endpoint_key))
        throw Error("ENDPOINT_OUT_OF_KEY_RANGE");
      // No maxrange: traverse the real ray. Domain restrictions apply to keys only.
      if (!tree_->computeRayKeys(octomap::point3d(origin.x, origin.y, origin.z),
                                octomap::point3d(p.x, p.y, p.z), ray))
        throw Error("RAY_PREPARATION_FAILED");
      for (const auto& key : ray) {
        const auto index = index_from_key(key);
        if (domain_->active(index)) frees.insert(index);
      }
      if (domain_->contains(endpoint)) {
        const auto index = index_from_key(endpoint_key);
        if (domain_->active(index)) hits.insert(index);
      }
    }
    for (const auto& hit : hits) frees.erase(hit);
    receipt.free_keys = frees.size(); receipt.hit_keys = hits.size();
    const auto prepared = Clock::now();
    receipt.preparation_ms = milliseconds(prepared_points, prepared);
    std::map<CellIndex, std::shared_ptr<CellBlock>> modified;
    auto update = [&](CellIndex index, bool hit) {
      const auto key = key_from_index(index);
      const auto* before = tree_->search(key);
      const bool existed = before != nullptr;
      const float old = existed ? before->getLogOdds() : 0;
      const bool old_occupied = existed && tree_->isNodeOccupied(before);
      auto* after = tree_->updateNode(key, hit, false);
      const bool occupied = tree_->isNodeOccupied(after);
      if (existed && old == after->getLogOdds()) return;
      ++receipt.changed_keys;
      if (!existed || old_occupied != occupied) ++receipt.occupancy_changed_keys;
      if (!existed) ++known_cells_;
      const auto block_index = Snapshot::block_index(index);
      auto& block = modified[block_index];
      if (!block) {
        const auto existing = blocks_.find(block_index);
        block = existing == blocks_.end() ? std::make_shared<CellBlock>() :
                std::make_shared<CellBlock>(*existing->second);
      }
      (*block)[index] = {after->getLogOdds(), occupied};
    };
    for (const auto& index : frees) update(index, false);
    for (const auto& index : hits) update(index, true);
    for (auto& entry : modified) blocks_[entry.first] = std::move(entry.second);
    receipt.rebuilt_blocks = modified.size();
    if (receipt.changed_keys) ++map_revision_;
    ++observed_revision_; ++integrated_;
    last_integrated_stamp_ns_ = std::max(last_integrated_stamp_ns_, id.stamp_ns);
    sensor_stamps_[id.sensor_id] = id.stamp_ns;
    fingerprints_[dedup_key] = digest; fingerprint_order_.push_back(dedup_key);
    if (fingerprint_order_.size() > 4096) {
      fingerprints_.erase(fingerprint_order_.front()); fingerprint_order_.pop_front();
    }
    if (receipt.changed_keys) cached_snapshot_.reset();
    else if (cached_snapshot_) cached_snapshot_ = cached_snapshot_->with_identity(identity_unlocked());
    receipt.status = "INTEGRATED";
    receipt.fusion_ms = milliseconds(prepared, Clock::now());
  } catch (const Error& error) {
    ++rejected_; receipt.status = error.what();
  }
  receipt.identity = identity_unlocked();
  receipt.total_ms = milliseconds(start, Clock::now());
  return receipt;
}

std::shared_ptr<const Snapshot> ObservationStore::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_integrated_stamp_ns_) throw Error("NO_DATA");
  if (cached_snapshot_) return cached_snapshot_;
  auto result = std::make_shared<const Snapshot>(identity_unlocked(), domain_, blocks_);
  cached_snapshot_ = result;
  return result;
}
Diagnostics ObservationStore::diagnostics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {identity_unlocked(), tree_->size(), known_cells_, blocks_.size(), tree_->memoryUsage(),
          received_, integrated_, rejected_, duplicates_, full_serializations_};
}
std::uint64_t ObservationStore::reset(std::uint64_t expected_epoch) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (epoch_ != expected_epoch) throw Error("EPOCH_MISMATCH");
  tree_ = std::make_unique<octomap::OcTree>(domain_->resolution()); configure_tree();
  ++epoch_; map_revision_ = observed_revision_ = 0; last_integrated_stamp_ns_ = 0;
  known_cells_ = 0; blocks_.clear(); cached_snapshot_.reset();
  sensor_stamps_.clear(); fingerprints_.clear(); fingerprint_order_.clear();
  return epoch_;
}
Checkpoint ObservationStore::checkpoint() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream stream(std::ios::binary);
  if (!tree_->write(stream)) throw Error("CHECKPOINT_FAILED");
  ++full_serializations_;
  return {identity_unlocked(), stream.str(), sensor_stamps_, fingerprints_, fingerprint_order_};
}
std::uint64_t ObservationStore::restore(const Checkpoint& checkpoint, std::uint64_t expected_epoch) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (expected_epoch != epoch_) throw Error("EPOCH_MISMATCH");
  const auto& id = checkpoint.identity;
  if (id.geometry_hash != domain_->descriptor().geometry_hash || id.config_digest != config_digest_ ||
      id.clock_domain != clock_domain_ || checkpoint.fingerprints.size() > 4096 ||
      checkpoint.fingerprints.size() != checkpoint.fingerprint_order.size() ||
      id.last_integrated_stamp_ns < 0) throw Error("CHECKPOINT_IDENTITY_MISMATCH");
  std::set<std::pair<std::string, std::string>> seen;
  for (const auto& key : checkpoint.fingerprint_order)
    if (!checkpoint.fingerprints.count(key) || !seen.insert(key).second)
      throw Error("INVALID_CHECKPOINT_DEDUP");
  for (const auto& sensor : checkpoint.sensor_stamps)
    if (sensor.first.empty() || sensor.second <= 0 || sensor.second > id.last_integrated_stamp_ns)
      throw Error("INVALID_CHECKPOINT_STAMP");
  std::istringstream stream(checkpoint.tree_bytes, std::ios::binary);
  std::unique_ptr<octomap::AbstractOcTree> raw(octomap::AbstractOcTree::read(stream));
  auto* restored = dynamic_cast<octomap::OcTree*>(raw.get());
  if (!restored || restored->getResolution() != domain_->resolution() || !stream)
    throw Error("INVALID_CHECKPOINT_TREE");
  // Full rebuilding is allowed only for explicit restore, never normal observations.
  std::map<CellIndex, std::shared_ptr<CellBlock>> rebuilt;
  std::size_t known = 0;
  // Expand pruned known leaves to the finest query grid before publishing anything.
  restored->expand();
  for (auto it = restored->begin_leafs(); it != restored->end_leafs(); ++it) {
    const auto index = index_from_key(it.getKey());
    if (!domain_->active(index) || !std::isfinite(it->getLogOdds()) ||
        it->getLogOdds() < octomap::logodds(.1192)-1e-6 ||
        it->getLogOdds() > octomap::logodds(.971)+1e-6)
      throw Error("INVALID_CHECKPOINT_CELL");
    auto& block = rebuilt[Snapshot::block_index(index)];
    if (!block) block = std::make_shared<CellBlock>();
    (*block)[index] = {it->getLogOdds(), it->getLogOdds() >= 0};
    ++known;
  }
  BlockRegistry published;
  for (auto& entry : rebuilt) published.emplace(entry.first, std::move(entry.second));
  raw.release(); tree_.reset(restored); configure_tree();
  blocks_ = std::move(published); known_cells_ = known;
  ++epoch_; map_revision_ = id.map_revision; observed_revision_ = id.observed_revision;
  last_integrated_stamp_ns_ = id.last_integrated_stamp_ns;
  sensor_stamps_ = checkpoint.sensor_stamps;
  // Payload fingerprints contain their original epoch. Restored timestamps still
  // reject stale observations; tokens and old-epoch duplicates are never revived.
  fingerprints_.clear(); fingerprint_order_.clear();
  cached_snapshot_.reset();
  return epoch_;
}
}  // namespace uld_occ_observer
