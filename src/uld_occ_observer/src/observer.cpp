#include "uld_occ_observer/observer.hpp"
#include "uld_occ_observer/resource_usage.hpp"
#include "uld_occ_observer/interface_trace.hpp"
#include "uld_occ_observer/block_storage.hpp"
#include <openssl/evp.h>
#include <optional>
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
    DepthZConfig depth, std::string clock_domain, std::string config_digest,
    bool discretize_endpoints)
    : domain_(std::make_shared<const UldDomain>(std::move(descriptor), resolution)),
      preprocessor_(depth), clock_domain_(std::move(clock_domain)),
      config_digest_(std::move(config_digest)), discretize_endpoints_(discretize_endpoints),
      grid_(resolution) {
  if (clock_domain_.empty() || config_digest_.empty()) throw Error("INVALID_CONFIG_IDENTITY");
}
SnapshotIdentity ObservationStore::identity_unlocked() const {
  return {epoch_, map_revision_, observed_revision_, last_integrated_stamp_ns_,
          domain_->descriptor().geometry_hash, config_digest_, clock_domain_};
}

UpdateReceipt ObservationStore::integrate(const OpticalObservation& observation, ProfileMode profile_mode) {
  const auto start = Clock::now();
  UpdateReceipt receipt;
  OccupancyDiagnostic diagnostic(profile_mode);
  auto publish = [&] {
    receipt.identity = identity_unlocked();
    receipt.total_ms = milliseconds(start, Clock::now());
    diagnostic.finish(receipt.total_ms);
    receipt.profile = diagnostic.report();
  };
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  { auto span = diagnostic.span(ProfileStage::LockWait); lock.lock(); }
  if (lock_probe_) lock_probe_();
  ++received_;
  receipt.sensor_id = observation.identity.sensor_id;
  receipt.observation_id = observation.identity.observation_id;
  receipt.input_stamp_ns = observation.identity.stamp_ns;
  receipt.raw_points = observation.points_m.size();
  interface_trace_event("integrate_start", receipt.observation_id.c_str(), "", "writer");
  struct TraceEnd {
    const UpdateReceipt* receipt;
    ~TraceEnd() {
      interface_trace_event("integrate_end", receipt->observation_id.c_str(),
                            receipt->status.c_str(), "writer");
    }
  } trace_end{&receipt};
  try {
    const auto& id = observation.identity;
    { auto span = diagnostic.span(ProfileStage::Validation);
      if (id.epoch != epoch_) throw Error("EPOCH_MISMATCH");
      if (id.geometry_hash != domain_->descriptor().geometry_hash) throw Error("GEOMETRY_MISMATCH");
      if (id.clock_domain != clock_domain_) throw Error("CLOCK_DOMAIN_MISMATCH");
      observation.uld_from_optical.validate(id.stamp_ns);
    }
    std::string digest;
    { auto span = diagnostic.span(ProfileStage::Fingerprint); digest = fingerprint(observation); }
    const auto dedup_key = std::make_pair(id.sensor_id, id.observation_id);
    bool duplicate = false;
    { auto span = diagnostic.span(ProfileStage::Dedup);
      const auto prior = fingerprints_.find(dedup_key);
      if (prior != fingerprints_.end()) {
        if (prior->second != digest) throw Error("CONFLICT");
        duplicate = true;
      }
      if (!duplicate) {
        const auto last = sensor_stamps_.find(id.sensor_id);
        if (last != sensor_stamps_.end() && id.stamp_ns <= last->second) throw Error("OUT_OF_ORDER");
      }
    }
    if (duplicate) {
      ++duplicates_; receipt.status = "ALREADY_PROCESSED";
      publish();
      return receipt;
    }
    FilterResult filtered;
    { auto span = diagnostic.span(ProfileStage::DepthFilter);
      filtered = preprocessor_.prepare(observation.points_m); }
    UldObservation frame;
    { auto span = diagnostic.span(ProfileStage::Transform);
      frame = UldFrameAdapter::transform(id, filtered, observation.uld_from_optical); }
    receipt.accepted_points = frame.points.size();
    const auto prepared_points = Clock::now();
    receipt.preprocessing_ms = milliseconds(start, prepared_points);
    if (frame.points.empty()) throw Error("NO_VALID_POINTS");

    OccupancyScan scan;
    grid_.collect(scan, frame.origin, frame.points, *domain_,
                  OccupancyScanOptions{discretize_endpoints_}, diagnostic);
    receipt.free_keys = scan.free_cells.size();
    receipt.hit_keys = scan.occupied_cells.size();
    const auto prepared = Clock::now();
    receipt.preparation_ms = milliseconds(prepared_points, prepared);
    { auto fusion_scope = diagnostic.span(ProfileStage::Fusion);
      // The mutation vector and the batch outlive both coarse intervals. Their
      // destructors stay inside ProfileStage::Fusion and outside ApplyBatch and
      // BlockBatch. receipt.fusion_ms is sampled later and still includes
      // Finalization; this scope does not redefine that field.
      std::vector<OccupancyMutation> mutations;
      std::optional<BlockUpdateBatch> batch;
      {
        auto apply_batch = diagnostic.fusion_span(FusionPhase::ApplyBatch);
        mutations = grid_.apply(scan, diagnostic);
      }
      if (diagnostic.enabled()) {
        diagnostic.record_mutation_storage(mutations.size(), mutations.capacity(),
                                           sizeof(OccupancyMutation));
      }
      {
        auto block_batch = diagnostic.fusion_span(FusionPhase::BlockBatch);
        batch.emplace(BlockLayout(domain_->shape()), blocks_, diagnostic);
        for (const auto& change : mutations) {
          auto block_span = diagnostic.detail(ProfileStage::BlockMaintenance);
          ++receipt.changed_keys;
          if (!change.existed || change.old_occupied != change.occupied)
            ++receipt.occupancy_changed_keys;
          if (!change.existed) ++known_cells_;
          batch->write(change.index, CellValue{change.new_log_odds, change.occupied});
        }
        batch->commit(blocks_);
        receipt.rebuilt_blocks = batch->touched_blocks();
      }
    }
    { auto final_span = diagnostic.span(ProfileStage::Finalization);
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
    }
    receipt.fusion_ms = milliseconds(prepared, Clock::now());
    // Release stays inside total_ms. Moving it past publish() would drop it from the receipt clock.
    scan.free_cells.clear();
    scan.occupied_cells.clear();
  } catch (const Error& error) {
    ++rejected_; receipt.status = error.what();
  }
  publish();
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
std::shared_ptr<const Snapshot> ObservationStore::try_snapshot() const {
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return nullptr;
  if (!last_integrated_stamp_ns_) throw Error("NO_DATA");
  if (cached_snapshot_) return cached_snapshot_;
  auto result = std::make_shared<const Snapshot>(identity_unlocked(), domain_, blocks_);
  cached_snapshot_ = result;
  return result;
}
std::shared_ptr<const Snapshot> ObservationStore::retained_snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_snapshot_;
}
std::uint64_t ObservationStore::published_epoch() const {
  return published_epoch_.load(std::memory_order_acquire);
}
void ObservationStore::collect_blocks(std::vector<std::shared_ptr<const CellBlock>>& out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& entry : blocks_) out.push_back(entry.second);
}
std::uint64_t ObservationStore::directory_entries() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return blocks_.size();
}
void ObservationStore::set_lock_probe_for_test(std::function<void()> probe) {
  std::lock_guard<std::mutex> lock(mutex_);
  lock_probe_ = std::move(probe);
}
Diagnostics ObservationStore::diagnostics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {identity_unlocked(), grid_.nodeCount(), known_cells_, blocks_.size(), grid_.memoryUsage(),
          received_, integrated_, rejected_, duplicates_, full_serializations_};
}
std::uint64_t ObservationStore::reset(std::uint64_t expected_epoch) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (epoch_ != expected_epoch) throw Error("EPOCH_MISMATCH");
  // Occupancy accounting: the registry nodes are released here; the blocks
  // they referenced release themselves wherever their last owner drops.
  AllocationTotals::instance().subtract(
      "block_directory_nodes",
      static_cast<std::uint64_t>(blocks_.size())
          * map_node_bytes<BlockRegistry::value_type>());
  grid_.reset();
  ++epoch_; map_revision_ = observed_revision_ = 0; last_integrated_stamp_ns_ = 0;
  published_epoch_.store(epoch_, std::memory_order_release);
  known_cells_ = 0; blocks_.clear(); cached_snapshot_.reset();
  sensor_stamps_.clear(); fingerprints_.clear(); fingerprint_order_.clear();
  return epoch_;
}
Checkpoint ObservationStore::checkpoint() const {
  std::lock_guard<std::mutex> lock(mutex_);
  ++full_serializations_;
  return {identity_unlocked(), grid_.serialize(), sensor_stamps_, fingerprints_, fingerprint_order_};
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
  // Read, expand, and validate on a temporary grid. A bad leaf must not replace live state.
  auto candidate = OccupancyGrid::fromCheckpointBytes(checkpoint.tree_bytes, domain_->resolution());
  candidate.validateLeaves(*domain_);
  BlockRegistry published;
  OccupancyDiagnostic rebuild(ProfileMode::Off);
  BlockUpdateBatch batch(BlockLayout(domain_->shape()), published, rebuild);
  std::size_t known = 0;
  candidate.forEachLeaf([&](CellIndex index, float log_odds) {
    batch.write(index, CellValue{log_odds, log_odds >= 0.f});
    ++known;
  });
  batch.commit(published);
  grid_ = std::move(candidate);
  blocks_ = std::move(published);
  known_cells_ = known;
  ++epoch_; map_revision_ = id.map_revision; observed_revision_ = id.observed_revision;
  published_epoch_.store(epoch_, std::memory_order_release);
  last_integrated_stamp_ns_ = id.last_integrated_stamp_ns;
  sensor_stamps_ = checkpoint.sensor_stamps;
  // Payload fingerprints contain their original epoch. Restored timestamps still
  // reject stale observations; tokens and old-epoch duplicates are never revived.
  fingerprints_.clear(); fingerprint_order_.clear();
  cached_snapshot_.reset();
  return epoch_;
}
}  // namespace uld_occ_observer
