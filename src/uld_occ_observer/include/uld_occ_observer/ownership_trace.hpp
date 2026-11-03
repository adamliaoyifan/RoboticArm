#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace uld_occ_observer {

// Per-object ownership ledger (plan §4: "retain object id, allocation/release
// time, owning handle, epoch and reference lifecycle to distinguish a
// legitimate overlap from a surviving final release"). The byte classes in
// ResourceUsage answer "how much"; this ledger answers "which object, owned
// by whom, since when" so the judge can prove a surviving charge is a live
// owner overlap (e.g. S3's 3,363,936-byte old view and 72-byte external
// surface buffer) rather than a leaked final release.
struct OwnershipRecord {
  std::string object_class;  // "surface_buffer" | "view"
  std::uint64_t object_id{};
  std::uint64_t bytes{};
  std::int64_t allocated_mono_ns{};
  std::int64_t released_mono_ns{};  // 0 while live
  std::string handle;               // lease handle, "" while unleased
  std::uint64_t epoch{};            // epoch the object was built in
  std::uint64_t refcount{};         // last observed shared_ptr use count
};

class OwnershipTrace {
 public:
  static constexpr std::size_t kMaxLive = 4096;
  static constexpr std::size_t kMaxRecent = 4096;

  static OwnershipTrace& instance() {
    static OwnershipTrace trace;
    return trace;
  }
  static bool enabled() {
    return instance().enabled_.load(std::memory_order_acquire);
  }
  // Tracing is opt-in (node parameter) so production hot paths pay one
  // atomic load; enabled_ is set once by the node before traffic starts.
  void set_enabled(bool on) { enabled_.store(on, std::memory_order_release); }

  std::uint64_t allocated(const std::string& object_class, std::uint64_t bytes,
                          std::uint64_t epoch, std::string handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto id = ++next_id_;
    auto& record = live_[id];
    record = OwnershipRecord{object_class, id, bytes, steady_now_ns(), 0,
                             std::move(handle), epoch, 1};
    if (live_.size() > kMaxLive) live_.erase(live_.begin());
    return id;
  }
  void handle(std::uint64_t object_id, std::string handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = live_.find(object_id); it != live_.end())
      it->second.handle = std::move(handle);
  }
  void refcount(std::uint64_t object_id, std::uint64_t count) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = live_.find(object_id); it != live_.end())
      it->second.refcount = count;
  }
  void released(std::uint64_t object_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = live_.find(object_id); it != live_.end()) {
      auto record = it->second;
      record.released_mono_ns = steady_now_ns();
      live_.erase(it);
      recent_.push_back(std::move(record));
      while (recent_.size() > kMaxRecent) recent_.pop_front();
    }
  }

  // Live records (sorted by id) plus recently released ones; the judge
  // reconciles this against the byte classes at sample time.
  std::vector<OwnershipRecord> snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<OwnershipRecord> out;
    out.reserve(live_.size() + recent_.size());
    for (const auto& [id, record] : live_) out.push_back(record);
    for (const auto& record : recent_) out.push_back(record);
    return out;
  }
  void clear_recent() {
    std::lock_guard<std::mutex> lock(mutex_);
    recent_.clear();
  }

 private:
  static std::int64_t steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  mutable std::mutex mutex_;  // leaf lock; never taken under store/repository
  std::map<std::uint64_t, OwnershipRecord> live_;
  std::deque<OwnershipRecord> recent_;
  std::uint64_t next_id_{};
  std::atomic<bool> enabled_{false};
};

}  // namespace uld_occ_observer
