#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>

namespace uld_occ_observer::test {

// Deterministic stage gates for reset-race injection (plan §5 V1/V3, §6
// "barriers at named stage boundaries, not sleeps"). Product call sites run
// gate_checkpoint(<stage>) on hot paths; the fast path is one relaxed atomic
// load and returns immediately when no gate is armed, so an unarmed build
// pays nothing. The benchmark harness arms a gate, lets a request block at
// the named boundary, injects reset while the request is parked, then
// releases. Gates never block the reset path itself and are not part of the
// lock-order table: their mutex is a test leaf taken only inside
// gate_checkpoint/arm/release.
class StageGates {
 public:
  static StageGates& instance() {
    static StageGates gates;
    return gates;
  }
  static bool any_armed() {
    return instance().armed_any_.load(std::memory_order_acquire);
  }

  // One-shot semantics: arm() parks exactly the FIRST arrival at the stage
  // (it consumes the arm), so a new-epoch request sent after the injection
  // crosses the same boundary instead of parking behind the old one.
  // release() wakes the parked arrival; unarmed arrivals pay nothing.
  void arm(const std::string& stage) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& slot = slots_[stage];
    slot.armed = true;
    slot.released = false;
    armed_any_.store(true, std::memory_order_release);
  }
  void release(const std::string& stage) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = slots_.find(stage);
    if (it == slots_.end()) return;
    it->second.armed = false;
    it->second.released = true;
    cv_.notify_all();
    bool any = false;
    for (const auto& [name, slot] : slots_) any = any || slot.armed;
    armed_any_.store(any, std::memory_order_release);
  }
  void release_all() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [name, slot] : slots_) {
      slot.armed = false;
      slot.released = true;
    }
    cv_.notify_all();
    armed_any_.store(false, std::memory_order_release);
  }
  // "stage=<name> hit=<total arrivals released> blocked=<currently parked>"
  // plus per-stage counters, for the harness to observe exact arrival.
  std::string probe() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string out;
    for (const auto& [name, slot] : slots_) {
      if (!out.empty()) out += ' ';
      out += name + ":hit=" + std::to_string(slot.hit)
          + ",blocked=" + std::to_string(slot.blocked);
    }
    return out.empty() ? "none" : out;
  }
  // Call site. Cheap no-op unless armed.
  void checkpoint(const std::string& stage) {
    if (!armed_any_.load(std::memory_order_acquire)) return;
    std::unique_lock<std::mutex> lock(mutex_);
    auto it = slots_.find(stage);
    if (it == slots_.end() || !it->second.armed) return;
    it->second.armed = false;  // one-shot: later arrivals pass through
    ++it->second.blocked;
    cv_.wait(lock, [&] { return it->second.released; });
    --it->second.blocked;
    ++it->second.hit;
  }

 private:
  struct Slot {
    bool armed{false};
    bool released{true};
    std::uint64_t hit{0}, blocked{0};
  };
  std::mutex mutex_;
  std::condition_variable cv_;
  std::map<std::string, Slot> slots_;
  std::atomic<bool> armed_any_{false};
};

inline void gate_checkpoint(const char* stage) {
  StageGates::instance().checkpoint(stage);
}

}  // namespace uld_occ_observer::test
