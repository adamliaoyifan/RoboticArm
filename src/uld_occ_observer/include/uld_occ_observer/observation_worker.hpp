#pragma once
#include "uld_occ_observer/observer.hpp"
#include <condition_variable>
#include <functional>
#include <thread>

namespace uld_occ_observer {
struct CompletedObservation { UpdateReceipt receipt; double queue_ms{}; };
struct WorkerDiagnostics {
  Diagnostics store;
  std::size_t waiting{}, in_flight{};
  std::uint64_t submitted{}, queue_rejected{};
  double max_queue_ms{};
};
class ObservationWorker {
 public:
  using Completion = std::function<void(CompletedObservation)>;
  explicit ObservationWorker(ObservationStore& store);
  ~ObservationWorker();
  ObservationWorker(const ObservationWorker&) = delete;
  ObservationWorker& operator=(const ObservationWorker&) = delete;
  bool enqueue(OpticalObservation observation, Completion completion);
  WorkerDiagnostics diagnostics() const;
  void refresh_store_diagnostics();
 private:
  using Clock = std::chrono::steady_clock;
  struct Job { OpticalObservation observation; Completion completion; Clock::time_point received; };
  void run();
  ObservationStore& store_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<Job> queue_;
  bool closing_{};
  WorkerDiagnostics diagnostics_;
  std::thread thread_;
};
}  // namespace uld_occ_observer
