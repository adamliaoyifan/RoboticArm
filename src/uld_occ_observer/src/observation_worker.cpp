#include "uld_occ_observer/observation_worker.hpp"
namespace uld_occ_observer {
ObservationWorker::ObservationWorker(ObservationStore& store) : store_(store) {
  diagnostics_.store = store_.diagnostics();
  thread_ = std::thread([this] { run(); });
}
ObservationWorker::~ObservationWorker() {
  { std::lock_guard<std::mutex> lock(mutex_); closing_ = true; }
  ready_.notify_one(); thread_.join();
}
bool ObservationWorker::enqueue(OpticalObservation observation, Completion completion) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++diagnostics_.submitted;
  if (closing_ || queue_.size() >= 2) { ++diagnostics_.queue_rejected; return false; }
  queue_.push_back({std::move(observation),std::move(completion),Clock::now()});
  diagnostics_.waiting = queue_.size(); ready_.notify_one(); return true;
}
WorkerDiagnostics ObservationWorker::diagnostics() const {
  std::lock_guard<std::mutex> lock(mutex_); return diagnostics_;
}
void ObservationWorker::refresh_store_diagnostics() {
  const auto state = store_.diagnostics();
  std::lock_guard<std::mutex> lock(mutex_); diagnostics_.store = state;
}
void ObservationWorker::run() {
  while (true) {
    Job job;
    { std::unique_lock<std::mutex> lock(mutex_);
      ready_.wait(lock,[this] { return closing_ || !queue_.empty(); });
      if (queue_.empty()) return;
      job = std::move(queue_.front()); queue_.pop_front();
      diagnostics_.waiting = queue_.size(); diagnostics_.in_flight = 1;
    }
    const double queue_ms = std::chrono::duration<double,std::milli>(Clock::now()-job.received).count();
    CompletedObservation done;
    done.queue_ms = queue_ms;
    try { done.receipt = store_.integrate(job.observation); }
    catch (const std::exception& error) {
      done.receipt.status = std::string("INTERNAL_ERROR:") + error.what();
      done.receipt.identity = store_.diagnostics().identity;
    }
    const auto state = store_.diagnostics();
    { std::lock_guard<std::mutex> lock(mutex_);
      diagnostics_.store = state; diagnostics_.in_flight = 0;
      diagnostics_.max_queue_ms = std::max(diagnostics_.max_queue_ms,queue_ms);
    }
    job.completion(std::move(done));
  }
}
}  // namespace uld_occ_observer
