"""Copy the observer package and apply the reviewable interface-trace patch.

The frozen product tree is not modified. The diagnostic copy keeps the same
lock scopes; trace calls sit beside them and no-op when the env switch is off.
"""
from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[2]
ROOT = PACKAGE.parents[1]
DIAG_SRC = ROOT / "diag" / "src" / "uld_occ_observer"
DIAG_FILES = Path(__file__).resolve().parent / "diag_src"
PATCH_PATH = Path(__file__).resolve().parent / "interface_trace.patch"


def replace(text, old, new, label):
    if old not in text:
        raise SystemExit("missing anchor in %s" % label)
    if text.count(old) != 1:
        raise SystemExit("anchor not unique in %s" % label)
    return text.replace(old, new, 1)


def patch_cmake(text):
    text = replace(
        text,
        "  src/snapshot_repository.cpp src/observation_worker.cpp)",
        "  src/snapshot_repository.cpp src/observation_worker.cpp\n"
        "  src/interface_trace.cpp)",
        "cmake core")
    return replace(
        text,
        "install(TARGETS observer_profile_cli DESTINATION lib/${PROJECT_NAME})\n",
        "install(TARGETS observer_profile_cli DESTINATION lib/${PROJECT_NAME})\n"
        "add_executable(interface_probe_cli src/interface_probe_cli.cpp test/query_content_digest.cpp)\n"
        "target_include_directories(interface_probe_cli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)\n"
        "target_link_libraries(interface_probe_cli PRIVATE uld_occ_observer_io)\n"
        "set_target_properties(interface_probe_cli PROPERTIES BUILD_RPATH \"${OCTOMAP_LIBRARY_DIRS}\" "
        "INSTALL_RPATH \"${OCTOMAP_LIBRARY_DIRS}\")\n"
        "install(TARGETS interface_probe_cli DESTINATION lib/${PROJECT_NAME})\n",
        "cmake probe")


def patch_observer(text):
    text = replace(text, '#include "uld_occ_observer/observer.hpp"\n',
                   '#include "uld_occ_observer/observer.hpp"\n'
                   '#include "uld_occ_observer/interface_trace.hpp"\n', "observer include")
    text = replace(
        text,
        "UpdateReceipt ObservationStore::integrate(const OpticalObservation& observation, ProfileMode profile_mode) {\n"
        "  const auto start = Clock::now();\n",
        "UpdateReceipt ObservationStore::integrate(const OpticalObservation& observation, ProfileMode profile_mode) {\n"
        "  const auto& trace_id = observation.identity.observation_id;\n"
        "  interface_trace_event(\"integrate_start\", trace_id.c_str(), \"\", \"writer\");\n"
        "  const auto start = Clock::now();\n",
        "integrate start")
    text = replace(
        text,
        "  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);\n"
        "  { auto span = diagnostic.span(ProfileStage::LockWait); lock.lock(); }\n",
        "  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);\n"
        "  interface_trace_event(\"store_lock_attempt\", trace_id.c_str(), \"integrate\", \"writer\");\n"
        "  { auto span = diagnostic.span(ProfileStage::LockWait); lock.lock(); }\n"
        "  interface_trace_event(\"store_lock_acquired\", trace_id.c_str(), \"integrate\", \"writer\");\n",
        "integrate lock")
    text = replace(
        text,
        "      publish();\n      return receipt;\n",
        "      publish();\n"
        "      interface_trace_event(\"store_lock_release\", trace_id.c_str(), receipt.status.c_str(), \"writer\");\n"
        "      interface_trace_event(\"integrate_end\", trace_id.c_str(), receipt.status.c_str(), \"writer\");\n"
        "      return receipt;\n",
        "integrate early return")
    text = replace(
        text,
        "  publish();\n  return receipt;\n}\n",
        "  publish();\n"
        "  interface_trace_event(\"store_lock_release\", trace_id.c_str(), receipt.status.c_str(), \"writer\");\n"
        "  interface_trace_event(\"integrate_end\", trace_id.c_str(), receipt.status.c_str(), \"writer\");\n"
        "  return receipt;\n}\n",
        "integrate return")
    text = replace(
        text,
        "std::shared_ptr<const Snapshot> ObservationStore::snapshot() const {\n"
        "  std::lock_guard<std::mutex> lock(mutex_);\n"
        "  if (!last_integrated_stamp_ns_) throw Error(\"NO_DATA\");\n"
        "  if (cached_snapshot_) return cached_snapshot_;\n"
        "  auto result = std::make_shared<const Snapshot>(identity_unlocked(), domain_, blocks_);\n"
        "  cached_snapshot_ = result;\n"
        "  return result;\n"
        "}\n",
        "std::shared_ptr<const Snapshot> ObservationStore::snapshot() const {\n"
        "  interface_trace_event(\"store_lock_attempt\", \"snapshot\", \"\", \"query\");\n"
        "  std::lock_guard<std::mutex> lock(mutex_);\n"
        "  interface_trace_event(\"store_lock_acquired\", \"snapshot\", \"\", \"query\");\n"
        "  if (!last_integrated_stamp_ns_) {\n"
        "    interface_trace_event(\"store_lock_release\", \"snapshot\", \"NO_DATA\", \"query\");\n"
        "    throw Error(\"NO_DATA\");\n"
        "  }\n"
        "  if (cached_snapshot_) {\n"
        "    interface_trace_event(\"store_lock_release\", \"snapshot\", \"hit\", \"query\");\n"
        "    return cached_snapshot_;\n"
        "  }\n"
        "  auto result = std::make_shared<const Snapshot>(identity_unlocked(), domain_, blocks_);\n"
        "  cached_snapshot_ = result;\n"
        "  interface_trace_event(\"store_lock_release\", \"snapshot\", \"build\", \"query\");\n"
        "  return result;\n"
        "}\n",
        "snapshot")
    return replace(
        text,
        "Diagnostics ObservationStore::diagnostics() const {\n"
        "  std::lock_guard<std::mutex> lock(mutex_);\n"
        "  return {identity_unlocked(), grid_.nodeCount(), known_cells_, blocks_.size(), grid_.memoryUsage(),\n"
        "          received_, integrated_, rejected_, duplicates_, full_serializations_};\n"
        "}\n",
        "Diagnostics ObservationStore::diagnostics() const {\n"
        "  interface_trace_event(\"store_lock_attempt\", \"diagnostics\", \"\", \"query\");\n"
        "  std::lock_guard<std::mutex> lock(mutex_);\n"
        "  interface_trace_event(\"store_lock_acquired\", \"diagnostics\", \"\", \"query\");\n"
        "  Diagnostics out{identity_unlocked(), grid_.nodeCount(), known_cells_, blocks_.size(), grid_.memoryUsage(),\n"
        "                  received_, integrated_, rejected_, duplicates_, full_serializations_};\n"
        "  interface_trace_event(\"store_lock_release\", \"diagnostics\", \"\", \"query\");\n"
        "  return out;\n"
        "}\n",
        "diagnostics")


def patch_worker(text):
    text = replace(text, '#include "uld_occ_observer/observation_worker.hpp"\n',
                   '#include "uld_occ_observer/observation_worker.hpp"\n'
                   '#include "uld_occ_observer/interface_trace.hpp"\n', "worker include")
    text = replace(
        text,
        "bool ObservationWorker::enqueue(OpticalObservation observation, Completion completion) {\n"
        "  std::lock_guard<std::mutex> lock(mutex_);\n"
        "  ++diagnostics_.submitted;\n"
        "  if (closing_ || queue_.size() >= 2) { ++diagnostics_.queue_rejected; return false; }\n"
        "  queue_.push_back({std::move(observation),std::move(completion),Clock::now()});\n"
        "  diagnostics_.waiting = queue_.size(); ready_.notify_one(); return true;\n"
        "}\n",
        "bool ObservationWorker::enqueue(OpticalObservation observation, Completion completion) {\n"
        "  const auto input_id = observation.identity.observation_id;\n"
        "  interface_trace_event(\"enqueue_begin\", input_id.c_str(), \"\", \"writer\");\n"
        "  bool accepted = false;\n"
        "  {\n"
        "    std::lock_guard<std::mutex> lock(mutex_);\n"
        "    ++diagnostics_.submitted;\n"
        "    if (closing_ || queue_.size() >= 2) ++diagnostics_.queue_rejected;\n"
        "    else {\n"
        "      queue_.push_back({std::move(observation),std::move(completion),Clock::now()});\n"
        "      diagnostics_.waiting = queue_.size(); ready_.notify_one(); accepted = true;\n"
        "    }\n"
        "  }\n"
        "  interface_trace_event(\"enqueue_result\", input_id.c_str(), accepted ? \"accepted\" : \"QUEUE_FULL\", \"writer\");\n"
        "  return accepted;\n"
        "}\n",
        "enqueue")
    return replace(
        text,
        "    const double queue_ms = std::chrono::duration<double,std::milli>(Clock::now()-job.received).count();\n"
        "    CompletedObservation done;\n"
        "    done.queue_ms = queue_ms;\n"
        "    try { done.receipt = store_.integrate(job.observation); }\n"
        "    catch (const std::exception& error) {\n"
        "      done.receipt.status = std::string(\"INTERNAL_ERROR:\") + error.what();\n"
        "      done.receipt.identity = store_.diagnostics().identity;\n"
        "    }\n",
        "    const auto input_id = job.observation.identity.observation_id;\n"
        "    interface_trace_event(\"worker_dequeue\", input_id.c_str(), \"\", \"writer\");\n"
        "    const double queue_ms = std::chrono::duration<double,std::milli>(Clock::now()-job.received).count();\n"
        "    CompletedObservation done;\n"
        "    done.queue_ms = queue_ms;\n"
        "    interface_trace_event(\"integrate_worker_start\", input_id.c_str(), \"\", \"writer\");\n"
        "    try { done.receipt = store_.integrate(job.observation); }\n"
        "    catch (const std::exception& error) {\n"
        "      done.receipt.status = std::string(\"INTERNAL_ERROR:\") + error.what();\n"
        "      done.receipt.identity = store_.diagnostics().identity;\n"
        "    }\n"
        "    interface_trace_event(\"integrate_worker_end\", input_id.c_str(), done.receipt.status.c_str(), \"writer\");\n",
        "worker integrate")


def patch_worker_completion(text):
    return replace(
        text,
        "    job.completion(std::move(done));\n",
        "    interface_trace_event(\"worker_completion\", input_id.c_str(), done.receipt.status.c_str(), \"writer\");\n"
        "    job.completion(std::move(done));\n"
        "    interface_trace_event(\"worker_completion_returned\", input_id.c_str(), \"\", \"writer\");\n",
        "worker completion")


def patch_repository(text):
    text = replace(text, '#include "uld_occ_observer/snapshot_repository.hpp"\n',
                   '#include "uld_occ_observer/snapshot_repository.hpp"\n'
                   '#include "uld_occ_observer/interface_trace.hpp"\n', "repository include")
    text = replace(
        text,
        "  std::lock_guard<std::mutex> lock(mutex_);\n"
        "  const auto latest = store_.snapshot();\n",
        "  interface_trace_event(\"repository_lock_attempt\", \"acquire\", \"\", \"query\");\n"
        "  std::lock_guard<std::mutex> lock(mutex_);\n"
        "  interface_trace_event(\"repository_lock_acquired\", \"acquire\", \"\", \"query\");\n"
        "  const auto latest = store_.snapshot();\n",
        "acquire lock")
    text = replace(
        text,
        "  if (expected_epoch && expected_epoch != latest->identity().epoch) throw Error(\"EPOCH_MISMATCH\");\n",
        "  if (expected_epoch && expected_epoch != latest->identity().epoch) {\n"
        "    interface_trace_event(\"repository_lock_release\", \"acquire\", \"EPOCH_MISMATCH\", \"query\");\n"
        "    throw Error(\"EPOCH_MISMATCH\");\n"
        "  }\n",
        "epoch mismatch")
    text = replace(
        text,
        "    if (!selected) throw Error(\"REVISION_UNAVAILABLE\");\n",
        "    if (!selected) {\n"
        "      interface_trace_event(\"repository_lock_release\", \"acquire\", \"REVISION_UNAVAILABLE\", \"query\");\n"
        "      throw Error(\"REVISION_UNAVAILABLE\");\n"
        "    }\n",
        "revision")
    text = replace(
        text,
        "  if (now_ns < identity.last_integrated_stamp_ns || max_age_ns < 0) throw Error(\"INVALID_QUERY_TIME\");\n",
        "  if (now_ns < identity.last_integrated_stamp_ns || max_age_ns < 0) {\n"
        "    interface_trace_event(\"repository_lock_release\", \"acquire\", \"INVALID_QUERY_TIME\", \"query\");\n"
        "    throw Error(\"INVALID_QUERY_TIME\");\n"
        "  }\n",
        "query time")
    text = replace(
        text,
        "  if (age > max_age_ns && !allow_stale) throw Error(\"STALE\");\n",
        "  interface_trace_event(\"snapshot_age\", \"acquire\", std::to_string(age).c_str(), \"query\");\n"
        "  if (age > max_age_ns && !allow_stale) {\n"
        "    interface_trace_event(\"repository_lock_release\", \"acquire\", \"STALE\", \"query\");\n"
        "    throw Error(\"STALE\");\n"
        "  }\n",
        "stale")
    text = replace(
        text,
        "  if (entries_.size() >= 8 || revisions.size() > 2) throw Error(\"SNAPSHOT_PRESSURE\");\n",
        "  if (entries_.size() >= 8 || revisions.size() > 2) {\n"
        "    interface_trace_event(\"repository_lock_release\", \"acquire\", \"SNAPSHOT_PRESSURE\", \"query\");\n"
        "    throw Error(\"SNAPSHOT_PRESSURE\");\n"
        "  }\n",
        "pressure")
    text = replace(
        text,
        "  entries_.emplace(handle, Entry{selected, now + std::chrono::seconds(5)});\n"
        "  return {handle, identity, age, age > max_age_ns};\n",
        "  entries_.emplace(handle, Entry{selected, now + std::chrono::seconds(5)});\n"
        "  interface_trace_event(\"snapshot_selected\", handle.c_str(), std::to_string(age).c_str(), \"query\");\n"
        "  interface_trace_event(\"repository_lock_release\", handle.c_str(), age > max_age_ns ? \"STALE_ALLOWED\" : \"OK\", \"query\");\n"
        "  return {handle, identity, age, age > max_age_ns};\n",
        "acquire return")
    text = replace(
        text,
        "std::shared_ptr<const Snapshot> SnapshotRepository::get(const std::string& handle) {\n"
        "  std::lock_guard<std::mutex> lock(mutex_);\n",
        "std::shared_ptr<const Snapshot> SnapshotRepository::get(const std::string& handle) {\n"
        "  interface_trace_event(\"repository_lock_attempt\", handle.c_str(), \"get\", \"query\");\n"
        "  std::lock_guard<std::mutex> lock(mutex_);\n"
        "  interface_trace_event(\"repository_lock_acquired\", handle.c_str(), \"get\", \"query\");\n",
        "get lock")
    text = replace(
        text,
        "  if (entry == entries_.end()) throw Error(\"INVALID_OR_EXPIRED_HANDLE\");\n"
        "  return entry->second.snapshot;\n",
        "  if (entry == entries_.end()) {\n"
        "    interface_trace_event(\"repository_lock_release\", handle.c_str(), \"INVALID_OR_EXPIRED_HANDLE\", \"query\");\n"
        "    throw Error(\"INVALID_OR_EXPIRED_HANDLE\");\n"
        "  }\n"
        "  interface_trace_event(\"repository_lock_release\", handle.c_str(), \"OK\", \"query\");\n"
        "  return entry->second.snapshot;\n",
        "get return")
    return replace(
        text,
        "void SnapshotRepository::release(const std::string& handle) {\n"
        "  std::lock_guard<std::mutex> lock(mutex_);\n"
        "  entries_.erase(handle);\n"
        "}\n",
        "void SnapshotRepository::release(const std::string& handle) {\n"
        "  interface_trace_event(\"repository_lock_attempt\", handle.c_str(), \"release\", \"query\");\n"
        "  std::lock_guard<std::mutex> lock(mutex_);\n"
        "  interface_trace_event(\"repository_lock_acquired\", handle.c_str(), \"release\", \"query\");\n"
        "  entries_.erase(handle);\n"
        "  interface_trace_event(\"repository_lock_release\", handle.c_str(), \"OK\", \"query\");\n"
        "}\n",
        "release")


def patch_snapshot(text):
    text = replace(text, '#include "uld_occ_observer/snapshot.hpp"\n',
                   '#include "uld_occ_observer/snapshot.hpp"\n'
                   '#include "uld_occ_observer/interface_trace.hpp"\n', "snapshot include")
    return replace(
        text,
        "  std::lock_guard<std::mutex> lock(surface_cache_->mutex);\n",
        "  std::lock_guard<std::mutex> lock(surface_cache_->mutex);\n"
        "  interface_trace_event(\"surface_cache_locked\", \"\", surface_cache_->value ? \"hit\" : \"miss\", \"query\");\n",
        "surface cache")


def patch_node(text):
    text = replace(text, "#include <atomic>\n",
                   "#include <atomic>\n#include \"uld_occ_observer/interface_trace.hpp\"\n", "node include")
    text = replace(
        text,
        "  ObserverNode() : Node(\"uld_occ_observer\", \"/uld_occ_observer\") {\n",
        "  ObserverNode() : Node(\"uld_occ_observer\", \"/uld_occ_observer\") {\n"
        "    interface_trace_configure_from_env();\n",
        "constructor")
    text = replace(text, "~ObserverNode() override { worker_.reset(); }\n",
                   "~ObserverNode() override { worker_.reset(); interface_trace_flush(); }\n", "destructor")
    text = replace(
        text,
        "    observation.points_m = optical_points(cloud);\n    return observation;\n",
        "    observation.points_m = optical_points(cloud);\n"
        "    interface_trace_event(\"points_decoded\", observation.identity.observation_id.c_str(),\n"
        "        std::to_string(observation.points_m.size()).c_str(), \"input\");\n"
        "    return observation;\n",
        "convert")
    text = replace(
        text,
        """  void on_cloud(const sensor_msgs::msg::PointCloud2& cloud) {
    try {
      if (cloud.header.frame_id != optical_frame_) throw Error("OPTICAL_FRAME_MISMATCH");
      const auto tf = tf_buffer_->lookupTransform("container_link", cloud.header.frame_id,
          rclcpp::Time(cloud.header.stamp),rclcpp::Duration::from_seconds(.1));
      const auto stamp = rclcpp::Time(cloud.header.stamp).nanoseconds();
      auto observation = convert(cloud,tf,sensor_id_,std::to_string(stamp),config_.clock_domain,
          config_.geometry.geometry_hash,store_->diagnostics().identity.epoch);
      if (!worker_->enqueue(std::move(observation),[this](CompletedObservation done) {
            receipt_pub_->publish(ros_receipt(done)); })) throw Error("QUEUE_FULL");
    } catch (const std::exception& error) {
      ++adapter_rejected_;
      msg::UpdateReceipt receipt; receipt.status = error.what();
      receipt.sensor_id = sensor_id_; receipt.input_stamp_ns = rclcpp::Time(cloud.header.stamp).nanoseconds();
      receipt.observation_id = std::to_string(receipt.input_stamp_ns); receipt_pub_->publish(receipt);
    }
  }
""",
        """  void on_cloud(const sensor_msgs::msg::PointCloud2& cloud) {
    const auto cloud_id = std::to_string(rclcpp::Time(cloud.header.stamp).nanoseconds());
    interface_trace_event("callback_enter", cloud_id.c_str(), "", "input");
    try {
      if (cloud.header.frame_id != optical_frame_) throw Error("OPTICAL_FRAME_MISMATCH");
      interface_trace_event("tf_lookup_begin", cloud_id.c_str(), "", "input");
      const auto tf = tf_buffer_->lookupTransform("container_link", cloud.header.frame_id,
          rclcpp::Time(cloud.header.stamp),rclcpp::Duration::from_seconds(.1));
      const auto stamp = rclcpp::Time(cloud.header.stamp).nanoseconds();
      interface_trace_event("tf_lookup_end", cloud_id.c_str(), "OK", "input");
      interface_trace_event("convert_begin", cloud_id.c_str(), "", "input");
      auto observation = convert(cloud,tf,sensor_id_,std::to_string(stamp),config_.clock_domain,
          config_.geometry.geometry_hash,store_->diagnostics().identity.epoch);
      interface_trace_event("convert_end", observation.identity.observation_id.c_str(), "", "input");
      if (!worker_->enqueue(std::move(observation),[this](CompletedObservation done) {
            interface_trace_event("receipt_publish_begin", done.receipt.observation_id.c_str(), done.receipt.status.c_str(), "receipt");
            receipt_pub_->publish(ros_receipt(done));
            interface_trace_event("receipt_publish_end", done.receipt.observation_id.c_str(), done.receipt.status.c_str(), "receipt");
            })) throw Error("QUEUE_FULL");
    } catch (const std::exception& error) {
      ++adapter_rejected_;
      interface_trace_event("adapter_rejected", cloud_id.c_str(), error.what(), "input");
      msg::UpdateReceipt receipt; receipt.status = error.what();
      receipt.sensor_id = sensor_id_; receipt.input_stamp_ns = rclcpp::Time(cloud.header.stamp).nanoseconds();
      receipt.observation_id = std::to_string(receipt.input_stamp_ns);
      interface_trace_event("receipt_publish_begin", receipt.observation_id.c_str(), receipt.status.c_str(), "receipt");
      receipt_pub_->publish(receipt);
      interface_trace_event("receipt_publish_end", receipt.observation_id.c_str(), receipt.status.c_str(), "receipt");
    }
    interface_trace_event("callback_exit", cloud_id.c_str(), "", "input");
  }
""",
        "on_cloud")
    text = replace(
        text,
        """          if (!worker_->enqueue(std::move(observation),[promise](CompletedObservation done) {
                promise->set_value(std::move(done)); })) throw Error("QUEUE_FULL");
          response->receipt = ros_receipt(future.get()); receipt_pub_->publish(response->receipt);
""",
        """          if (!worker_->enqueue(std::move(observation),[promise](CompletedObservation done) {
                promise->set_value(std::move(done)); })) throw Error("QUEUE_FULL");
          response->receipt = ros_receipt(future.get());
          interface_trace_event("receipt_publish_begin", response->receipt.observation_id.c_str(), response->receipt.status.c_str(), "receipt");
          receipt_pub_->publish(response->receipt);
          interface_trace_event("receipt_publish_end", response->receipt.observation_id.c_str(), response->receipt.status.c_str(), "receipt");
""",
        "service publish")
    text = replace(
        text,
        """        try {
          if (request->max_age_ms < 0 || request->max_age_ms > 86400000) throw Error("INVALID_MAX_AGE");
          const auto lease = repository_->acquire(now().nanoseconds(),request->max_age_ms*1000000,
""",
        """        try {
          interface_trace_event("query_callback_enter", "get_snapshot", "", "query");
          if (request->max_age_ms < 0 || request->max_age_ms > 86400000) throw Error("INVALID_MAX_AGE");
          interface_trace_event("acquire_call_begin", "get_snapshot", "", "query");
          const auto lease = repository_->acquire(now().nanoseconds(),request->max_age_ms*1000000,
""",
        "acquire enter")
    text = replace(
        text,
        """          response->status = "OK"; response->handle = lease.handle;
          response->identity = ros_identity(lease.identity); response->age_ns = lease.age_ns;
          response->stale = lease.stale;
        } catch (const std::exception& error) { response->status = error.what(); }
""",
        """          interface_trace_event("response_assemble_begin", lease.handle.c_str(), "get_snapshot", "query");
          response->status = "OK"; response->handle = lease.handle;
          response->identity = ros_identity(lease.identity); response->age_ns = lease.age_ns;
          response->stale = lease.stale;
          interface_trace_event("response_assemble_end", lease.handle.c_str(), "OK", "query");
          interface_trace_event("query_callback_exit", lease.handle.c_str(), "OK", "query");
        } catch (const std::exception& error) {
          response->status = error.what();
          interface_trace_event("query_callback_exit", "get_snapshot", error.what(), "query");
        }
""",
        "acquire exit")
    text = replace(
        text,
        """        try {
          const auto snapshot = repository_->get(request->handle);
          const auto& b = request->begin; const auto& e = request->end;
          const auto page = snapshot->read_cells({b[0],b[1],b[2]},{e[0],e[1],e[2]},request->cursor,request->limit);
""",
        """        try {
          interface_trace_event("query_callback_enter", request->handle.c_str(), "read_cells", "query");
          const auto snapshot = repository_->get(request->handle);
          const auto& b = request->begin; const auto& e = request->end;
          interface_trace_event("roi_begin", request->handle.c_str(), "", "query");
          const auto page = snapshot->read_cells({b[0],b[1],b[2]},{e[0],e[1],e[2]},request->cursor,request->limit);
          interface_trace_event("roi_end", request->handle.c_str(), std::to_string(page.cells.size()).c_str(), "query");
          interface_trace_event("response_assemble_begin", request->handle.c_str(), "read_cells", "query");
""",
        "roi begin")
    text = replace(
        text,
        """          response->status = "OK";
        } catch (const std::exception& error) { response->status = error.what(); }
      });
    surface_ = service<srv::ReadSurface>("read_surface",
""",
        """          response->status = "OK";
          interface_trace_event("response_assemble_end", request->handle.c_str(), "OK", "query");
          interface_trace_event("query_callback_exit", request->handle.c_str(), "OK", "query");
        } catch (const std::exception& error) {
          response->status = error.what();
          interface_trace_event("query_callback_exit", request->handle.c_str(), error.what(), "query");
        }
      });
    surface_ = service<srv::ReadSurface>("read_surface",
""",
        "roi end")
    text = replace(
        text,
        """        try {
          const auto snapshot = repository_->get(request->handle);
          const auto surface = snapshot->surface();
""",
        """        try {
          interface_trace_event("query_callback_enter", request->handle.c_str(), "read_surface", "query");
          const auto snapshot = repository_->get(request->handle);
          interface_trace_event("surface_begin", request->handle.c_str(), "", "query");
          const auto surface = snapshot->surface();
          interface_trace_event("surface_end", request->handle.c_str(), std::to_string(surface->size()).c_str(), "query");
          interface_trace_event("response_assemble_begin", request->handle.c_str(), "read_surface", "query");
""",
        "surface begin")
    text = replace(
        text,
        """          response->next_cursor = stop; response->done = stop == surface->size(); response->status = "OK";
        } catch (const std::exception& error) { response->status = error.what(); }
      });
    release_ = service<srv::ReleaseSnapshot>("release_snapshot",
""",
        """          response->next_cursor = stop; response->done = stop == surface->size(); response->status = "OK";
          interface_trace_event("response_assemble_end", request->handle.c_str(), "OK", "query");
          interface_trace_event("query_callback_exit", request->handle.c_str(), "OK", "query");
        } catch (const std::exception& error) {
          response->status = error.what();
          interface_trace_event("query_callback_exit", request->handle.c_str(), error.what(), "query");
        }
      });
    release_ = service<srv::ReleaseSnapshot>("release_snapshot",
""",
        "surface end")
    text = replace(
        text,
        """        repository_->release(request->handle); response->status = "OK";
      });
""",
        """        interface_trace_event("query_callback_enter", request->handle.c_str(), "release_snapshot", "query");
        repository_->release(request->handle); response->status = "OK";
        interface_trace_event("query_callback_exit", request->handle.c_str(), "OK", "query");
      });
""",
        "release service")
    return replace(
        text,
        "    std_msgs::msg::String message; message.data = YAML::Dump(health); health_pub_->publish(message);\n",
        "    std_msgs::msg::String message; message.data = YAML::Dump(health);\n"
        "    interface_trace_event(\"health_publish_begin\", \"\", \"\", \"health\");\n"
        "    health_pub_->publish(message);\n"
        "    interface_trace_event(\"health_publish_end\", \"\", \"\", \"health\");\n",
        "health")


def main():
    if DIAG_SRC.exists():
        shutil.rmtree(DIAG_SRC)
    shutil.copytree(PACKAGE, DIAG_SRC, symlinks=True,
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    for name, dest in (
        ("interface_trace.hpp", DIAG_SRC / "include" / "uld_occ_observer" / "interface_trace.hpp"),
        ("interface_trace.cpp", DIAG_SRC / "src" / "interface_trace.cpp"),
        ("interface_probe_cli.cpp", DIAG_SRC / "src" / "interface_probe_cli.cpp"),
    ):
        shutil.copy2(DIAG_FILES / name, dest)
    files = {
        "CMakeLists.txt": patch_cmake,
        "src/observer.cpp": patch_observer,
        "src/observation_worker.cpp": patch_worker,
        "src/snapshot_repository.cpp": patch_repository,
        "src/snapshot.cpp": patch_snapshot,
        "src/observer_node.cpp": patch_node,
    }
    for relative, function in files.items():
        path = DIAG_SRC / relative
        path.write_text(function(path.read_text()))
    worker = DIAG_SRC / "src" / "observation_worker.cpp"
    worker.write_text(patch_worker_completion(worker.read_text()))
    diff = subprocess.run(
        ["diff", "-ruN", "src/uld_occ_observer", "diag/src/uld_occ_observer"],
        cwd=ROOT, capture_output=True, text=True, check=False)
    if diff.returncode not in (0, 1):
        raise SystemExit(diff.stderr)
    PATCH_PATH.write_text(diff.stdout)
    print("diagnostic source %s" % DIAG_SRC)
    print("patch %s bytes %d" % (PATCH_PATH, PATCH_PATH.stat().st_size))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
