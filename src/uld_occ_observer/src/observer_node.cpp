#include "uld_occ_observer/configuration.hpp"
#include "uld_occ_observer/checkpoint_io.hpp"
#include "uld_occ_observer/snapshot_repository.hpp"
#include "uld_occ_observer/ros_adapters.hpp"
#include "uld_occ_observer/observation_ingress.hpp"
#include "uld_occ_observer/observation_ledger.hpp"
#include "uld_occ_observer/snapshot_wait_board.hpp"
#include "uld_occ_observer/srv/integrate_observation.hpp"
#include "uld_occ_observer/srv/get_snapshot.hpp"
#include "uld_occ_observer/srv/read_cells.hpp"
#include "uld_occ_observer/srv/read_surface.hpp"
#include "uld_occ_observer/srv/release_snapshot.hpp"
#include "uld_occ_observer/srv/reset_observation.hpp"
#include "uld_occ_observer/srv/checkpoint_observation.hpp"
#include "uld_occ_observer/srv/get_observation_status.hpp"
#include "uld_occ_observer/srv/wait_snapshot.hpp"
#include "uld_occ_observer/srv/get_resource_usage.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <condition_variable>
#include <future>
#include <iostream>
#include <atomic>
#include <thread>

namespace uld_occ_observer {
class ObserverNode : public rclcpp::Node {
 public:
  ObserverNode() : Node("uld_occ_observer", "/uld_occ_observer") {
    const auto geometry_file = declare_parameter<std::string>("geometry_file", "");
    const auto config_file = declare_parameter<std::string>("config_file", "");
    config_ = load_configuration(geometry_file, config_file);
    mode_ = declare_parameter<std::string>("input_mode", "service");
    optical_frame_ = declare_parameter<std::string>("optical_frame", "");
    sensor_id_ = declare_parameter<std::string>("sensor_id", "d555");
    if (mode_ != "service" && mode_ != "subscription") throw Error("INVALID_INPUT_MODE");
    if (mode_ == "subscription" && optical_frame_.empty()) throw Error("OPTICAL_FRAME_REQUIRED");
    if (config_.clock_domain != "ros") throw Error("ROS_CLOCK_DOMAIN_REQUIRED");
    store_ = std::make_unique<ObservationStore>(config_.geometry,config_.resolution,
        config_.depth,config_.clock_domain,config_.digest,config_.discretize_endpoints);
    repository_ = std::make_unique<SnapshotRepository>(*store_);
    receipt_pub_ = create_publisher<msg::UpdateReceipt>(
        "receipts", rclcpp::QoS(rclcpp::KeepLast(1024)).reliable());
    health_pub_ = create_publisher<std_msgs::msg::String>("health",rclcpp::QoS(10).reliable());
    worker_ = std::make_unique<ObservationWorker>(*store_);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    health_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    if (mode_ == "subscription") {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      rclcpp::SubscriptionOptions options; options.callback_group = input_group_;
      const auto topic = declare_parameter<std::string>("cloud_topic", "optical_points");
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          topic, rclcpp::QoS(rclcpp::KeepLast(32)).reliable(),
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { on_cloud(*cloud); }, options);
      adapter_ = std::thread([this] { adapt_loop(); });
    }
    install_services();
    health_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { publish_health(); }, health_group_);
    wait_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] { wake_waits(); }, service_group_);
    RCLCPP_INFO(get_logger(),"Ready: independent observer, OctoMap 1.9.8, input_mode=%s",mode_.c_str());
  }
  ~ObserverNode() override {
    adapter_stop_.store(true);
    adapter_cv_.notify_all();
    if (adapter_.joinable()) adapter_.join();
    worker_.reset();
  }
  void run() {
    rclcpp::executors::SingleThreadedExecutor input_executor;
    input_executor.add_callback_group(input_group_, get_node_base_interface());
    std::thread input_thread([&] { input_executor.spin(); });
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_callback_group(service_group_, get_node_base_interface());
    executor.add_callback_group(health_group_, get_node_base_interface());
    executor.spin();
    input_executor.cancel();
    input_thread.join();
  }
 private:
  OpticalObservation convert(const sensor_msgs::msg::PointCloud2& cloud,
      const geometry_msgs::msg::TransformStamped& tf, const std::string& sensor,
      const std::string& id, const std::string& clock, const std::string& geometry,
      std::uint64_t epoch) {
    if (!optical_frame_.empty() && cloud.header.frame_id != optical_frame_)
      throw Error("OPTICAL_FRAME_MISMATCH");
    OpticalObservation observation;
    observation.identity = {sensor,id,cloud.header.frame_id,clock,geometry,epoch,
      static_cast<std::int64_t>(cloud.header.stamp.sec)*1000000000+cloud.header.stamp.nanosec};
    observation.uld_from_optical = rigid_transform(tf,cloud.header.frame_id);
    observation.points_m = optical_points(cloud);
    return observation;
  }
  void on_cloud(const sensor_msgs::msg::PointCloud2& cloud) {
    if (cloud.header.frame_id != optical_frame_) {
      publish_rejection("", "OPTICAL_FRAME_MISMATCH", rclcpp::Time(cloud.header.stamp).nanoseconds());
      return;
    }
    const auto stamp = rclcpp::Time(cloud.header.stamp).nanoseconds();
    const auto id = std::to_string(stamp);
    const auto epoch = store_->published_epoch();
    const auto now = std::chrono::steady_clock::now();
    const auto admitted = ledger_.admit(epoch, sensor_id_, id, now);
    if (!admitted.accepted) {
      publish_rejection(id, admitted.status, stamp);
      return;
    }
    ObservationIngress::Item item;
    item.sensor_id = sensor_id_;
    item.observation_id = id;
    item.epoch = epoch;
    item.payload = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud);
    const auto queued = ingress_.try_admit(std::move(item));
    if (!queued.accepted) {
      UpdateReceipt receipt;
      receipt.status = queued.status;
      receipt.sensor_id = sensor_id_;
      receipt.observation_id = id;
      receipt.input_stamp_ns = stamp;
      ledger_.complete(epoch, sensor_id_, id, receipt, now);
      publish_rejection(id, queued.status, stamp);
      return;
    }
    adapter_cv_.notify_one();
  }
  void adapt_loop() {
    while (!adapter_stop_.load()) {
      auto item = ingress_.try_pop();
      if (!item) {
        std::unique_lock<std::mutex> lock(adapter_mu_);
        adapter_cv_.wait_for(lock, std::chrono::milliseconds(5), [&] {
          return adapter_stop_.load() || ingress_.size() > 0;
        });
        continue;
      }
      adapt(*item);
    }
  }
  void adapt(const ObservationIngress::Item& item) {
    const auto now = std::chrono::steady_clock::now();
    ledger_.mark(item.epoch, item.sensor_id, item.observation_id, "PROCESSING", true, now);
    try {
      const auto cloud = std::static_pointer_cast<sensor_msgs::msg::PointCloud2>(item.payload);
      const auto tf = tf_buffer_->lookupTransform("container_link", cloud->header.frame_id,
          rclcpp::Time(cloud->header.stamp), rclcpp::Duration::from_seconds(.1));
      auto observation = convert(*cloud, tf, item.sensor_id, item.observation_id, config_.clock_domain,
          config_.geometry.geometry_hash, store_->published_epoch());
      const auto stamp = observation.identity.stamp_ns;
      const auto epoch = item.epoch;
      const auto sensor = item.sensor_id;
      const auto id = item.observation_id;
      if (!worker_->enqueue(std::move(observation), [this, epoch, sensor, id](CompletedObservation done) {
            finish_receipt(epoch, sensor, id, std::move(done)); })) {
        UpdateReceipt receipt;
        receipt.status = "QUEUE_FULL";
        receipt.sensor_id = sensor;
        receipt.observation_id = id;
        receipt.input_stamp_ns = stamp;
        ledger_.complete(epoch, sensor, id, receipt, std::chrono::steady_clock::now());
        publish_rejection(id, "QUEUE_FULL", stamp);
      }
    } catch (const std::exception& error) {
      UpdateReceipt receipt;
      receipt.status = error.what();
      receipt.sensor_id = item.sensor_id;
      receipt.observation_id = item.observation_id;
      ledger_.complete(item.epoch, item.sensor_id, item.observation_id, receipt, std::chrono::steady_clock::now());
      publish_rejection(item.observation_id, error.what(), 0);
    }
  }
  void finish_receipt(std::uint64_t epoch, const std::string& sensor, const std::string& id,
                      CompletedObservation done) {
    ledger_.complete(epoch, sensor, id, done.receipt, std::chrono::steady_clock::now());
    receipt_pub_->publish(ros_receipt(done));
    SnapshotWaitBoard::Completion completion;
    completion.epoch = epoch;
    completion.sensor_id = sensor;
    completion.observation_id = id;
    completion.status = done.receipt.status == "INTEGRATED" ? "COMPLETED" : done.receipt.status;
    completion.observed_revision = done.receipt.identity.observed_revision;
    board_.note_completion(completion);
  }
  void publish_rejection(const std::string& id, const std::string& status, std::int64_t stamp) {
    ++adapter_rejected_;
    msg::UpdateReceipt receipt;
    receipt.status = status;
    receipt.sensor_id = sensor_id_;
    receipt.observation_id = id;
    receipt.input_stamp_ns = stamp;
    receipt_pub_->publish(receipt);
  }
  template<class Service, class Callback>
  typename rclcpp::Service<Service>::SharedPtr service(const std::string& name, Callback callback) {
    return create_service<Service>(name,callback,rmw_qos_profile_services_default,service_group_);
  }
  void install_services() {
    integrate_ = service<srv::IntegrateObservation>("integrate_observation",
      [this](const std::shared_ptr<srv::IntegrateObservation::Request> request,
             std::shared_ptr<srv::IntegrateObservation::Response> response) {
        try {
          if (mode_ != "service") throw Error("INPUT_MODE_MISMATCH");
          auto observation = convert(request->cloud,request->uld_from_optical,request->sensor_id,
              request->observation_id,request->clock_domain,request->geometry_hash,request->epoch);
          auto promise = std::make_shared<std::promise<CompletedObservation>>();
          auto future = promise->get_future();
          const auto epoch = request->epoch;
          const auto sensor = request->sensor_id;
          const auto id = request->observation_id;
          ledger_.admit(epoch, sensor, id, std::chrono::steady_clock::now());
          ledger_.mark(epoch, sensor, id, "PROCESSING", true, std::chrono::steady_clock::now());
          if (!worker_->enqueue(std::move(observation),[promise](CompletedObservation done) {
                promise->set_value(std::move(done)); })) throw Error("QUEUE_FULL");
          const auto done = future.get();
          ledger_.complete(epoch, sensor, id, done.receipt, std::chrono::steady_clock::now());
          response->receipt = ros_receipt(done); receipt_pub_->publish(response->receipt);
        } catch (const std::exception& error) {
          response->receipt.status = error.what(); response->receipt.sensor_id = request->sensor_id;
          response->receipt.observation_id = request->observation_id;
          response->receipt.input_stamp_ns = rclcpp::Time(request->cloud.header.stamp).nanoseconds();
        }
      });
    acquire_ = service<srv::GetSnapshot>("get_snapshot",
      [this](const std::shared_ptr<srv::GetSnapshot::Request> request,
             std::shared_ptr<srv::GetSnapshot::Response> response) {
        try {
          if (request->max_age_ms < 0 || request->max_age_ms > 86400000) throw Error("INVALID_MAX_AGE");
          const auto lease = repository_->acquire(now().nanoseconds(),request->max_age_ms*1000000,
              request->allow_stale,request->expected_epoch,request->exact_revision ?
                std::optional<std::uint64_t>(request->expected_revision) : std::nullopt);
          response->status = "OK"; response->handle = lease.handle;
          response->identity = ros_identity(lease.identity); response->age_ns = lease.age_ns;
          response->stale = lease.stale;
        } catch (const std::exception& error) { response->status = error.what(); }
      });
    cells_ = service<srv::ReadCells>("read_cells",
      [this](const std::shared_ptr<srv::ReadCells::Request> request,
             std::shared_ptr<srv::ReadCells::Response> response) {
        try {
          const auto started = std::chrono::steady_clock::now();
          const auto snapshot = repository_->get(request->handle);
          response->repository_wait_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - started).count();
          response->store_wait_ms = 0;
          const auto& b = request->begin; const auto& e = request->end;
          const auto page = snapshot->read_cells({b[0],b[1],b[2]},{e[0],e[1],e[2]},request->cursor,request->limit);
          response->identity = ros_identity(snapshot->identity());
          response->next_cursor = page.next_cursor; response->done = page.done;
          response->states.reserve(page.cells.size());
          response->log_odds.reserve(page.cells.size());
          const auto fill_started = std::chrono::steady_clock::now();
          for (const auto& cell : page.cells) {
            response->states.push_back(!cell.active ? 3 : !cell.known ? 0 : cell.value.occupied ? 2 : 1);
            response->log_odds.push_back(cell.known ? cell.value.log_odds : 0);
          }
          response->response_fill_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - fill_started).count();
          response->surface_build_ms = 0;
          response->server_total_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - started).count();
          response->status = "OK";
        } catch (const std::exception& error) { response->status = error.what(); }
      });
    surface_ = service<srv::ReadSurface>("read_surface",
      [this](const std::shared_ptr<srv::ReadSurface::Request> request,
             std::shared_ptr<srv::ReadSurface::Response> response) {
        try {
          const auto started = std::chrono::steady_clock::now();
          const auto snapshot = repository_->get(request->handle);
          response->repository_wait_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - started).count();
          response->store_wait_ms = 0;
          SurfaceProfile profile;
          const auto surface = snapshot->surface(&profile);
          response->surface_build_ms = profile.total_ms;
          if (!request->limit || request->limit > 65536 || request->cursor > surface->size())
            throw Error("INVALID_SURFACE_PAGE");
          const auto count = std::min<std::uint64_t>(request->limit, surface->size() - request->cursor);
          const auto stop = request->cursor + count;
          response->x.reserve(count); response->y.reserve(count);
          response->active_count.reserve(count); response->known_count.reserve(count);
          response->has_occupied.reserve(count); response->top_z.reserve(count);
          repository_->set_inflight_response_bytes(count * sizeof(SurfaceColumn));
          const auto fill_started = std::chrono::steady_clock::now();
          for (auto i = request->cursor; i < stop; ++i) {
            const auto& column = surface->at(i);
            response->x.push_back(column.x); response->y.push_back(column.y);
            response->active_count.push_back(column.active_count); response->known_count.push_back(column.known_count);
            response->has_occupied.push_back(column.has_occupied); response->top_z.push_back(column.top_z);
          }
          response->response_fill_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - fill_started).count();
          repository_->set_inflight_response_bytes(0);
          response->identity = ros_identity(snapshot->identity());
          response->next_cursor = stop; response->done = stop == surface->size();
          response->server_total_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - started).count();
          response->status = "OK";
        } catch (const std::exception& error) { response->status = error.what(); }
      });
    release_ = service<srv::ReleaseSnapshot>("release_snapshot",
      [this](const std::shared_ptr<srv::ReleaseSnapshot::Request> request,
             std::shared_ptr<srv::ReleaseSnapshot::Response> response) {
        repository_->release(request->handle); response->status = "OK";
      });
    reset_ = service<srv::ResetObservation>("reset_observation",
      [this](const std::shared_ptr<srv::ResetObservation::Request> request,
             std::shared_ptr<srv::ResetObservation::Response> response) {
        try {
          if (request->reset_id.empty()) throw Error("RESET_ID_REQUIRED");
          response->epoch = store_->reset(request->expected_epoch);
          worker_->refresh_store_diagnostics(); response->status = "OK";
        } catch (const std::exception& error) { response->status = error.what(); }
      });
    checkpoint_ = service<srv::CheckpointObservation>("checkpoint_observation",
      [this](const std::shared_ptr<srv::CheckpointObservation::Request> request,
             std::shared_ptr<srv::CheckpointObservation::Response> response) {
        try {
          if (request->operation == "checkpoint") {
            const auto data = store_->checkpoint();
            if (data.identity.epoch != request->expected_epoch) throw Error("EPOCH_MISMATCH");
            write_checkpoint(data,request->path); response->epoch = data.identity.epoch;
          } else if (request->operation == "restore")
            response->epoch = store_->restore(read_checkpoint(request->path),request->expected_epoch);
          else throw Error("UNSUPPORTED_OPERATION");
          worker_->refresh_store_diagnostics(); response->status = "OK";
        } catch (const std::exception& error) { response->status = error.what(); }
      });
    status_ = service<srv::GetObservationStatus>("get_observation_status",
      [this](const std::shared_ptr<srv::GetObservationStatus::Request> request,
             std::shared_ptr<srv::GetObservationStatus::Response> response) {
        const auto record = ledger_.query(request->epoch, request->sensor_id, request->observation_id);
        if (!record) { response->status = "NOT_SEEN"; return; }
        response->status = record->state;
        response->receipt = ros_receipt(CompletedObservation{record->receipt, 0});
      });
    resource_ = service<srv::GetResourceUsage>("get_resource_usage",
      [this](const std::shared_ptr<srv::GetResourceUsage::Request> request,
             std::shared_ptr<srv::GetResourceUsage::Response> response) {
        (void)request;
        repository_->set_queue_counts(ingress_.size(), ingress_.peak(), ledger_.size(), ledger_.peak());
        const auto usage = repository_->usage();
        response->status = "OK";
        response->shared_block_bytes = usage.shared_block_bytes;
        response->shared_block_bytes_peak = usage.shared_block_bytes_peak;
        response->old_view_block_bytes = usage.old_view_block_bytes;
        response->view_directory_entries = usage.view_directory_entries;
        response->view_directory_entries_peak = usage.view_directory_entries_peak;
        response->surface_cache_bytes = usage.surface_cache_bytes;
        response->surface_cache_bytes_peak = usage.surface_cache_bytes_peak;
        response->inflight_response_bytes = usage.inflight_response_bytes;
        response->inflight_response_bytes_peak = usage.inflight_response_bytes_peak;
        response->entry_queue_messages = usage.entry_queue_messages;
        response->entry_queue_messages_peak = usage.entry_queue_messages_peak;
        response->receipt_ledger_records = usage.receipt_ledger_records;
        response->receipt_ledger_records_peak = usage.receipt_ledger_records_peak;
        response->uncovered_allocations = usage.uncovered_allocations;
      });
    wait_ = create_service<srv::WaitSnapshot>("wait_snapshot",
      [this](const std::shared_ptr<rclcpp::Service<srv::WaitSnapshot>> service,
             const std::shared_ptr<rmw_request_id_t> header,
             const std::shared_ptr<srv::WaitSnapshot::Request> request) {
        SnapshotWaitBoard::Request wait;
        wait.expected_epoch = request->expected_epoch;
        wait.sensor_id = request->sensor_id;
        wait.observation_id = request->observation_id;
        wait.max_age_ms = request->max_age_ms;
        wait.timeout_ms = request->timeout_ms;
        wait.started = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(wait_mu_);
        if (const auto immediate = board_.try_begin(waiters_.size(), wait, store_->published_epoch())) {
          srv::WaitSnapshot::Response response;
          response.status = immediate->status;
          service->send_response(*header, response);
          return;
        }
        waiters_.push_back(Waiter{header, request, wait});
      }, rmw_qos_profile_services_default, service_group_);
  }
  void wake_waits() {
    std::lock_guard<std::mutex> lock(wait_mu_);
    if (!wait_) return;
    const auto now = std::chrono::steady_clock::now();
    const auto now_ns = this->now().nanoseconds();
    const auto epoch = store_->published_epoch();
    std::shared_ptr<const Snapshot> latest_snapshot;
    try { latest_snapshot = store_->snapshot(); }
    catch (const std::exception&) { latest_snapshot.reset(); }
    const SnapshotIdentity* latest = latest_snapshot ? &latest_snapshot->identity() : nullptr;
    for (auto it = waiters_.begin(); it != waiters_.end();) {
      if (const auto outcome = board_.poll(it->board_request, now, now_ns, epoch, latest)) {
        srv::WaitSnapshot::Response response;
        response.status = outcome->status;
        response.age_ns = outcome->age_ns;
        if (outcome->status == "OK" && latest) {
          response.identity.epoch = latest->epoch;
          response.identity.map_revision = latest->map_revision;
          response.identity.observed_revision = latest->observed_revision;
          response.identity.last_integrated_stamp_ns = latest->last_integrated_stamp_ns;
          response.identity.geometry_hash = latest->geometry_hash;
          response.identity.config_digest = latest->config_digest;
          response.identity.clock_domain = latest->clock_domain;
        }
        wait_->send_response(*it->header, response);
        it = waiters_.erase(it);
      } else ++it;
    }
  }
  void publish_health() {
    const auto d = worker_->diagnostics();
    YAML::Node health;
    health["stamp_ns"] = now().nanoseconds();
    health["last_integrated_stamp_ns"] = d.store.identity.last_integrated_stamp_ns;
    health["geometry_hash"] = config_.geometry.geometry_hash;
    health["config_digest"] = config_.digest; health["clock_domain"] = config_.clock_domain;
    health["epoch"] = d.store.identity.epoch; health["map_revision"] = d.store.identity.map_revision;
    health["waiting"] = d.waiting; health["in_flight"] = d.in_flight;
    health["submitted"] = d.submitted; health["queue_rejected"] = d.queue_rejected;
    health["adapter_rejected"] = adapter_rejected_.load();
    health["max_queue_ms"] = d.max_queue_ms; health["tree_nodes"] = d.store.tree_nodes;
    health["known_cells"] = d.store.known_cells; health["integrated"] = d.store.integrated;
    health["rejected"] = d.store.rejected; health["duplicates"] = d.store.duplicates;
    health["capabilities"] = std::vector<std::string>{
        "OBSERVE","RESET","QUERY","CHECKPOINT","RESTORE","STATUS","WAIT","RESOURCE"};
    std_msgs::msg::String message; message.data = YAML::Dump(health); health_pub_->publish(message);
  }
  ObserverConfiguration config_;
  std::string mode_, optical_frame_, sensor_id_;
  std::unique_ptr<ObservationStore> store_;
  std::unique_ptr<SnapshotRepository> repository_;
  std::unique_ptr<ObservationWorker> worker_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::atomic<std::uint64_t> adapter_rejected_{0};
  std::atomic<bool> adapter_stop_{false};
  std::mutex adapter_mu_;
  std::condition_variable adapter_cv_;
  std::thread adapter_;
  ObservationIngress ingress_;
  ObservationLedger ledger_;
  SnapshotWaitBoard board_;
  struct Waiter {
    std::shared_ptr<rmw_request_id_t> header;
    std::shared_ptr<srv::WaitSnapshot::Request> request;
    SnapshotWaitBoard::Request board_request;
  };
  std::mutex wait_mu_;
  std::vector<Waiter> waiters_;
  rclcpp::CallbackGroup::SharedPtr service_group_, input_group_, health_group_;
  rclcpp::Publisher<msg::UpdateReceipt>::SharedPtr receipt_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::Service<srv::IntegrateObservation>::SharedPtr integrate_;
  rclcpp::Service<srv::GetSnapshot>::SharedPtr acquire_;
  rclcpp::Service<srv::ReadCells>::SharedPtr cells_;
  rclcpp::Service<srv::ReadSurface>::SharedPtr surface_;
  rclcpp::Service<srv::ReleaseSnapshot>::SharedPtr release_;
  rclcpp::Service<srv::ResetObservation>::SharedPtr reset_;
  rclcpp::Service<srv::CheckpointObservation>::SharedPtr checkpoint_;
  rclcpp::Service<srv::GetObservationStatus>::SharedPtr status_;
  rclcpp::Service<srv::GetResourceUsage>::SharedPtr resource_;
  rclcpp::Service<srv::WaitSnapshot>::SharedPtr wait_;
  rclcpp::TimerBase::SharedPtr wait_timer_;
};
}
int main(int argc, char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node = std::make_shared<uld_occ_observer::ObserverNode>();
    node->run();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; rclcpp::shutdown(); return 3;
  }
  rclcpp::shutdown(); return 0;
}
