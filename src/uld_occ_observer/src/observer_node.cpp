#include "uld_occ_observer/configuration.hpp"
#include "uld_occ_observer/checkpoint_io.hpp"
#include "uld_occ_observer/snapshot_repository.hpp"
#include "uld_occ_observer/ros_adapters.hpp"
#include "uld_occ_observer/observation_ingress.hpp"
#include "uld_occ_observer/observation_ledger.hpp"
#include "uld_occ_observer/observation_terminal.hpp"
#include "uld_occ_observer/interface_trace.hpp"
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
#include "uld_occ_observer/srv/set_behavior_hook.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <iostream>
#include <iterator>
#include <mutex>
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
    behavior_hooks_ = declare_parameter<bool>("behavior_test_hooks", false);
    if (behavior_hooks_) install_behavior_hooks();
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
    interface_trace_configure_from_env();
    RCLCPP_INFO(get_logger(),"Ready: independent observer, OctoMap 1.9.8, input_mode=%s",mode_.c_str());
  }
  ~ObserverNode() override {
    hold_integrate_.store(false);
    adapter_stop_.store(true);
    adapter_pause_.store(false);
    adapter_cv_.notify_all();
    if (adapter_.joinable()) adapter_.join();
    worker_.reset();
    interface_trace_flush();
  }
  void run() {
    rclcpp::executors::SingleThreadedExecutor input_executor;
    input_executor.add_callback_group(input_group_, get_node_base_interface());
    std::thread input_thread([&] { input_executor.spin(); });
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_callback_group(service_group_, get_node_base_interface());
    executor.add_callback_group(health_group_, get_node_base_interface());
    // Parameter services stay on the default group. The behavior test uses them
    // to pause the adapter, hold integrate, and shorten receipt retention.
    executor.add_callback_group(get_node_base_interface()->get_default_callback_group(),
                                get_node_base_interface());
    executor.spin();
    input_executor.cancel();
    input_thread.join();
  }
 private:
  struct Waiter {
    std::shared_ptr<rmw_request_id_t> header;
    SnapshotWaitBoard::Request board_request;
  };
  void install_behavior_hooks() {
    adapter_pause_.store(declare_parameter<bool>("adapter_paused", false));
    hold_integrate_.store(declare_parameter<bool>("hold_integrate", false));
    const auto retention_ms = declare_parameter<int>("receipt_retention_ms", 120000);
    ledger_.set_retention(std::chrono::milliseconds(retention_ms < 0 ? 0 : retention_ms));
    // The probe runs while integrate holds the store mutex. It only watches an
    // atomic, so health and wait timeouts can still run on their own threads.
    store_->set_lock_probe_for_test([this] {
      while (hold_integrate_.load(std::memory_order_acquire))
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    });
    param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto& param : params) {
        if (param.get_name() == "adapter_paused") {
          adapter_pause_.store(param.as_bool());
          adapter_cv_.notify_all();
        } else if (param.get_name() == "hold_integrate") {
          hold_integrate_.store(param.as_bool());
        } else if (param.get_name() == "receipt_retention_ms") {
          ledger_.set_retention(std::chrono::milliseconds(std::max<std::int64_t>(0, param.as_int())));
        }
      }
      return result;
    });
  }
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
  msg::UpdateReceipt publish_message(msg::UpdateReceipt receipt) {
    receipt_pub_->publish(receipt);
    return receipt;
  }
  msg::UpdateReceipt publish_sealed(const SealedTerminal& sealed) {
    return publish_message(ros_receipt(CompletedObservation{sealed.receipt, sealed.queue_ms}));
  }
  msg::UpdateReceipt publish_open(const std::string& sensor, const std::string& id,
                                  const std::string& status, std::int64_t stamp,
                                  std::int64_t enter_ns = 0) {
    msg::UpdateReceipt receipt;
    receipt.status = status;
    receipt.sensor_id = sensor;
    receipt.observation_id = id;
    receipt.input_stamp_ns = stamp;
    receipt.callback_enter_ns = enter_ns;
    return publish_message(receipt);
  }
  void reject_subscribed(std::uint64_t epoch, const std::string& id, const std::string& status,
                         std::int64_t stamp, bool stored, std::int64_t enter_ns = 0) {
    ++adapter_rejected_;
    if (stored) {
      UpdateReceipt receipt;
      receipt.status = status;
      receipt.sensor_id = sensor_id_;
      receipt.observation_id = id;
      receipt.input_stamp_ns = stamp;
      const auto entered = enter_ns != 0 ? enter_ns
          : ledger_entered(epoch, id);
      if (entered != 0) receipt.callback_enter_ns = entered;
      const auto sealed = seal_inflight(ledger_, board_, epoch, sensor_id_, id, std::move(receipt),
                                        std::chrono::steady_clock::now());
      if (sealed.stored) {
        publish_sealed(sealed);
        return;
      }
    } else if (status != "DUPLICATE") {
      note_unstored_rejection(board_, epoch, sensor_id_, id, status, stamp);
    }
    publish_open(sensor_id_, id, status, stamp, enter_ns);
  }
  // The admission entry stamp recorded for this id, if the ledger kept it.
  std::int64_t ledger_entered(std::uint64_t epoch, const std::string& id) {
    const auto record = ledger_.query(epoch, sensor_id_, id);
    return record ? record->entered_ns : 0;
  }
  void on_cloud(const sensor_msgs::msg::PointCloud2& cloud) {
    // First statement: the subscription callback entry moment, CLOCK_MONOTONIC.
    const auto enter_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (cloud.header.frame_id != optical_frame_) {
      ++adapter_rejected_;
      publish_open("", "", "OPTICAL_FRAME_MISMATCH", rclcpp::Time(cloud.header.stamp).nanoseconds(),
                   enter_ns);
      return;
    }
    const auto stamp = rclcpp::Time(cloud.header.stamp).nanoseconds();
    const auto id = std::to_string(stamp);
    const auto epoch = store_->published_epoch();
    const auto now = std::chrono::steady_clock::now();
    const auto admitted = ledger_.admit(epoch, sensor_id_, id, now, enter_ns);
    if (!admitted.accepted) {
      reject_subscribed(epoch, id, admitted.status, stamp, false, enter_ns);
      return;
    }
    ObservationIngress::Item item;
    item.sensor_id = sensor_id_;
    item.observation_id = id;
    item.epoch = epoch;
    item.payload = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud);
    item.payload_bytes = cloud.data.size();
    const auto queued = ingress_.try_admit(std::move(item));
    if (!queued.accepted) {
      reject_subscribed(epoch, id, queued.status, stamp, true, enter_ns);
      return;
    }
    adapter_cv_.notify_one();
  }
  void adapt_loop() {
    while (!adapter_stop_.load()) {
      {
        std::unique_lock<std::mutex> lock(adapter_mu_);
        if (adapter_pause_.load()) {
          adapter_cv_.wait(lock, [&] { return adapter_stop_.load() || !adapter_pause_.load(); });
          continue;
        }
      }
      if (adapter_stop_.load() || adapter_pause_.load()) continue;
      auto item = ingress_.try_pop();
      if (!item) {
        std::unique_lock<std::mutex> lock(adapter_mu_);
        adapter_cv_.wait_for(lock, std::chrono::milliseconds(5), [&] {
          return adapter_stop_.load() || adapter_pause_.load() || ingress_.size() > 0;
        });
        continue;
      }
      adapt(*item);
    }
  }
  void adapt(const ObservationIngress::Item& item) {
    const auto reject = [&](const std::string& status, std::int64_t stamp) {
      reject_subscribed(item.epoch, item.observation_id, status, stamp, true);
    };
    if (item.epoch != store_->published_epoch()) {
      reject("EPOCH_MISMATCH", 0);
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    ledger_.mark(item.epoch, item.sensor_id, item.observation_id, "PROCESSING", true, now);
    try {
      const auto cloud = std::static_pointer_cast<sensor_msgs::msg::PointCloud2>(item.payload);
      const auto tf = tf_buffer_->lookupTransform("container_link", cloud->header.frame_id,
          rclcpp::Time(cloud->header.stamp), rclcpp::Duration::from_seconds(.1));
      if (item.epoch != store_->published_epoch()) {
        reject("EPOCH_MISMATCH", rclcpp::Time(cloud->header.stamp).nanoseconds());
        return;
      }
      auto observation = convert(*cloud, tf, item.sensor_id, item.observation_id, config_.clock_domain,
          config_.geometry.geometry_hash, item.epoch);
      const auto stamp = observation.identity.stamp_ns;
      const auto epoch = item.epoch;
      const auto sensor = item.sensor_id;
      const auto id = item.observation_id;
      if (!worker_->enqueue(std::move(observation), [this, epoch, sensor, id](CompletedObservation done) {
            finish_receipt(epoch, sensor, id, std::move(done)); }))
        reject("QUEUE_FULL", stamp);
    } catch (const std::exception& error) {
      reject(error.what(), 0);
    }
  }
  void finish_receipt(std::uint64_t epoch, const std::string& sensor, const std::string& id,
                      CompletedObservation done) {
    const auto sealed = seal_inflight(ledger_, board_, epoch, sensor, id, std::move(done.receipt),
                                      std::chrono::steady_clock::now(), done.queue_ms);
    if (sealed.stored) publish_sealed(sealed);
  }
  // False means the store mutex is busy and the waiter must be queued again.
  // The response has not been sent.
  bool send_wait(const Waiter& waiter, const SnapshotWaitBoard::Outcome& outcome,
                 const SnapshotIdentity* latest) {
    // The wait mutex is not held. acquire() registers a real lease and does not
    // wait on the store lock while holding the handle table.
    srv::WaitSnapshot::Response response;
    response.status = outcome.status;
    response.reason = outcome.reason;
    response.age_ns = outcome.age_ns;
    if (outcome.status == "OK") {
      try {
        if (waiter.board_request.max_age_ms < 0 || waiter.board_request.max_age_ms > 86400000)
          throw Error("INVALID_MAX_AGE");
        const auto deadline = waiter.board_request.started +
            std::chrono::milliseconds(std::max<std::int64_t>(0, waiter.board_request.timeout_ms));
        if (std::chrono::steady_clock::now() >= deadline) {
          response.status = "TIMEOUT";
          response.handle.clear();
        } else {
          const auto now_ns = now().nanoseconds();
          const auto lease = repository_->try_acquire(
              now_ns, waiter.board_request.max_age_ms * 1000000LL, false,
              waiter.board_request.expected_epoch, std::nullopt);
          if (!lease) return false;
          if (std::chrono::steady_clock::now() >= deadline) {
            repository_->release(lease->handle);
            response.status = "TIMEOUT";
            response.handle.clear();
          } else if (lease->identity.epoch != waiter.board_request.expected_epoch ||
                     lease->identity.observed_revision < outcome.observed_revision) {
            repository_->release(lease->handle);
            response.status = "REVISION_NOT_COVERED";
            response.handle.clear();
          } else {
            response.status = "OK";
            response.handle = lease->handle;
            response.identity = ros_identity(lease->identity);
            response.age_ns = lease->age_ns;
          }
        }
      } catch (const std::exception& error) {
        response.status = error.what();
        response.handle.clear();
      }
    } else if (latest) {
      response.identity = ros_identity(*latest);
    }
    wait_->send_response(*waiter.header, response);
    return true;
  }
  template<class Service, class Callback>
  typename rclcpp::Service<Service>::SharedPtr service(const std::string& name, Callback callback) {
    return create_service<Service>(name,callback,rmw_qos_profile_services_default,service_group_);
  }
  template <class Values>
  static std::uint64_t reserved_bytes(const Values& values) {
    return static_cast<std::uint64_t>(values.capacity()) * sizeof(typename Values::value_type);
  }
  void install_services() {
    integrate_ = service<srv::IntegrateObservation>("integrate_observation",
      [this](const std::shared_ptr<srv::IntegrateObservation::Request> request,
             std::shared_ptr<srv::IntegrateObservation::Response> response) {
        const auto epoch = request->epoch;
        const auto sensor = request->sensor_id;
        const auto id = request->observation_id;
        const auto stamp = rclcpp::Time(request->cloud.header.stamp).nanoseconds();
        bool admitted = false;
        try {
          if (mode_ != "service") throw Error("INPUT_MODE_MISMATCH");
          auto observation = convert(request->cloud,request->uld_from_optical,sensor,id,
              request->clock_domain,request->geometry_hash,epoch);
          const auto now = std::chrono::steady_clock::now();
          const auto decision = ledger_.admit(epoch, sensor, id, now);
          if (!decision.accepted) {
            if (decision.status != "DUPLICATE")
              note_unstored_rejection(board_, epoch, sensor, id, decision.status, stamp);
            response->receipt = publish_open(sensor, id, decision.status, stamp);
            return;
          }
          admitted = true;
          ledger_.mark(epoch, sensor, id, "PROCESSING", true, now);
          auto promise = std::make_shared<std::promise<CompletedObservation>>();
          auto future = promise->get_future();
          if (!worker_->enqueue(std::move(observation), [promise](CompletedObservation done) {
                promise->set_value(std::move(done)); })) {
            UpdateReceipt receipt;
            receipt.status = "QUEUE_FULL";
            receipt.sensor_id = sensor;
            receipt.observation_id = id;
            receipt.input_stamp_ns = stamp;
            const auto sealed = seal_inflight(ledger_, board_, epoch, sensor, id, std::move(receipt),
                                              std::chrono::steady_clock::now());
            response->receipt = sealed.stored ? publish_sealed(sealed)
                                              : publish_open(sensor, id, "QUEUE_FULL", stamp);
            return;
          }
          const auto done = future.get();
          const auto sealed = seal_inflight(ledger_, board_, epoch, sensor, id, done.receipt,
                                            std::chrono::steady_clock::now(), done.queue_ms);
          response->receipt = sealed.stored ? publish_sealed(sealed)
                                            : publish_open(sensor, id, done.receipt.status, stamp);
        } catch (const std::exception& error) {
          if (admitted) {
            UpdateReceipt receipt;
            receipt.status = error.what();
            receipt.sensor_id = sensor;
            receipt.observation_id = id;
            receipt.input_stamp_ns = stamp;
            const auto sealed = seal_inflight(ledger_, board_, epoch, sensor, id, std::move(receipt),
                                              std::chrono::steady_clock::now());
            response->receipt = sealed.stored ? publish_sealed(sealed)
                                              : publish_open(sensor, id, error.what(), stamp);
            return;
          }
          if (!id.empty())
            note_unstored_rejection(board_, epoch, sensor, id, error.what(), stamp);
          response->receipt = publish_open(sensor, id, error.what(), stamp);
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
        InflightResponse inflight;
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
          inflight = repository_->begin_response(reserved_bytes(response->states) + reserved_bytes(response->log_odds));
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
        InflightResponse inflight;
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
          const auto fill_started = std::chrono::steady_clock::now();
          for (auto i = request->cursor; i < stop; ++i) {
            const auto& column = surface->at(i);
            response->x.push_back(column.x); response->y.push_back(column.y);
            response->active_count.push_back(column.active_count); response->known_count.push_back(column.known_count);
            response->has_occupied.push_back(column.has_occupied); response->top_z.push_back(column.top_z);
          }
          inflight = repository_->begin_response(
              reserved_bytes(response->x) + reserved_bytes(response->y) +
              reserved_bytes(response->active_count) + reserved_bytes(response->known_count) +
              reserved_bytes(response->has_occupied) + reserved_bytes(response->top_z));
          response->response_fill_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - fill_started).count();
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
          worker_->refresh_store_diagnostics();
          board_.drop_other_epochs(response->epoch);
          ledger_.drop_terminal_other_epochs(response->epoch);
          response->status = "OK";
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
        const auto now = std::chrono::steady_clock::now();
        ledger_.expire(now);
        const auto record = ledger_.query(request->epoch, request->sensor_id, request->observation_id);
        if (record) {
          response->status = record->state;
          response->receipt = ros_receipt(CompletedObservation{record->receipt, record->queue_ms});
          return;
        }
        if (const auto completion = board_.find(request->epoch, request->sensor_id, request->observation_id)) {
          if (completion->status != "COMPLETED") {
            response->status = "REJECTED";
            response->receipt.status = completion->reason.empty() ? completion->status : completion->reason;
            response->receipt.sensor_id = request->sensor_id;
            response->receipt.observation_id = request->observation_id;
            response->receipt.input_stamp_ns = completion->input_stamp_ns;
            return;
          }
        }
        response->status = "NOT_SEEN";
      });
    resource_ = service<srv::GetResourceUsage>("get_resource_usage",
      [this](const std::shared_ptr<srv::GetResourceUsage::Request> request,
             std::shared_ptr<srv::GetResourceUsage::Response> response) {
        (void)request;
        ResourceAccounts accounts;
        accounts.entry_messages = ingress_.size();
        accounts.entry_message_peak = ingress_.peak();
        accounts.entry_payload_bytes = ingress_.payload_bytes();
        accounts.entry_payload_bytes_peak = ingress_.payload_peak();
        accounts.entry_queue_allocated_bytes = ingress_.allocated_bytes();
        accounts.ledger_records = ledger_.size();
        accounts.ledger_record_peak = ledger_.peak();
        accounts.ledger_bytes = ledger_.accounted_bytes();
        accounts.ledger_bytes_peak = ledger_.accounted_bytes_peak();
        accounts.completion_records = board_.size();
        accounts.completion_records_peak = board_.peak();
        accounts.completion_bytes = board_.accounted_bytes();
        accounts.completion_bytes_peak = board_.accounted_bytes_peak();
        repository_->account(accounts);
        const auto usage = repository_->usage();
        response->status = "OK";
        response->shared_block_bytes = usage.shared_block_bytes;
        response->shared_block_bytes_peak = usage.shared_block_bytes_peak;
        response->old_view_block_bytes = usage.old_view_block_bytes;
        response->view_directory_entries = usage.view_directory_entries;
        response->view_directory_entries_peak = usage.view_directory_entries_peak;
        response->view_directory_bytes = usage.view_directory_bytes;
        response->view_directory_bytes_peak = usage.view_directory_bytes_peak;
        response->surface_cache_bytes = usage.surface_cache_bytes;
        response->surface_cache_bytes_peak = usage.surface_cache_bytes_peak;
        response->inflight_response_bytes = usage.inflight_response_bytes;
        response->inflight_response_bytes_peak = usage.inflight_response_bytes_peak;
        response->entry_queue_messages = usage.entry_queue_messages;
        response->entry_queue_messages_peak = usage.entry_queue_messages_peak;
        response->entry_queue_payload_bytes = usage.entry_queue_payload_bytes;
        response->entry_queue_payload_bytes_peak = usage.entry_queue_payload_bytes_peak;
        response->receipt_ledger_records = usage.receipt_ledger_records;
        response->receipt_ledger_records_peak = usage.receipt_ledger_records_peak;
        response->receipt_ledger_bytes = usage.receipt_ledger_bytes;
        response->receipt_ledger_bytes_peak = usage.receipt_ledger_bytes_peak;
        response->completion_records = usage.completion_records;
        response->completion_records_peak = usage.completion_records_peak;
        response->read_view_bytes = usage.read_view_bytes;
        response->read_view_bytes_peak = usage.read_view_bytes_peak;
        response->cache_bytes = usage.cache_bytes;
        response->cache_bytes_peak = usage.cache_bytes_peak;
        response->read_view_alloc_bytes = usage.read_view_alloc_bytes;
        response->cache_alloc_bytes = usage.cache_alloc_bytes;
        response->entry_queue_allocated_bytes = accounts.entry_queue_allocated_bytes;
        response->store_tree_bytes = usage.store_tree_bytes;
        response->uncovered_allocations = usage.uncovered_allocations;
        response->certification = usage.certification;
        response->uncovered_classes = usage.uncovered_classes;
      });
    if (behavior_hooks_) {
      hook_ = service<srv::SetBehaviorHook>("set_behavior_hook",
        [this](const std::shared_ptr<srv::SetBehaviorHook::Request> request,
               std::shared_ptr<srv::SetBehaviorHook::Response> response) {
          if (request->name == "receipt_retention_ms") {
            ledger_.set_retention(std::chrono::milliseconds(request->value < 0 ? 0 : request->value));
          } else if (request->name == "hold_integrate") {
            hold_integrate_.store(request->value != 0);
          } else if (request->name == "adapter_paused") {
            adapter_pause_.store(request->value != 0);
            adapter_cv_.notify_all();
          } else if (request->name == "wait_phase") {
            wait_phase_.store(static_cast<int>(request->value));
          } else if (request->name == "wait_accounting") {
            std::lock_guard<std::mutex> lock(wait_mu_);
            response->accepted = true;
            response->status = "checking=" + std::to_string(waits_checking_) +
                " queued=" + std::to_string(waiters_.size());
            return;
          } else {
            response->accepted = false;
            response->status = "UNKNOWN_HOOK";
            return;
          }
          response->accepted = true;
          response->status = "OK";
        });
    }
    wait_ = create_service<srv::WaitSnapshot>("wait_snapshot",
      [this](const std::shared_ptr<rclcpp::Service<srv::WaitSnapshot>> service,
             const std::shared_ptr<rmw_request_id_t> header,
             const std::shared_ptr<srv::WaitSnapshot::Request> request) {
        (void)service;
        SnapshotWaitBoard::Request wait;
        wait.expected_epoch = request->expected_epoch;
        wait.sensor_id = request->sensor_id;
        wait.observation_id = request->observation_id;
        wait.max_age_ms = request->max_age_ms;
        wait.timeout_ms = request->timeout_ms;
        wait.started = std::chrono::steady_clock::now();
        Waiter waiter{header, wait};
        std::unique_lock<std::mutex> lock(wait_mu_);
        const auto epoch = store_->published_epoch();
        if (const auto immediate = board_.try_begin(waiters_.size() + waits_checking_, wait, epoch)) {
          const auto outcome = *immediate;
          lock.unlock();
          send_wait(waiter, outcome, nullptr);
          return;
        }
        const auto latest = worker_->diagnostics().store.identity;
        if (const auto ready = board_.poll(wait, wait.started, this->now().nanoseconds(), epoch, &latest)) {
          const auto outcome = *ready;
          // Count the lease attempt itself. A store lock that is busy must not
          // leave this request outside the 8-wait limit.
          const bool leasing = outcome.status == "OK";
          if (leasing) ++waits_checking_;
          lock.unlock();
          const bool sent = send_wait(waiter, outcome, outcome.status == "OK" ? &latest : nullptr);
          std::lock_guard<std::mutex> guard(wait_mu_);
          if (leasing && waits_checking_) --waits_checking_;
          if (!sent) waiters_.push_back(std::move(waiter));
          return;
        }
        waiters_.push_back(std::move(waiter));
      }, rmw_qos_profile_services_default, service_group_);
  }
  void wake_waits() {
    // Phase 1 leaves every queued waiter in place so a test can fill all 8
    // before one check begins. Phase 2 swaps that batch and holds it there.
    if (behavior_hooks_ && wait_phase_.load() == 1) return;
    std::vector<Waiter> pending;
    {
      std::lock_guard<std::mutex> lock(wait_mu_);
      if (waiters_.empty() || !wait_) return;
      pending.swap(waiters_);
      waits_checking_ += pending.size();
    }
    struct ReleaseChecking {
      ObserverNode* self;
      std::size_t count;
      ~ReleaseChecking() {
        if (!count) return;
        std::lock_guard<std::mutex> lock(self->wait_mu_);
        self->waits_checking_ -= count;
      }
    } release{this, pending.size()};
    if (behavior_hooks_ && wait_phase_.load() == 2) {
      while (wait_phase_.load() == 2)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto now = std::chrono::steady_clock::now();
    const auto epoch = store_->published_epoch();
    std::vector<Waiter> need_revision;
    std::vector<Waiter> still;
    for (auto& waiter : pending) {
      if (const auto outcome = board_.poll(waiter.board_request, now, 0, epoch, nullptr)) {
        if (!send_wait(waiter, *outcome, nullptr)) still.push_back(std::move(waiter));
      } else need_revision.push_back(std::move(waiter));
    }
    if (!need_revision.empty()) {
      const auto now_ns = this->now().nanoseconds();
      const auto latest = worker_->diagnostics().store.identity;
      for (auto& waiter : need_revision) {
        if (const auto outcome = board_.poll(waiter.board_request, now, now_ns, epoch, &latest)) {
          if (!send_wait(waiter, *outcome, outcome->status == "OK" ? &latest : nullptr))
            still.push_back(std::move(waiter));
        } else still.push_back(std::move(waiter));
      }
    }
    {
      std::lock_guard<std::mutex> lock(wait_mu_);
      waits_checking_ -= release.count;
      release.count = 0;
      still.insert(still.end(), std::make_move_iterator(waiters_.begin()),
                   std::make_move_iterator(waiters_.end()));
      waiters_.swap(still);
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
  std::atomic<bool> adapter_pause_{false};
  std::atomic<bool> hold_integrate_{false};
  bool behavior_hooks_{false};
  std::mutex adapter_mu_;
  std::condition_variable adapter_cv_;
  std::thread adapter_;
  ObservationIngress ingress_;
  ObservationLedger ledger_;
  SnapshotWaitBoard board_;
  std::mutex wait_mu_;
  std::vector<Waiter> waiters_;
  std::size_t waits_checking_{0};
  std::atomic<int> wait_phase_{0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
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
  rclcpp::Service<srv::SetBehaviorHook>::SharedPtr hook_;
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
