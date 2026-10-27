#include "uld_occ_observer/configuration.hpp"
#include "uld_occ_observer/checkpoint_io.hpp"
#include "uld_occ_observer/snapshot_repository.hpp"
#include "uld_occ_observer/ros_adapters.hpp"
#include "uld_occ_observer/srv/integrate_observation.hpp"
#include "uld_occ_observer/srv/get_snapshot.hpp"
#include "uld_occ_observer/srv/read_cells.hpp"
#include "uld_occ_observer/srv/read_surface.hpp"
#include "uld_occ_observer/srv/release_snapshot.hpp"
#include "uld_occ_observer/srv/reset_observation.hpp"
#include "uld_occ_observer/srv/checkpoint_observation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <future>
#include <iostream>
#include <atomic>

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
    receipt_pub_ = create_publisher<msg::UpdateReceipt>("receipts",rclcpp::QoS(100).reliable());
    health_pub_ = create_publisher<std_msgs::msg::String>("health",rclcpp::QoS(10).reliable());
    worker_ = std::make_unique<ObservationWorker>(*store_);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    if (mode_ == "subscription") {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      rclcpp::SubscriptionOptions options; options.callback_group = input_group_;
      const auto topic = declare_parameter<std::string>("cloud_topic", "optical_points");
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic,rclcpp::QoS(2).reliable(),
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { on_cloud(*cloud); },options);
    }
    install_services();
    health_timer_ = create_wall_timer(std::chrono::milliseconds(500),[this] { publish_health(); });
    RCLCPP_INFO(get_logger(),"Ready: independent observer, OctoMap 1.9.8, input_mode=%s",mode_.c_str());
  }
  ~ObserverNode() override { worker_.reset(); }
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
          if (!worker_->enqueue(std::move(observation),[promise](CompletedObservation done) {
                promise->set_value(std::move(done)); })) throw Error("QUEUE_FULL");
          response->receipt = ros_receipt(future.get()); receipt_pub_->publish(response->receipt);
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
          const auto snapshot = repository_->get(request->handle);
          const auto& b = request->begin; const auto& e = request->end;
          const auto page = snapshot->read_cells({b[0],b[1],b[2]},{e[0],e[1],e[2]},request->cursor,request->limit);
          response->identity = ros_identity(snapshot->identity());
          response->next_cursor = page.next_cursor; response->done = page.done;
          for (const auto& cell : page.cells) {
            response->states.push_back(!cell.active ? 3 : !cell.known ? 0 : cell.value.occupied ? 2 : 1);
            response->log_odds.push_back(cell.known ? cell.value.log_odds : 0);
          }
          response->status = "OK";
        } catch (const std::exception& error) { response->status = error.what(); }
      });
    surface_ = service<srv::ReadSurface>("read_surface",
      [this](const std::shared_ptr<srv::ReadSurface::Request> request,
             std::shared_ptr<srv::ReadSurface::Response> response) {
        try {
          const auto snapshot = repository_->get(request->handle);
          const auto surface = snapshot->surface();
          if (!request->limit || request->limit > 65536 || request->cursor > surface->size())
            throw Error("INVALID_SURFACE_PAGE");
          const auto stop = request->cursor + std::min<std::uint64_t>(request->limit,surface->size()-request->cursor);
          for (auto i = request->cursor; i < stop; ++i) {
            const auto& column = surface->at(i);
            response->x.push_back(column.x); response->y.push_back(column.y);
            response->active_count.push_back(column.active_count); response->known_count.push_back(column.known_count);
            response->has_occupied.push_back(column.has_occupied); response->top_z.push_back(column.top_z);
          }
          response->identity = ros_identity(snapshot->identity());
          response->next_cursor = stop; response->done = stop == surface->size(); response->status = "OK";
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
    health["capabilities"] = std::vector<std::string>{"OBSERVE","RESET","QUERY","CHECKPOINT","RESTORE"};
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
  rclcpp::CallbackGroup::SharedPtr service_group_, input_group_;
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
};
}
int main(int argc, char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node = std::make_shared<uld_occ_observer::ObserverNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),6);
    executor.add_node(node); executor.spin();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; rclcpp::shutdown(); return 3;
  }
  rclcpp::shutdown(); return 0;
}
