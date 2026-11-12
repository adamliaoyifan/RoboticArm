#pragma once
#include <Eigen/Geometry>
#include <deque>
#include <stdexcept>
#include <string>
#include <luggage_msgs/srv/set_observer_payload_mask.hpp>

// Single owner, serialized by the node executor. State is selected at acquisition
// time, never by whichever attachment happens to be current on cloud arrival.
class ObserverPayloadMask {
public:
  using Service = luggage_msgs::srv::SetObserverPayloadMask;
  struct State {
    int64_t stamp_ns;
    uint8_t mode;
    std::string operation_id, tool_frame;
    Eigen::Isometry3d tool_box = Eigen::Isometry3d::Identity();
    Eigen::Vector3d half_size = Eigen::Vector3d::Zero();
  };
  explicit ObserverPayloadMask(bool required) {
    history_.push_back(State{0, uint8_t(required ? 1 : 0), "", ""});
  }
  const State& at(int64_t stamp) const {
    for (auto i=history_.rbegin(); i!=history_.rend(); ++i)
      if (i->stamp_ns<=stamp) return *i;
    throw std::runtime_error("payload acquisition state retired");
  }
  void transition(const Service::Request& request, int64_t now) {
    if (request.operation_id.empty() || request.state>2 || now< history_.back().stamp_ns)
      throw std::runtime_error("invalid payload state or clock regression");
    const auto& previous=history_.back();
    if (request.state!=1 && !(history_.size()==1 && previous.operation_id.empty() && request.state==0) &&
        !(previous.mode==1 && previous.operation_id==request.operation_id))
      throw std::runtime_error("payload confirmation requires its pending barrier");
    State next{now,request.state,request.operation_id,request.tool_frame};
    if (request.state==2) {
      const auto& geometry=request.geometry;
      const auto& p=geometry.tool_conservative_translation;
      const auto& q=geometry.tool_conservative_quaternion_xyzw;
      next.tool_box.translation()=Eigen::Vector3d(p[0],p[1],p[2]);
      Eigen::Quaterniond rotation(q[3],q[0],q[1],q[2]);
      next.half_size=Eigen::Vector3d(geometry.conservative_dimensions_wdh.data())/2.;
      if (request.tool_frame.empty() || geometry.geometry_digest.empty() ||
          !next.tool_box.translation().allFinite() || !rotation.coeffs().allFinite() ||
          std::abs(rotation.squaredNorm()-1.)>1e-5 || !next.half_size.allFinite() ||
          next.half_size.minCoeff()<=0.)
        throw std::runtime_error("confirmed measured payload geometry required");
      next.tool_box.linear()=rotation.toRotationMatrix();
    }
    history_.push_back(next);
    // Late clouds outside this bounded history are refused, never reinterpreted.
    if (history_.size()>64) history_.pop_front();
  }
private:
  std::deque<State> history_;
};
