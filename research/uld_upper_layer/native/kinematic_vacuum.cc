// Explicit ideal-vacuum model, equivalent to the existing sim backend's
// measured-relative-pose follow, evaluated at physics steps rather than ROS
// timer ticks. Once released, this system sends NO payload pose commands.
#include <atomic>
#include <iomanip>
#include <sstream>
#include <vector>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <google/protobuf/util/json_util.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

namespace uld_upper {
namespace sim=ignition::gazebo;
namespace comp=sim::components;
class KinematicVacuum : public sim::System, public sim::ISystemConfigure,
    public sim::ISystemPreUpdate, public sim::ISystemPostUpdate {
 public:
  void Configure(const sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
      sim::EntityComponentManager &, sim::EventManager &) override {
    payloadName=sdf->Get<std::string>("payload_model");
    parentName=sdf->Get<std::string>("parent_model");
    linkName=sdf->Get<std::string>("parent_link");
    payloadLinkName=sdf->Get<std::string>("payload_link");
    payloadCollisionName=sdf->Get<std::string>("payload_collision");
    auto service=sdf->Get<std::string>("service");
    state=transport.Advertise<ignition::msgs::StringMsg>(sdf->Get<std::string>("state_topic"));
    std::function<bool(const ignition::msgs::Boolean &,ignition::msgs::Boolean &)> callback=
      [this](const ignition::msgs::Boolean &request,ignition::msgs::Boolean &response) {
        if (!ready.load()) {response.set_data(false);return true;}
        requested.store(request.data()?1:0);response.set_data(true);return true;
      };
    transport.Advertise(service,callback);
  }
  void PreUpdate(const sim::UpdateInfo &info,sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    if (!ready.load()) {
      payload=ecm.EntityByComponents(comp::Model(),comp::Name(payloadName));
      auto parent=ecm.EntityByComponents(comp::Model(),comp::Name(parentName));
      if (payload==sim::kNullEntity || parent==sim::kNullEntity) return;
      link=sim::Model(parent).LinkByName(ecm,linkName);
      if (link==sim::kNullEntity) return;
      auto payloadLink=sim::Model(payload).LinkByName(ecm,payloadLinkName);
      if (payloadLink==sim::kNullEntity) return;
      payloadCollision=sim::Link(payloadLink).CollisionByName(ecm,payloadCollisionName);
      if (payloadCollision==sim::kNullEntity) return;
      joints.clear();
      for (int i=1;i<=6;++i) {
        auto joint=sim::Model(parent).JointByName(ecm,"elfin_joint"+std::to_string(i));
        if (joint==sim::kNullEntity) return;
        joints.push_back(joint);
        if (!ecm.Component<comp::JointPosition>(joint))
          ecm.CreateComponent(joint,comp::JointPosition());
      }
      ready.store(true);
    }
    const auto tool=sim::worldPose(link,ecm);
    const int command=requested.exchange(-1);
    if (command!=-1) ++processedCommands;
    if (command==1 && !attached) {
      // Freeze the observed relation; no target/candidate pose enters this model.
      relation=tool.Inverse()*sim::worldPose(payload,ecm);attached=true;++epoch;
    } else if (command==0) {
      attached=false;
      // Gazebo lets WorldPoseCmd survive one iteration for other systems.
      // Remove the last attached command before this detached physics step,
      // so even an old command cannot be reapplied after release.
      ecm.RemoveComponent<comp::WorldPoseCmd>(payload);
    }
    if (!attached) return;
    const auto target=tool*relation;
    auto pose=ecm.Component<comp::WorldPoseCmd>(payload);
    if (pose) pose->Data()=target;
    else ecm.CreateComponent(payload,comp::WorldPoseCmd(target));
    ++poseCommands;
  }
  void PostUpdate(const sim::UpdateInfo &info,const sim::EntityComponentManager &ecm) override {
    ignition::msgs::StringMsg message;
    std::ostringstream json;json<<std::setprecision(15)
      <<"{\"ready\":"<<(ready.load()?"true":"false")
      <<",\"attached\":"<<(attached?"true":"false")
      <<",\"pose_command_count\":"<<poseCommands<<",\"epoch\":"<<epoch
      <<",\"processed_commands\":"<<processedCommands
      <<",\"payload_pose_command_component_present\":"
      <<(ready.load() && ecm.Component<comp::WorldPoseCmd>(payload)?"true":"false")
      <<",\"sim_time_s\":"<<std::chrono::duration<double>(info.simTime).count()
      <<",\"link_T_payload_xyz\":["<<relation.Pos().X()<<","<<relation.Pos().Y()<<","<<relation.Pos().Z()
      <<"],\"link_T_payload_quat_xyzw\":["<<relation.Rot().X()<<","<<relation.Rot().Y()<<","<<relation.Rot().Z()<<","<<relation.Rot().W()<<"]";
    // Observation only: native post-step state survives any controller/pose lag.
    // Detached payload motion is never commanded by this instrumentation.
    if (ready.load()) {
      auto payloadPose=sim::worldPose(payload,ecm),linkPose=sim::worldPose(link,ecm);
      auto appendPose=[&json](const char *name,const ignition::math::Pose3d &pose) {
        json<<",\""<<name<<"\":{\"position\":{\"x\":"<<pose.Pos().X()
          <<",\"y\":"<<pose.Pos().Y()<<",\"z\":"<<pose.Pos().Z()
          <<"},\"orientation\":{\"x\":"<<pose.Rot().X()<<",\"y\":"<<pose.Rot().Y()
          <<",\"z\":"<<pose.Rot().Z()<<",\"w\":"<<pose.Rot().W()<<"}}";
      };
      appendPose("payload_pose_world",payloadPose);appendPose("link_pose_world",linkPose);
      json<<",\"q\":[";
      bool complete=true;
      for (size_t i=0;i<joints.size();++i) {
        if (i) json<<",";
        const auto component=ecm.Component<comp::JointPosition>(joints[i]);
        if (!component || component->Data().size()!=1) {json<<"null";complete=false;}
        else json<<component->Data()[0];
      }
      json<<"],\"joint_positions_complete\":"<<(complete?"true":"false");
      // Physics::UpdateCollisions writes/clears this component each step.
      // The legacy Contact sensor topic emits positive events only. Export
      // the actual component, including its empty value, with this step stamp;
      // do not synthesize "no contact" from a missing transport message.
      const auto contacts=ecm.Component<comp::ContactSensorData>(payloadCollision);
      std::string contactJson;
      bool contactAvailable=contacts && google::protobuf::util::MessageToJsonString(
          contacts->Data(),&contactJson).ok();
      json<<",\"contacts_component_available\":"<<(contactAvailable?"true":"false");
      if (contactAvailable) json<<",\"payload_contacts\":"<<contactJson;
    }
    json<<"}";message.set_data(json.str());state.Publish(message);
  }
 private:
  std::string payloadName,parentName,linkName,payloadLinkName,payloadCollisionName;
  std::vector<sim::Entity> joints;
  sim::Entity payload=sim::kNullEntity,link=sim::kNullEntity;
  sim::Entity payloadCollision=sim::kNullEntity;
  std::atomic<int> requested{-1};std::atomic<bool> ready{false};
  bool attached=false;uint64_t poseCommands=0,epoch=0,processedCommands=0;
  ignition::math::Pose3d relation;
  ignition::transport::Node transport;
  ignition::transport::Node::Publisher state;
};
}
IGNITION_ADD_PLUGIN(uld_upper::KinematicVacuum,ignition::gazebo::System,
 uld_upper::KinematicVacuum::ISystemConfigure,
 uld_upper::KinematicVacuum::ISystemPreUpdate,
 uld_upper::KinematicVacuum::ISystemPostUpdate)
