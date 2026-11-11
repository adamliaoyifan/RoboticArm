// Explicit ideal vacuum: native fixed constraint at the measured relation.
// The physics engine carries mass and momentum; no payload poses, velocities,
// forces, gravity changes or collision-mask changes are commanded here.
#include <atomic>
#include <iomanip>
#include <sstream>
#include <set>
#include <stdexcept>
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
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <google/protobuf/util/json_util.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

namespace uld_upper {
namespace sim=ignition::gazebo;
namespace comp=sim::components;
class IdealVacuum : public sim::System, public sim::ISystemConfigure,
    public sim::ISystemPreUpdate, public sim::ISystemPostUpdate {
 public:
  void Configure(const sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
      sim::EntityComponentManager &, sim::EventManager &) override {
    payloadName=sdf->Get<std::string>("payload_model");
    parentName=sdf->Get<std::string>("parent_model");
    linkName=sdf->Get<std::string>("parent_link");
    payloadLinkName=sdf->Get<std::string>("payload_link");
    payloadCollisionName=sdf->Get<std::string>("payload_collision");
    // An explicit joint list supports the independent release carriage. Keep
    // the six-arm default byte-for-byte in name/order for existing fixtures.
    if (sdf->HasElement("joint_name")) {
      for (auto node=sdf->FindElement("joint_name");node;
           node=node->GetNextElement("joint_name")) jointNames.push_back(node->Get<std::string>());
    } else {
      for (int i=1;i<=6;++i) jointNames.push_back("elfin_joint"+std::to_string(i));
    }
    std::set<std::string> unique;
    for (const auto &name:jointNames)
      if (name.empty() || !unique.insert(name).second)
        throw std::invalid_argument("INVALID_VACUUM_JOINT_NAMES");
    if (sdf->HasElement("observed_model")) {
      unique.clear();
      for (auto node=sdf->FindElement("observed_model");node;
           node=node->GetNextElement("observed_model")) {
        auto name=node->Get<std::string>();
        if (name.empty() || !unique.insert(name).second)
          throw std::invalid_argument("INVALID_VACUUM_OBSERVED_MODELS");
        observedNames.push_back(name);
      }
    }
    // Fixture-only initial condition: establish the native constraint before
    // the first physics step, so below-policy calibration cases do not fall
    // while a transport subscriber is discovering the state topic.
    if (sdf->Get<bool>("initially_attached",false).first) requested.store(1);
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
      payloadLink=sim::Model(payload).LinkByName(ecm,payloadLinkName);
      if (payloadLink==sim::kNullEntity) return;
      payloadCollision=sim::Link(payloadLink).CollisionByName(ecm,payloadCollisionName);
      if (payloadCollision==sim::kNullEntity) return;
      joints.clear();
      for (const auto &name:jointNames) {
        auto joint=sim::Model(parent).JointByName(ecm,name);
        if (joint==sim::kNullEntity) return;
        joints.push_back(joint);
        if (!ecm.Component<comp::JointPosition>(joint))
          ecm.CreateComponent(joint,comp::JointPosition());
      }
      observedModels.clear();
      for (const auto &name:observedNames) {
        auto model=ecm.EntityByComponents(comp::Model(),comp::Name(name));
        if (model==sim::kNullEntity) return;
        observedModels.push_back(model);
      }
      ready.store(true);
    }
    // A queued entity removal is not a completed physics detach. Keep the
    // identity and attached acknowledgement until the next step observes the
    // native component gone; never remove the component before EachRemoved.
    if (detachPending && !ecm.Component<comp::DetachableJoint>(attachmentJoint)) {
      attached=false;detachPending=false;attachmentJoint=sim::kNullEntity;
      ++attachmentDetachCount;
    }
    const auto tool=sim::worldPose(link,ecm);
    const int command=requested.exchange(-1);
    if (command!=-1) ++processedCommands;
    if (command==1 && !attached) {
      // Freeze the observed relation; no target/candidate pose enters this model.
      relation=tool.Inverse()*sim::worldPose(payload,ecm);
      attachmentJoint=ecm.CreateEntity();
      ecm.CreateComponent(attachmentJoint,comp::DetachableJoint(
          comp::DetachableJointInfo{link,payloadLink,"fixed"}));
      attached=true;++epoch;++attachmentCreateCount;
    } else if (command==0 && !detachPending) {
      if (attachmentJoint!=sim::kNullEntity) {
        // Same ownership protocol as Gazebo's DetachableJoint system:
        // RequestRemoveEntity alone preserves removed-view component data.
        ecm.RequestRemoveEntity(attachmentJoint);
        detachPending=true;
      } else attached=false;
    }
  }
  void PostUpdate(const sim::UpdateInfo &info,const sim::EntityComponentManager &ecm) override {
    ignition::msgs::StringMsg message;
    std::ostringstream json;json<<std::setprecision(15)
      <<"{\"ready\":"<<(ready.load()?"true":"false")
      <<",\"attached\":"<<(attached?"true":"false")
      <<",\"pose_command_count\":"<<poseCommands<<",\"epoch\":"<<epoch
      <<",\"processed_commands\":"<<processedCommands
      <<",\"attachment_model\":\"native_detachable_fixed_joint\""
      <<",\"attachment_create_count\":"<<attachmentCreateCount
      <<",\"attachment_detach_count\":"<<attachmentDetachCount
      <<",\"attachment_joint_component_present\":"
      <<(attachmentJoint!=sim::kNullEntity && ecm.Component<comp::DetachableJoint>(attachmentJoint)?"true":"false")
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
      json<<",\"joint_names\":[";
      for (size_t i=0;i<jointNames.size();++i) {
        if (i) json<<",";
        json<<"\""<<jointNames[i]<<"\"";
      }
      json<<"]";
      if (!observedNames.empty()) {
        json<<",\"observed_model_poses_world\": {";
        bool first=true;
        for (size_t i=0;i<observedNames.size();++i) {
          // All lower poses and contacts carry this same native step stamp.
          if (!first) json<<",";
          first=false;
          json<<"\""<<observedNames[i]<<"\":{\"valid\":true";
          appendPose("pose",sim::worldPose(observedModels[i],ecm));
          json<<"}";
        }
        json<<"}";
      }
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
  std::vector<std::string> jointNames,observedNames;
  std::vector<sim::Entity> observedModels;
  sim::Entity payload=sim::kNullEntity,link=sim::kNullEntity;
  sim::Entity payloadLink=sim::kNullEntity,attachmentJoint=sim::kNullEntity;
  sim::Entity payloadCollision=sim::kNullEntity;
  std::atomic<int> requested{-1};std::atomic<bool> ready{false};
  bool attached=false,detachPending=false;uint64_t poseCommands=0,epoch=0,processedCommands=0;
  uint64_t attachmentCreateCount=0,attachmentDetachCount=0;
  ignition::math::Pose3d relation;
  ignition::transport::Node transport;
  ignition::transport::Node::Publisher state;
};
}
IGNITION_ADD_PLUGIN(uld_upper::IdealVacuum,ignition::gazebo::System,
 uld_upper::IdealVacuum::ISystemConfigure,
 uld_upper::IdealVacuum::ISystemPreUpdate,
 uld_upper::IdealVacuum::ISystemPostUpdate)
