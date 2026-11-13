// Persistent transport adapter for two carriage joint setpoints. No dynamics,
// trajectory, attachment or payload commands are implemented in this process.
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <ignition/msgs/double.pb.h>
#include <ignition/transport/Node.hh>

int main(int argc,char **argv) {
  if (argc!=4) {
    std::cerr<<"usage: joint_command_bridge X_TOPIC Z_TOPIC CONNECT_TIMEOUT_MS\n";
    return 2;
  }
  long long timeout;
  try {
    size_t consumed=0;
    timeout=std::stoll(argv[3],&consumed);
    if (consumed!=std::string(argv[3]).size() || timeout<=0) return 2;
  } catch (...) {return 2;}
  ignition::transport::Node node;
  auto x=node.Advertise<ignition::msgs::Double>(argv[1]);
  auto z=node.Advertise<ignition::msgs::Double>(argv[2]);
  if (!x || !z) return 2;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout);
  while (!x.HasConnections() || !z.HasConnections()) {
    if (std::chrono::steady_clock::now()>=deadline) {
      std::cerr<<"JOINT_COMMAND_SUBSCRIBER_TIMEOUT\n";
      return 3;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::cout<<"{\"ready\":true}"<<std::endl;
  std::string line;
  unsigned long long previous=0;
  while (std::getline(std::cin,line)) {
    std::istringstream input(line);
    unsigned long long sequence;
    double first,second;
    std::string extra;
    if (!(input>>sequence>>first>>second) || (input>>extra) ||
        sequence!=previous+1 || !std::isfinite(first) || !std::isfinite(second)) {
      std::cerr<<"INVALID_JOINT_COMMAND\n";
      return 2;
    }
    ignition::msgs::Double a,b;a.set_data(first);b.set_data(second);
    const bool sentX=x.Publish(a),sentZ=z.Publish(b);
    if (!sentX || !sentZ) {
      std::cerr<<"JOINT_COMMAND_PUBLISH_FAILED\n";
      return 4;
    }
    previous=sequence;
    // This acknowledges publication, never controller receipt or tracking.
    std::cout<<"{\"sequence\":"<<sequence<<",\"published\":true}"<<std::endl;
  }
  return 0;
}
