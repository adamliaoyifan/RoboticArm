cmake_minimum_required(VERSION 3.0.2)
project(luggage_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
)

add_message_files(
  FILES
  DetectedLuggage.msg
  SlotSpec.msg
  MotionSegment.msg
  LoadTaskStatus.msg
)

add_service_files(
  FILES
  DetectLuggage.srv
  GetNextSlot.srv
  PlanMotion.srv
  VacuumCommand.srv
  SyncStaticScene.srv
  AddPlacedBox.srv
  BuildMotionSequence.srv
  GoToRobotPose.srv
  GoToJointValues.srv
  AimCameraAtContainer.srv
  InspectContainer.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)
