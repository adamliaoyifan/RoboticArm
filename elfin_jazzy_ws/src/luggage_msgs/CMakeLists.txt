cmake_minimum_required(VERSION 3.8)
project(luggage_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedLuggage.msg"
  "msg/SlotSpec.msg"
  "msg/MotionSegment.msg"
  "msg/LoadTaskStatus.msg"
  "msg/ContainerOpeningEstimate.msg"
  "srv/DetectLuggage.srv"
  "srv/GetCurrentBox.srv"
  "srv/SpawnNextBox.srv"
  "srv/ClearCurrentBox.srv"
  "srv/FinalizeCurrentBox.srv"
  "srv/VerifyPlacedBox.srv"
  "srv/GetNextSlot.srv"
  "srv/ComputePlacement.srv"
  "srv/VacuumCommand.srv"
  "srv/SyncStaticScene.srv"
  "srv/SyncPickupBox.srv"
  "srv/AddPlacedBox.srv"
  "srv/RemovePlacedBox.srv"
  "srv/BuildMotionSequence.srv"
  "srv/InspectContainer.srv"
  "srv/ResetCargoMap.srv"
  "srv/IntegrateCargoView.srv"
  "srv/GetCargoMapStats.srv"
  "srv/EvaluateCargoViews.srv"
  "srv/OrchestratorStep.srv"
  "action/PlanMotion.action"
  "action/GoToRobotPose.action"
  "action/GoToJointValues.action"
  "action/AimCameraAtContainer.action"
  "action/ValidateMotionSequence.action"
  "action/PlanNextCargoView.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
