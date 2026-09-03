cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AprilTagDetection.msg"
  "msg/AprilTagDetectionArray.msg"
  "msg/AprilTagStampedDetection.msg"
  "msg/CargoSlotState.msg"
  "msg/ManipulationFeedback.msg"
  "msg/ManipulationResult.msg"
  "msg/ManipulationState.msg"
  "msg/WheelCommand.msg"
  "srv/ManageManipulationState.srv"
  "action/AnalyzeAprilTags.action"
  "action/ExecuteMission.action"
  "action/FollowWall.action"
  "action/PickObject.action"
  "action/PlaceAtPose.action"
  "action/PlaceInContainer.action"
  "action/PlaceOnShelf.action"
  "action/PlaceOnTable.action"
  "action/PrepareManipulator.action"
  "action/RetrieveObject.action"
  "action/StackObject.action"
  "action/StoreObject.action"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
