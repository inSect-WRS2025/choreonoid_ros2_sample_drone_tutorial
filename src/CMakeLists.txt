choreonoid_add_drone_controller(
    SampleDroneFlightController SampleDroneFlightController.cpp)

ament_target_dependencies(
    SampleDroneFlightController rclcpp geometry_msgs sensor_msgs std_msgs)

choreonoid_add_drone_controller(
    TwistPublisherController TwistPublisherController.cpp)

ament_target_dependencies(
    TwistPublisherController rclcpp geometry_msgs sensor_msgs)

choreonoid_add_simple_controller(
    JoyTopicSubscriberController JoyTopicSubscriberController.cpp)

ament_target_dependencies(
    JoyTopicSubscriberController rclcpp sensor_msgs)

choreonoid_add_simple_controller(
    ResetZeroVelocityController reset_zero_velocity_controller.cpp)

ament_target_dependencies(
    ResetZeroVelocityController rclcpp std_msgs)

choreonoid_add_simple_controller(
    MothcopePanTiltController MothcopePanTiltController.cpp)

ament_target_dependencies(
    MothcopePanTiltController rclcpp geometry_msgs)
    
choreonoid_add_simple_controller(
    AizuSpiderController AizuSpiderController.cpp)
    
ament_target_dependencies(
    AizuSpiderController rclcpp sensor_msgs)

choreonoid_add_simple_controller(
    Jaco2Controller Jaco2Controller.cpp)
    
ament_target_dependencies(
    Jaco2Controller rclcpp sensor_msgs)

choreonoid_add_simple_controller(
    MobileRobotFlipperController MobileRobotFlipperController.cpp)
    
ament_target_dependencies(
    MobileRobotFlipperController rclcpp sensor_msgs)

choreonoid_add_simple_controller(
    MobileRobotTrackController MobileRobotTrackController.cpp)
    
ament_target_dependencies(
    MobileRobotTrackController rclcpp sensor_msgs)

choreonoid_add_simple_controller(
    AgxTankRos2Controller AgxTankRos2Controller.cpp)
    
ament_target_dependencies(
    AgxTankRos2Controller rclcpp sensor_msgs)

choreonoid_add_simple_controller(
    CratlasAnglePubController CratlasAnglePubController.cpp)
    
ament_target_dependencies(
    CratlasAnglePubController rclcpp sensor_msgs)

choreonoid_add_simple_controller(
    CratlasPanTiltController CratlasPanTiltController.cpp)

ament_target_dependencies(
    CratlasPanTiltController rclcpp geometry_msgs)