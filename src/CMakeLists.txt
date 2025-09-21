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
