#include "vicon_receiver/publisher.hpp"

#include "rclcpp/qos.hpp"


Publisher::Publisher(std::string subject_name, std::string topic_name, rclcpp::Node* node)
{
    if (subject_to_mavros_vehicle_map.find(subject_name) != subject_to_mavros_vehicle_map.end()) {
        is_mavros = true;
        auto mavros_topic_name = subject_to_mavros_vehicle_map.at(subject_name);

        // rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        mavros_position_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(mavros_topic_name, 10);
        RCLCPP_INFO(node->get_logger(), "Detected as MAVROS, creating topic %s", mavros_topic_name.c_str());
    } else {
        position_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);
        RCLCPP_INFO(node->get_logger(), "Creating topic %s", topic_name.c_str());
    }

    is_ready = true;
}

void Publisher::publish(PositionStruct p)
{
    // Send using PoseStamped msg
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    // msg->header.seq = p.frame_number;
    msg->header.stamp = p.receive_time;
    msg->header.frame_id = "map";
    // Change to ROS frame?
    // ROS X = Vicon -Y
    // ROS Y = Vicon X
    msg->pose.position.x = -p.translation[1];
    msg->pose.position.y = p.translation[0];
    msg->pose.position.z = p.translation[2];
    msg->pose.orientation.x = p.rotation[0];
    msg->pose.orientation.y = p.rotation[1];
    msg->pose.orientation.z = p.rotation[2];
    msg->pose.orientation.w = p.rotation[3];
    
    if(is_mavros) {
        mavros_position_publisher_->publish(*msg);
    } else {
        position_publisher_->publish(*msg);
    }
}
