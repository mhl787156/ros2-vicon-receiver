#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP
#include <map>
#include <vector>

// #include <unistd.h>
#include "rclcpp/rclcpp.hpp"
// #include "vicon_receiver/msg/position.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Struct used to hold segment data to transmit to the Publisher class.
struct PositionStruct
{
    rclcpp::Time receive_time;
    double translation[3];
    double rotation[4];
    std::string subject_name;
    std::string segment_name;
    std::string translation_type;
    unsigned int frame_number;

} typedef PositionStruct;

const std::map<const std::string, const std::string> subject_to_mavros_vehicle_map = {
    {"clover11", "/vehicle_11/mavros/vision_pose/pose"},
    {"clover12", "/vehicle_12/mavros/vision_pose/pose"},
    {"clover13", "/vehicle_13/mavros/vision_pose/pose"},
    {"clover14", "/vehicle_14/mavros/vision_pose/pose"},
};

// Class that allows segment data to be published in a ROS2 topic.
class Publisher
{
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_position_publisher_;

public:
    bool is_ready = false;
    bool is_mavros = false;
    std::string topic_name;
    std::string subject_name;
    rclcpp::Node* node;

    int moving_average_buffer_length = 50;
    int previous_publish_times_index = 0;
    std::vector<double> previous_publish_times;
    rclcpp::Time previous_publish_time;

    std::vector<double> previous_publish_only_times;

    Publisher(std::string subject_name, std::string topic_name, rclcpp::Node* node);

    // Publishes the given position in the ROS2 topic whose name is indicated in
    // the constructor.
    void publish(PositionStruct p);
};

#endif