#include "vicon_receiver/publisher.hpp"

#include "rclcpp/qos.hpp"


Publisher::Publisher(std::string subject_name, std::string topic_name, rclcpp::Node* node)
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensors = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

    if (subject_to_mavros_vehicle_map.find(subject_name) != subject_to_mavros_vehicle_map.end()) {
        is_mavros = true;
        auto mavros_topic_name = subject_to_mavros_vehicle_map.at(subject_name);

        mavros_position_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(mavros_topic_name, qos_sensors);
        RCLCPP_INFO(node->get_logger(), "Detected as MAVROS, creating topic %s", mavros_topic_name.c_str());
    } else {
        position_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, qos_sensors);
        RCLCPP_INFO(node->get_logger(), "Creating topic %s", topic_name.c_str());
    }

    this->subject_name = subject_name;
    this->topic_name = topic_name;
    is_ready = true;
    this->node = node;

    for(int i = 0; i<this->moving_average_buffer_length; i++) {
        this->previous_publish_times.push_back(0.0);
        this->previous_publish_only_times.push_back(0.0);
    }
    this->previous_publish_time  = this->node->now();
}

void Publisher::publish(PositionStruct p)
{
    auto publish_start_time = this->node->now();

    // Send using PoseStamped msg
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    // msg->header.seq = p.frame_number;
    msg->header.stamp = p.receive_time;
    msg->header.frame_id = "map";
    // Change to ROS frame?
    // ROS X = Vicon -Y
    // ROS Y = Vicon X
    // Rotation around Z axis by 90 degrees from vicon->ros2
    msg->pose.position.x = p.translation[0];
    msg->pose.position.y = p.translation[1];
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

    // Calculate moving average of between publish times
    rclcpp::Time current_time = this->node->now();
    rclcpp::Duration time_diff = current_time - this->previous_publish_time;
    this->previous_publish_times[this->previous_publish_times_index] = time_diff.seconds();   
    double sum_of_elems = 0.0;
    for(double& n: this->previous_publish_times){
        sum_of_elems += n;
    }
    double average = sum_of_elems / this->moving_average_buffer_length;
    double freq = 1.0 / average;

    // Calculate moving average of time to publish times
    rclcpp::Duration publish_time_diff = current_time - publish_start_time;
    this->previous_publish_only_times[this->previous_publish_times_index] = publish_time_diff.seconds();
    double publish_sum_of_elems = 0.0;
    for(double& n: this->previous_publish_only_times){
        publish_sum_of_elems += n;
    }
    double publish_average = publish_sum_of_elems / this->moving_average_buffer_length;
    // double publish_freq = 1.0 / publish_average;

    if(this->previous_publish_times_index == 0) {
        RCLCPP_INFO(this->node->get_logger(), "Publishing %s, freq is: %f, publish average is: %f", this->subject_name.c_str(), freq, publish_average);
    }

    this->previous_publish_time = current_time;
    this->previous_publish_times_index = (this->previous_publish_times_index+1) % this->moving_average_buffer_length;
}
