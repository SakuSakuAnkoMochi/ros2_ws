#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "interactive_waypoint/marker_manager.hpp"

class WaypointNode : public rclcpp::Node
{
public:
    WaypointNode();

private:
    void clickedPointCallback(geometry_msgs::msg::PointStamped::SharedPtr msg);

    void timerCallback();

    void saveYaml();

    void loadYaml();

private:
    std::string yaml_path_;

    uint32_t waypoint_counter_{0};

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<MarkerManager> marker_manager_;
};