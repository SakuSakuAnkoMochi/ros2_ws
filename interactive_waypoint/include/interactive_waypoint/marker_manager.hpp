#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <tf2_ros/buffer.h>

#include "interactive_waypoint/waypoint.hpp"

class MarkerManager
{
public:
    using SaveCallback = std::function<void(void)>;

    MarkerManager(rclcpp::Node* node, tf2_ros::Buffer *tf_buffer);

    void setSaveCallback(SaveCallback callback);

    void addWaypoint(const Waypoint &waypoint);

    bool removeWaypoint(const std::string &name);

    std::vector<Waypoint> getWaypoints() const;

    void updateMarkers();

    geometry_msgs::msg::Pose worldToLocal(const std::string &frame_id, const geometry_msgs::msg::Pose &world_pose);

private:
    using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;

    using FeedbackConstPtr = Feedback::ConstSharedPtr;

    void createInteractiveMarker(const Waypoint &wp);

    void feedbackCallback(FeedbackConstPtr feedback);

    void deleteCallback(FeedbackConstPtr feedback);

    void saveCallback(FeedbackConstPtr feedback);

    void add6DofControls(visualization_msgs::msg::InteractiveMarker &marker);

    geometry_msgs::msg::Pose localToWorld(const Waypoint &wp);

private:
    rclcpp::Node* node_;

    tf2_ros::Buffer *tf_buffer_;

    SaveCallback save_callback_;

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

    interactive_markers::MenuHandler menu_;

    interactive_markers::MenuHandler::EntryHandle delete_handle_;

    interactive_markers::MenuHandler::EntryHandle save_handle_;

    std::unordered_map<std::string, Waypoint> waypoints_;
};