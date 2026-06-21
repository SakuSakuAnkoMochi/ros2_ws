#include "interactive_waypoint/waypoint_node.hpp"

#include <cstdlib>

#include "interactive_waypoint/yaml_io.hpp"

WaypointNode::WaypointNode() : Node("interactive_waypoint")
{
    const char *home = std::getenv("HOME");

    yaml_path_ = std::string(home) + "/waypoints.yaml";

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    marker_manager_ = std::make_unique<MarkerManager>(this, tf_buffer_.get());

    marker_manager_->setSaveCallback(std::bind(&WaypointNode::saveYaml, this));

    clicked_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, std::bind(&WaypointNode::clickedPointCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&WaypointNode::timerCallback, this));

    loadYaml();

    RCLCPP_INFO(get_logger(), "interactive_waypoint started");
}

void WaypointNode::clickedPointCallback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    RCLCPP_INFO(
    get_logger(),
    "marker create called");
    Waypoint wp;
    wp.name = "waypoint_" + std::to_string(waypoint_counter_++);
    wp.frame_id = msg->header.frame_id;
    geometry_msgs::msg::Pose world_pose;
    world_pose.position.x = msg->point.x;
    world_pose.position.y = msg->point.y;
    world_pose.position.z = msg->point.z;
    world_pose.orientation.w = 1.0;
    wp.local_pose = marker_manager_->worldToLocal(wp.frame_id, world_pose);
    marker_manager_->addWaypoint(wp);
}

void WaypointNode::timerCallback()
{
    marker_manager_->updateMarkers();
}

void WaypointNode::saveYaml()
{
    auto waypoints = marker_manager_->getWaypoints();

    bool success = YamlIO::save(yaml_path_, waypoints);

    if (success)
    {
        RCLCPP_INFO(get_logger(), "saved %s", yaml_path_.c_str());
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "failed to save yaml");
    }
}
void WaypointNode::loadYaml()
{
    auto waypoints = YamlIO::load(yaml_path_);

    for (const auto &wp : waypoints)
    {
        marker_manager_->addWaypoint(wp);

        waypoint_counter_++;
    }

    RCLCPP_INFO(get_logger(), "loaded %zu waypoints", waypoints.size());
}