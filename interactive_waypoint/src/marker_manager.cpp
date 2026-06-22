#include "interactive_waypoint/marker_manager.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static geometry_msgs::msg::Pose transformToPose(const tf2::Transform &tf)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = tf.getOrigin().x();
    pose.position.y = tf.getOrigin().y();
    pose.position.z = tf.getOrigin().z();
    pose.orientation = tf2::toMsg(tf.getRotation());

    return pose;
}

using InteractiveMarker = visualization_msgs::msg::InteractiveMarker;
using InteractiveMarkerControl = visualization_msgs::msg::InteractiveMarkerControl;
using Marker = visualization_msgs::msg::Marker;

MarkerManager::MarkerManager(rclcpp::Node *node, tf2_ros::Buffer *tf_buffer) : node_(node), tf_buffer_(tf_buffer)
{
    // server_ =std::make_shared<interactive_markers::InteractiveMarkerServer>("waypoint_server",node_->shared_from_this());
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("waypoint_server", node_);

    delete_handle_ = menu_.insert("Delete", std::bind(&MarkerManager::deleteCallback, this, std::placeholders::_1));

    save_handle_ = menu_.insert("Save YAML", std::bind(&MarkerManager::saveCallback, this, std::placeholders::_1));
}

void MarkerManager::setSaveCallback(SaveCallback callback)
{
    save_callback_ = callback;
}

void MarkerManager::addWaypoint(const Waypoint &waypoint)
{
    waypoints_[waypoint.name] = waypoint;

    createInteractiveMarker(waypoint);

    server_->applyChanges();
}

bool MarkerManager::removeWaypoint(const std::string &name)
{
    auto it = waypoints_.find(name);

    if (it == waypoints_.end())
    {
        return false;
    }

    server_->erase(name);

    waypoints_.erase(it);

    server_->applyChanges();

    return true;
}

std::vector<Waypoint> MarkerManager::getWaypoints() const
{
    std::vector<Waypoint> result;

    result.reserve(waypoints_.size());

    for (const auto &pair : waypoints_)
    {
        result.push_back(pair.second);
    }

    return result;
}
void MarkerManager::createInteractiveMarker(const Waypoint &wp)
{
    RCLCPP_INFO(
        node_->get_logger(),
        "create marker %s",
        wp.name.c_str());
    InteractiveMarker marker;

    marker.header.frame_id = "map";

    marker.name = wp.name;

    marker.description = wp.name;

    marker.scale = 0.3;

    marker.pose = localToWorld(wp);

    // Marker sphere;

    // sphere.type = Marker::SPHERE;

    // sphere.scale.x = 0.08;
    // sphere.scale.y = 0.08;
    // sphere.scale.z = 0.08;

    // sphere.color.r = 0.0;
    // sphere.color.g = 1.0;
    // sphere.color.b = 0.0;
    // sphere.color.a = 1.0;

    Marker arrow;
    arrow.type = Marker::ARROW;
    arrow.scale.x = 0.25;
    arrow.scale.y = 0.04;
    arrow.scale.z = 0.04;
    arrow.color.r = 1.0;
    arrow.color.g = 0.6;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;

    InteractiveMarkerControl visual;

    visual.always_visible = true;

    // visual.markers.push_back(sphere);
    visual.markers.push_back(arrow);

    marker.controls.push_back(visual);

    add6DofControls(marker);

    server_->insert(marker, std::bind(&MarkerManager::feedbackCallback, this, std::placeholders::_1));

    menu_.apply(*server_, marker.name);
    server_->applyChanges();
}

void MarkerManager::add6DofControls(InteractiveMarker &marker)
{
    InteractiveMarkerControl control;
    control.orientation_mode = InteractiveMarkerControl::FIXED;

    control.orientation.w = 1.0;
    control.orientation.x = 1.0;
    control.orientation.y = 0.0;
    control.orientation.z = 0.0;

    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;

    marker.controls.push_back(control);

    // control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;

    // marker.controls.push_back(control);

    control.orientation.x = 0.0;
    control.orientation.y = 1.0;
    control.orientation.z = 0.0;

    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;

    marker.controls.push_back(control);

    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;

    marker.controls.push_back(control);

    control.orientation.x = 0.0;
    control.orientation.y = 0.0;
    control.orientation.z = 1.0;

    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;

    marker.controls.push_back(control);

    // control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;

    // marker.controls.push_back(control);
}

geometry_msgs::msg::Pose MarkerManager::localToWorld(const Waypoint &wp)
{
    geometry_msgs::msg::Pose result;

    try
    {
        auto tf = tf_buffer_->lookupTransform("map", wp.frame_id, tf2::TimePointZero);

        tf2::Transform map_T_frame;

        tf2::fromMsg(tf.transform, map_T_frame);

        tf2::Transform frame_T_local;

        frame_T_local.setOrigin(tf2::Vector3(wp.local_pose.position.x, wp.local_pose.position.y, wp.local_pose.position.z));

        frame_T_local.setRotation(tf2::Quaternion(wp.local_pose.orientation.x, wp.local_pose.orientation.y, wp.local_pose.orientation.z, wp.local_pose.orientation.w));

        tf2::Transform map_T_local = map_T_frame * frame_T_local;

        result = transformToPose(map_T_local);
    }
    catch (...)
    {
        result = wp.local_pose;
    }

    return result;
}

geometry_msgs::msg::Pose MarkerManager::worldToLocal(const std::string &frame_id, const geometry_msgs::msg::Pose &world_pose)
{
    geometry_msgs::msg::Pose result;

    try
    {
        auto tf = tf_buffer_->lookupTransform("map", frame_id, tf2::TimePointZero);

        tf2::Transform map_T_frame;

        tf2::fromMsg(tf.transform, map_T_frame);

        tf2::Transform frame_T_map = map_T_frame.inverse();

        tf2::Transform map_T_pose;

        map_T_pose.setOrigin(tf2::Vector3(world_pose.position.x, world_pose.position.y, world_pose.position.z));

        map_T_pose.setRotation(tf2::Quaternion(world_pose.orientation.x, world_pose.orientation.y, world_pose.orientation.z, world_pose.orientation.w));

        tf2::Transform frame_T_pose = frame_T_map * map_T_pose;

        result = transformToPose(frame_T_pose);
    }
    catch (...)
    {
        result = world_pose;
    }

    return result;
}

void MarkerManager::feedbackCallback(FeedbackConstPtr feedback)
{
    if (feedback->event_type != Feedback::POSE_UPDATE)
    {
        return;
    }

    auto it = waypoints_.find(feedback->marker_name);

    if (it == waypoints_.end())
    {
        return;
    }

    it->second.local_pose = worldToLocal(it->second.frame_id, feedback->pose);
}

void MarkerManager::deleteCallback(FeedbackConstPtr feedback)
{
    removeWaypoint(feedback->marker_name);
}

void MarkerManager::saveCallback(FeedbackConstPtr)
{
    if (save_callback_)
    {
        save_callback_();
    }
}

void MarkerManager::updateMarkers()
{
    for (auto &pair : waypoints_)
    {
        auto pose = localToWorld(pair.second);

        server_->setPose(pair.first, pose);
    }

    server_->applyChanges();
}