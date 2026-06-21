#pragma once

#include <string>

#include <geometry_msgs/msg/pose.hpp>

struct Waypoint
{
    std::string name;

    /*
     * parent frame
     *
     * map
     * odom
     * base_link
     * tool0
     * camera_link
     * ...
     */
    std::string frame_id;

    /*
     * pose relative to frame_id
     */
    geometry_msgs::msg::Pose local_pose;
};