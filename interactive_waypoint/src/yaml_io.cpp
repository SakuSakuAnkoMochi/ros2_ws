#include "interactive_waypoint/yaml_io.hpp"

#include <fstream>
#include <iostream>

#include <yaml-cpp/yaml.h>

bool YamlIO::save(const std::string &path, const std::vector<Waypoint> &waypoints)
{
    YAML::Emitter out;

    out << YAML::BeginMap;

    out << YAML::Key << "waypoints";
    out << YAML::Value << YAML::BeginSeq;

    for (const auto &wp : waypoints)
    {
        out << YAML::BeginMap;

        out << YAML::Key << "name";
        out << YAML::Value << wp.name;

        out << YAML::Key << "frame_id";
        out << YAML::Value << wp.frame_id;

        out << YAML::Key << "position";
        out << YAML::Value;

        out << YAML::BeginMap;

        out << YAML::Key << "x";
        out << YAML::Value << wp.local_pose.position.x;

        out << YAML::Key << "y";
        out << YAML::Value << wp.local_pose.position.y;

        out << YAML::Key << "z";
        out << YAML::Value << wp.local_pose.position.z;

        out << YAML::EndMap;

        out << YAML::Key << "orientation";
        out << YAML::Value;

        out << YAML::BeginMap;

        out << YAML::Key << "x";
        out << YAML::Value << wp.local_pose.orientation.x;

        out << YAML::Key << "y";
        out << YAML::Value << wp.local_pose.orientation.y;

        out << YAML::Key << "z";
        out << YAML::Value << wp.local_pose.orientation.z;

        out << YAML::Key << "w";
        out << YAML::Value << wp.local_pose.orientation.w;

        out << YAML::EndMap;

        out << YAML::EndMap;
    }

    out << YAML::EndSeq;

    out << YAML::EndMap;

    std::ofstream file(path);

    if (!file.is_open())
    {
        return false;
    }

    file << out.c_str();

    return true;
}

std::vector<Waypoint> YamlIO::load(const std::string &path)
{
    std::vector<Waypoint> result;

    try
    {
        YAML::Node root = YAML::LoadFile(path);

        if (!root["waypoints"])
        {
            return result;
        }

        for (const auto &node : root["waypoints"])
        {
            Waypoint wp;

            wp.name = node["name"].as<std::string>();

            wp.frame_id = node["frame_id"].as<std::string>();

            auto pos = node["position"];

            auto rot = node["orientation"];

            wp.local_pose.position.x = pos["x"].as<double>();
            wp.local_pose.position.y = pos["y"].as<double>();
            wp.local_pose.position.z = pos["z"].as<double>();

            wp.local_pose.orientation.x = rot["x"].as<double>();
            wp.local_pose.orientation.y = rot["y"].as<double>();
            wp.local_pose.orientation.z = rot["z"].as<double>();
            wp.local_pose.orientation.w = rot["w"].as<double>();

            result.push_back(wp);
        }
    }
    catch (...)
    {
        return {};
    }

    return result;
}