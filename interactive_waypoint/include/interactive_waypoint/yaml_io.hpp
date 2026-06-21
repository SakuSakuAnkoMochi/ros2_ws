#pragma once

#include <string>
#include <vector>

#include "interactive_waypoint/waypoint.hpp"

class YamlIO
{
public:

    static bool save(const std::string& path,const std::vector<Waypoint>& waypoints);

    static std::vector<Waypoint> load(const std::string& path);

private:

    YamlIO() = delete;
};