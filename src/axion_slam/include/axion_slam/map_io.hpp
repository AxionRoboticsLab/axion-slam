#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>
#include <vector>

namespace axion_slam
{

struct MapPaths
{
  std::string yaml_path;
  std::string pgm_path;
  std::string logical_name;
};

/** Build ~/data/maps/{name}_2dmap.{yaml,pgm} (or override maps_dir). */
MapPaths make_map_paths(const std::string & maps_dir, const std::string & logical_name);

bool is_valid_map_name(const std::string & name);

/** List logical names that have both yaml + pgm under maps_dir. */
std::vector<std::string> list_map_names(const std::string & maps_dir);

bool save_occupancy_grid(
  const nav_msgs::msg::OccupancyGrid & grid,
  const MapPaths & paths,
  std::string & error);

bool load_occupancy_grid(
  const MapPaths & paths,
  nav_msgs::msg::OccupancyGrid & grid,
  std::string & error);

}  // namespace axion_slam
