#pragma once

#include "utils.hpp"

namespace professor {
/*!
* Plan a safe and fast path in the arena
* @param[in]  borders        border of the arena [m]
* @param[out] obstacle_list  list of obstacle polygon [m]
* @param[out] victim_list    list of pair victim_id and polygon [m]
* @param[out] gate           polygon representing the gate [m]
* @param[out] x              x position of the robot in the arena reference system
* @param[out] y              y position of the robot in the arena reference system
* @param[out] theta          yaw of the robot in the arena reference system
* @param[in]  config_folder  A custom string from config file.
*/
bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list,
              const std::vector<Polygon>& gate_list, 
              const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta,
              std::vector<Path>& path, const std::string& config_folder);

}
