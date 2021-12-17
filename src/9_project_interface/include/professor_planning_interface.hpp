#pragma once

#include "utils.hpp"

namespace professor {
	/*!
	* Plan a safe and fast path in the arena
	* @param[in]  borders        border of the arena [m]
	* @param[out] obstacle_list  list of obstacle polygon [m]
	* @param[out] gate_list      list of polygons representing the gate [m]
	* @param[out] x              list of x positions of the robots in the arena reference system
	* @param[out] y              list of y positions of the robots in the arena reference system
	* @param[out] theta          list of yaw of the robots in the arena reference system
	* @param[out] path           list of paths for the robots
	* @param[in]  config_folder  A custom string from config file.
	*/
	bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<Polygon>& gate_list, const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta, std::vector<Path>& path, const std::string& config_folder);
}
