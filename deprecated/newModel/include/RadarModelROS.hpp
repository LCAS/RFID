#ifndef RADARMODELROS_H
#define RADARMODELROS_H

#include "RadarModel.hpp"

#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

class RadarModelROS: public RadarModel {

    RadarModelROS(const nav_msgs::OccupancyGrid& nav_map, const double sigma_power, const double sigma_phase );

    void initRefMap(const nav_msgs::OccupancyGrid& nav_map);

    void loadBelief(const std::string imageURI);

    void saveBelief(const std::string imageURI);

}; // end class

#endif