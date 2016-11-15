/**
 * 
 * 
 * 
 * 
 * */


    
#pragma once


#include <list>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <string> 
#include <fstream>
#include <XmlRpcValue.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include <geometry_msgs/PolygonStamped.h>

#include <visualization_msgs/Marker.h>
#include <nav_msgs/MapMetaData.h>

#include <grid_map_ros/grid_map_ros.hpp>

using namespace std;
using namespace ros;
using namespace grid_map;

namespace define_polygon_areas {

/*!
 
 */
class define_polygon_areas
{
    struct type_area{
        grid_map::Polygon polygon;
        string name;
    };
    
    public:

      /*!
       * Constructor.
       * @param nodeHandle the ROS node handle.
       */
      define_polygon_areas(ros::NodeHandle& nodeHandle);

      ~define_polygon_areas();
      
      void drawSquare(double start_x,double start_y,double end_x,double end_y,double value);

      void publishMap();
      
      void mapCallback(const nav_msgs::OccupancyGrid& msg);
      
      void plotMarker(double x, double  y, string text );
    
     void timerCallback(const ros::TimerEvent&);
     
     void clickCallback(const geometry_msgs::PointStamped::ConstPtr msg);
     
     std::list<define_polygon_areas::type_area> loadAreas(std::string regions_file);
     
     void drawPolygon(const grid_map::Polygon poly,double value);

     void yamlSave();
     
    private:
      //! ROS nodehandle.
      ros::NodeHandle& nodeHandle_;
      
      //! Grid map data.
      grid_map::GridMap map_;

      //! Grid map publisher.
      ros::Publisher gridMapPublisher_;

      bool isMapLoaded;
      
      nav_msgs::MapMetaData mapDesc;
      string mapFrame;
      
      ros::Publisher vis_pub;
      
      int numberOfClicks;
      grid_map::Polygon plg;
      
      vector<type_area> definedAreas;
      int areaCounter;
  
}; // End of Class define_polygon_areas

} // end of  namespace define_polygon_areas
