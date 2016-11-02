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
#include <boost/algorithm/string/predicate.hpp>


// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>    
#include <image_transport/image_transport.h>
// - messages
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/MapMetaData.h>  
// - grid map
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
//

// ROS - OURS
#include <rfid_node/TagReading.h>

using namespace std;
using namespace ros;
using namespace grid_map;

namespace rfid_grid_map {

/*!
 
 */
class rfid_gridMap
{
    
    struct type_area{
        double prob;
        grid_map::Polygon polygon;
        string name;
    };
    
    public:

      /*!
       * Constructor.
       * @param nodeHandle the ROS node handle.
       */
      rfid_gridMap(ros::NodeHandle& nodeHandle);

      virtual ~rfid_gridMap();
      
      //! callback for rfid messages...
      void tagCallback(const rfid_node::TagReading::ConstPtr& msg);
    
      //! periodic map updates
      void updateMapCallback(const ros::TimerEvent&);
    
      void updateProbs(const ros::TimerEvent&);
      
      void updateTransform();        
      
      void publishMap();
      
      double countValuesInArea(Polygon pol);
      
      void drawSquare(double start_x,double start_y,double end_x,double end_y,double value);
      
      void drawCircle(double x, double y, double radius, double value);

      void updateLastDetectionPose(double x, double y);

      void wasHere(type_area area);
    
      void drawPolygon(const grid_map::Polygon poly,double value);
      
      std::vector<rfid_gridMap::type_area> loadAreas();
      
      void saveMapCallback(const ros::TimerEvent&);
      
    private:
    
      Position lastP;
      type_area lastRegion;

      //! ROS nodehandle.
      ros::NodeHandle& nodeHandle_;
      
      //! ROS subscriber to rfid messages...
      ros::Subscriber sub_;
      //! Grid map data.
      grid_map::GridMap map_;      
      //! Grid map layer name (note: we could have more...)
      std::string layerName;
      
      //! Grid map layer lowest value
      double lowerValue;
      //! Grid map layer highest value
      double upperValue;

      //! gridmap actualization rate .        
      double intensity_;
            
      //! publisher for probs
      ros::Publisher prob_pub_ ;
       
      //! map Size (in meters)
      double size_x; 
      double size_y;
      //! Grid map publisher.
      ros::Publisher gridMapPublisher_;

      tf::TransformListener listener_;
      tf::StampedTransform transform_;



      std::string gridmap_image_file;
      std::string rosEncoding;
      //! regions description file 
      YAML::Node config ;
      //! rfid tag id
      std::string tagID;
      
      //! global frame id (for maps)
      std::string global_frame;
      //! robot frame id 
      std::string robot_frame;
      
      std::vector<rfid_gridMap::type_area> mapAreas;

}; // End of Class rfid_gridMap

} // end of  namespace rfid_grid_map
