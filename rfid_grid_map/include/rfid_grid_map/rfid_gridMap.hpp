/**
 *
 *
 *
 *
 * */



#pragma once
#include <time.h>       /* time_t, time, ctime */
#include <math.h>
#include <list>
#include <cmath>
#include <string>
#include <fstream>
#include <regex>

//#include <boost/algorithm/string/predicate.hpp>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <XmlRpcException.h>

// - messages
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/MapMetaData.h>

// - grid map
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
//

// ROS - OURS
#include <rfid_node/TagReading.h>
#include "rfid_grid_map/GetBeliefMaps.h"

// lib
#include "RadarModelROS.hpp"
#include "readings_queue.hpp"

using namespace std;
using namespace ros;
using namespace grid_map;

namespace rfid_grid_map {

/*!

 */
class rfid_gridMap
{

    struct type_measurement{
        std::string tagID;
        int tagNum;
        int numDetections;        
        double x_m;
        double y_m;
        double th_deg;
        double rxPower_dB;
        double rxPhase_rad;
        double rxFreq_Hz;
        double txPower_dB;
    };

    public:

      /*!
       * Constructor.
       * @param nodeHandle the ROS node handle.
       */
      rfid_gridMap(ros::NodeHandle& nodeHandle);

      virtual ~rfid_gridMap();

      //! callback for rfid messages...
      void rfid_readings_topic_callback(const rfid_node::TagReading::ConstPtr& msg);

      bool rfid_belief_srv_callback(rfid_grid_map::GetBeliefMaps::Request  &req, rfid_grid_map::GetBeliefMaps::Response &res);

      void loadROSParams();

      void showROSParams();

      void updateRobotPose();

      void getMapDimensions();

      void map_topic_callback(const nav_msgs::OccupancyGrid& msg);

      void do_stuff();

    private:
      //! ROS nodehandle.
      ros::NodeHandle& nodeHandle_;

      //! ROS subscriber to rfid messages...
      std::string rfid_readings_topic_name_;
      ros::Subscriber rfid_readings_topic_sub_;
      // stores readings
      ConsumerProducerQueue<type_measurement> readings_queue_;

      //! We read the static map once.
      bool isMapLoaded_;
      std::string map_service_name_;
      std::string map_topic_name_;            
      // this is not needed to read the topic once
      //ros::Subscriber map_topic_sub_;
      nav_msgs::MapMetaData mapDesc_;
      string map_frame_id_;

      //! ROS service for rfid belief grid maps.
      std::string rfid_belief_srv_name_; 
      ros::ServiceServer rfid_belief_srv_ss_; 

      //! ROS publisher for rfid belief grid map.
      std::string rfid_belief_topic_name_; //grid_map_name
      ros::Publisher rfid_belief_topic_pub_; //gridMapPublisher_

      // tfs to track robot position
      tf::TransformListener listener_;
      tf::StampedTransform transform_;
      double robot_x_,robot_y_,robot_h_;

      // radar model
      RadarModelROS model_;
      double sigma_power_;
      double sigma_phase_;    
      //! map resolution // m. /cell
      double map_resolution_; 

      //! robot frame id
      std::string robot_frame_;

      // Counts each tag number of detections.
      std::unordered_map<std::string, int> tagID_detections_map_;
      
      // Assing each tag a number
      std::unordered_map<std::string, int> tagID_enumeration_map_;

      // prevents from reading more tags when service is called ...
      bool isReadingEnabled_;


}; // End of Class rfid_gridMap

} // end of  namespace rfid_grid_map
