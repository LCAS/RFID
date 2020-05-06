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
      void rfid_readings_topic_callback(const rfid_node::TagReading::ConstPtr& msg);

      void loadROSParams();

      void showROSParams();

      //! periodic map updates
      void belief_publishing_callback(const ros::TimerEvent&);

      void prob_publishing_callback(const ros::TimerEvent&);

      void updateRobotPose();

      void publishMap();

      double countValuesInArea(Polygon pol);

      void getMapDimensions();

      void drawSquare(double start_x,double start_y,double end_x,double end_y,double value);

      void drawCircle(double x, double y, double radius, double value);

      void drawDetectionShape(double cx,double cy, double rh,
        double radius,double radiusMin,  double max_heading,
        double midProb, double lowProb, double highProb);

      void updateLastDetectionPose(double x, double y);

      void wasHere(type_area area);

      void drawPolygon(const grid_map::Polygon poly,double value);

      void loadZois();

      void belief_saving_callback(const ros::TimerEvent&);

      void map_topic_callback(const nav_msgs::OccupancyGrid& msg);
      bool isUpdatePose();
      bool isSubregion(std::string zoiName,std::string &parent);

      void do_stuff();
    private:
      type_area lastRegion_;
      Position lastDetectPose_;


      //! ROS nodehandle.
      ros::NodeHandle& nodeHandle_;

      //! relevant detections counter.
      unsigned int numDetections_;

      //! ROS subscriber to rfid messages...
      std::string rfid_readings_topic_name_;
      ros::Subscriber rfid_readings_topic_sub_;
      // stores readings
      ConsumerProducerQueue<std::vector<double>> readings_queue_;

      //! We read the static map once.
      bool isMapLoaded_;
      std::string map_service_name_;
      std::string map_topic_name_;            
      // this is not needed to read the topic once
      //ros::Subscriber map_topic_sub_;
      nav_msgs::MapMetaData mapDesc_;
      string map_frame_id_;

      //! preload gridmaps from files
      bool loadGrids_;
      //! load/save gridmap path
      std::string save_route_;
      //! load/save gridmap file name name
      std::string gridmap_image_file_;

      //! ROS publisher for location probs
      std::string prob_topic_name_;
      ros::Publisher prob_topic_pub_;

      //! ROS publisher for rfid belief grid map.
      std::string rfid_belief_topic_name_; //grid_map_name
      ros::Publisher rfid_belief_topic_pub_; //gridMapPublisher_

      //! ROS timers for periodic tasks
      // publish rfid belief map
      ros::Timer belief_publishing_timer_;
      //! period [s]
      double belief_publishing_period_; 
      // publish location probabilities
      ros::Timer prob_publishing_timer_;
      //! period [s]
      double prob_publishing_period_; 
      // save temporal probability maps
      ros::Timer belief_saving_timer_ ;
      //! period [s]
      double belief_saving_period_; 


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

      //! saved ROS gridmap image format
      std::string rosEncoding;
      //! rfid tag id
      std::string tagID_;

      //! tagged object name
      std::string object_name_;

      //! robot frame id
      std::string robot_frame_;


      double constrainAngle2PI(double x);

      double constrainAnglePI(double x);

      void nextTimeDecay();

      // zois indexed by name
      std::map<std::string,rfid_gridMap::type_area> mapAreas;


}; // End of Class rfid_gridMap

} // end of  namespace rfid_grid_map
