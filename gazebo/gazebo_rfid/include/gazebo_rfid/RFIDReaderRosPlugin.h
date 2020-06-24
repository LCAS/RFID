
#ifndef GAZEBO_RFIDREADER_RFIDREADERROSPLUGIN_H
#define GAZEBO_RFIDREADER_RFIDREADERROSPLUGIN_H


#include <algorithm>
#include <string>
#include <assert.h>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/LaserScan.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_plugins/PubQueue.h>

#include <rfid_node/TagReading.h>
#include "gazebo_rfid/RFIDReader.h"

namespace gazebo
{
class GazeboRosRFIDReader : public SensorPlugin
{
  /// \brief Constructor
  public: GazeboRosRFIDReader();

  /// \brief Destructor
  public: ~GazeboRosRFIDReader();

  /// \brief Load the plugin
  /// \param take in SDF root element
  public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Keep track of number of connctions
  private: int connect_count_;
  private: void ClientConnect();
  private: void ClientDisconnect();

  // Pointer to the model
  GazeboRosPtr gazebo_ros_;
  private: std::string world_name_;
  private: physics::WorldPtr world_;
  /// \brief The parent sensor
  private: std::shared_ptr<sensors::RFIDReader> parent_reader_sensor_;

  /// \brief pointer to ros node
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;
  private: PubQueue<rfid_node::TagReading>::Ptr pub_queue_;

  /// \brief topic name
  private: std::string topic_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  /// \brief frame transform name, should match link name
  private: std::string frame_name_;
  
  // deferred load in case ros is blocking
  private: sdf::ElementPtr sdf;
  private: void LoadThread();
  private: boost::thread deferred_load_thread_;
  private: unsigned int seed;

  private: gazebo::transport::NodePtr gazebo_node_;
  private: gazebo::transport::SubscriberPtr rfid_scan_sub_;
  private: void OnScan(ConstGzStringPtr &_msg);

  /// \brief prevents blocking
  private: PubMultiQueue pmq;
};
}

#endif //GAZEBO_RFIDREADER_RFIDREADERROSPLUGIN_H