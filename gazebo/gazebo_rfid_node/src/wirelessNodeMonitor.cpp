/**
*
* Based on gazebo-contactMonitor.
* This time we forward  a gazebo WirelessNode msg into a ROS fid_node TagReading
* msg.
*
* TODO:
*     propagation model is inherited from gazebo wireless
*     TagReading transmitted power (txP) is not filled
*     TagReading phase  (phase) is not filled
*
* /gazebo/get_model_state
*/

#include <wirelessNodeMonitor/wirelessNodeMonitor.h>

std::string ros_rfid_topic_name;
std::string gazebo_wireless_node_topic_name;
std::string ros_rfid_frame_id;
ros::Publisher ros_rfid_pub;
int seq;
double tx_power_db;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void rfid_callback(ConstGzStringPtr &_msg){
    // We got a new message from the Gazebo sensor.  Stuff a
    // corresponding ROS message and publish it.

    static const std::string fieldSep_ = (":");
    static const std::string entrySep_ = ("\n");

    rfid_node::TagReading tag_msg;
    tag_msg.header.frame_id = ros_rfid_frame_id;

    //split each reading 
    std::vector<std::string> readings;
    std::vector<std::string> reading_fields;
    boost::split(readings, _msg->data(), boost::is_any_of(entrySep_));
    
    for (auto reading: readings){
        boost::split(reading_fields, reading, boost::is_any_of(fieldSep_));
        if (reading_fields.size()==5){
          tag_msg.header.stamp = ros::Time::now();
          tag_msg.ID = reading_fields[0];
          tag_msg.frequency = int ( atof(reading_fields[1].c_str()) /1000.0); // Internally we use Hz., but rfid_node uses KHz as unit for frequency
          tag_msg.rssi  = int ( atof(reading_fields[2].c_str()) + 30.0) ; // Internally we use dB, but rfid_node uses dBm as unit for RSSI
          tag_msg.txP  = int ( (atof(reading_fields[3].c_str()) + 30.0 ) * 100.0 ) ; // Internally we use dB, but rfid_node uses (100*dBm) as unit for transmitted power
          tag_msg.phase  = int ( atof(reading_fields[4].c_str()) * 180.0/M_PI ); // Internally we use radians, but rfid_node uses degrees as unit for received phase          
          tag_msg.timestamp = tag_msg.header.stamp;          
          ros_rfid_pub.publish(tag_msg);            
        }
    }

  }
/////////////////////////////////////////////////
int main(int _argc, char** _argv)
{

  ros::init(_argc, _argv, "wirelessNodeMonitor");
  ros::NodeHandle nh;

  // read configuration
  ros::param::param<std::string>("~ros_rfid_topic_name",
                                 ros_rfid_topic_name, "/lastTag");
  ros::param::param<std::string>("~ros_rfid_frame_id",
                                    ros_rfid_frame_id, "lastTag");
  ros::param::param<double>("~tx_power_db",
                                    tx_power_db, 1);

  ros::param::param<std::string>("~gazebo_wireless_node_topic_name",
                                 gazebo_wireless_node_topic_name, "/gazebo/default/thorvald_001/base_link/_head_reader_sensor/transceiver");


  ros_rfid_pub = nh.advertise<rfid_node::TagReading>(ros_rfid_topic_name, 5);

  seq=0;
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Check if the topic to subscribe is present within the list, and
  // create the subscriber only later
  bool topic_found = false;
  bool one_error = false;
  while (topic_found == false)
  {
    std::list<std::string> topics_list =
        gazebo::transport::getAdvertisedTopics("");
    for (std::list<std::string>::iterator topic = topics_list.begin();
         topic != topics_list.end(); topic++)
    {
      if ((*topic).compare(gazebo_wireless_node_topic_name) == 0)
      {
        topic_found = true;
        break;
      }
    }
    ROS_WARN("[%s] Topic [%s] not found. Waiting 1 sec. for it to be available",ros::this_node::getName().c_str(), gazebo_wireless_node_topic_name.c_str());
    one_error= true;
    ros::Duration(1).sleep();
  }

  gazebo::transport::SubscriberPtr sub =
      node->Subscribe(gazebo_wireless_node_topic_name, rfid_callback);

  if (one_error){
        ROS_WARN("[%s] Topic [%s] was finally found",ros::this_node::getName().c_str(), gazebo_wireless_node_topic_name.c_str());
  }

  ros::spin();

  // Make sure to shut everything down.
  gazebo::transport::fini();
}
