#include <gazebo_rfid/RFIDReaderRosPlugin.h>

namespace gazebo {
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosRFIDReader)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosRFIDReader::GazeboRosRFIDReader(){
    this->seed = 0;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosRFIDReader::~GazeboRosRFIDReader(){
    this->rosnode_->shutdown();
    delete this->rosnode_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void GazeboRosRFIDReader::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf){
    // load plugin
    SensorPlugin::Load(_parent, this->sdf);
    // Get the world name.
    std::string worldName = _parent->WorldName();
    this->world_ = physics::get_world(worldName);
    // save pointers
    this->sdf = _sdf;

    GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
    this->parent_reader_sensor_ = dynamic_pointer_cast<sensors::RFIDReader>(_parent);

    if (!this->parent_reader_sensor_){
      gzthrow("GazeboRosRFIDReader controller requires a RFIDReader Sensor as its parent");
    }

    this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "RFIDReader");

    if (!this->sdf->HasElement("frameName")){
      ROS_INFO_NAMED("RFIDReader", "RFIDReader plugin missing <frameName>, defaults to /world");
      this->frame_name_ = "/world";
    } else {
      this->frame_name_ = this->sdf->Get<std::string>("frameName");
    }


    if (!this->sdf->HasElement("topicName")){
      ROS_INFO_NAMED("RFIDReader", "RFIDReader plugin missing <topicName>, defaults to /lastTag");
      this->topic_name_ = "/lastTag";
    }
    else{
      this->topic_name_ = this->sdf->Get<std::string>("topicName");
    }

    this->connect_count_ = 0;

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()){
      ROS_FATAL_STREAM_NAMED("RFIDReader", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    ROS_INFO_NAMED("RFIDReader", "Starting RFIDReader Plugin (ns = %s)", this->robot_namespace_.c_str() );
    // ros callback queue for processing subscription
    this->deferred_load_thread_ = boost::thread(boost::bind(&GazeboRosRFIDReader::LoadThread, this));

  }

  ////////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void GazeboRosRFIDReader::LoadThread()
  {
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init(this->world_name_);

    this->pmq.startServiceThread();

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    if (this->topic_name_ != ""){
      ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<rfid_node::TagReading>( this->topic_name_, 1, boost::bind(&GazeboRosRFIDReader::ClientConnect, this), boost::bind(&GazeboRosRFIDReader::ClientDisconnect, this), ros::VoidPtr(), NULL);
      this->pub_ = this->rosnode_->advertise(ao);
      this->pub_queue_ = this->pmq.addPub<rfid_node::TagReading>();
    }

    // Initialize the controller

    // sensor generation off by default
    this->parent_reader_sensor_->SetActive(false);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Increment count
  void GazeboRosRFIDReader::ClientConnect(){
    this->connect_count_++;
    if (this->connect_count_ == 1){
      this->rfid_scan_sub_ = this->gazebo_node_->Subscribe(this->parent_reader_sensor_->Topic(), &GazeboRosRFIDReader::OnScan, this);
    }
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  // Decrement count
  void GazeboRosRFIDReader::ClientDisconnect()
  {
    this->connect_count_--;
    if (this->connect_count_ == 0)
      this->rfid_scan_sub_.reset();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Convert new Gazebo message to ROS message and publish it
  void GazeboRosRFIDReader::OnScan(ConstGzStringPtr &_msg)
  {
    // We got a new message from the Gazebo sensor.  Stuff a
    // corresponding ROS message and publish it.

      static const std::string fieldSep_ = (":");
      static const std::string entrySep_ = ("\n");

    rfid_node::TagReading tag_msg;
    tag_msg.header.frame_id = this->frame_name_;

    //split each reading 
    std::vector<std::string> readings;
    std::vector<std::string> reading_fields;
    boost::split(readings, _msg->data(), boost::is_any_of(entrySep_));

    for (auto reading: readings){
        boost::split(reading_fields, reading, boost::is_any_of(fieldSep_));
        tag_msg.header.stamp = ros::Time::now();
        tag_msg.ID = reading_fields[0];
        tag_msg.frequency = int ( atof(reading_fields[1].c_str()) /1000.0); // Internally we use Hz., but rfid_node uses KHz as unit for frequency
        tag_msg.rssi  = int ( atof(reading_fields[2].c_str()) + 30.0) ; // Internally we use dB, but rfid_node uses dBm as unit for RSSI
        tag_msg.txP  = int ( (atof(reading_fields[3].c_str()) + 30.0 ) * 100.0 ) ; // Internally we use dB, but rfid_node uses (100*dBm) as unit for transmitted power
        tag_msg.phase  = int ( atof(reading_fields[4].c_str()) * 180.0/M_PI ); // Internally we use radians, but rfid_node uses degrees as unit for received phase
        tag_msg.timestamp = tag_msg.header.stamp;
        
        this->pub_queue_->push(tag_msg, this->pub_);
    }

  }

}