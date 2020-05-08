#include "rfid_grid_map/rfid_gridMap.hpp"


namespace rfid_grid_map {


    rfid_gridMap::rfid_gridMap(ros::NodeHandle& n)
    : nodeHandle_(n)
    {

        loadROSParams();

        // enable readings
        isReadingEnabled_ = true;

        getMapDimensions();

        //debug  
        ROS_DEBUG("Debugging initial pose...");      
        updateRobotPose();
        ROS_DEBUG_STREAM("Robot starts at pose: (" << robot_x_ << ", " << robot_y_ << ") m. " << robot_h_ * 180.0/M_PI << " deg. ");
        model_.saveProbMapDebug("/tmp/test/",-1, -1 ,robot_x_, robot_y_, robot_h_);

        // Param load and setup finished....................................
        showROSParams();

        // topic publishers .........................................................................................................
        rfid_belief_topic_pub_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(rfid_belief_topic_name_, 1, true);

        // topic subscribers .........................................................................................................

        // tag readings
        rfid_readings_topic_sub_ = nodeHandle_.subscribe(rfid_readings_topic_name_, 1000, &rfid_gridMap::rfid_readings_topic_callback, this);

        // service servers ......................................................................................................
        rfid_belief_srv_ss_ = nodeHandle_.advertiseService(rfid_belief_srv_name_, &rfid_gridMap::rfid_belief_srv_callback, this);
      
        // threads  ......................................................................................................
        ros::AsyncSpinner spinner(0);
        spinner.start();

        // mfc consider creating multiple processing threads ....
        // Create a thread using member function
        boost::thread* thr = new boost::thread(boost::bind(&rfid_gridMap::do_stuff, this));
    	
        ros::waitForShutdown();
        thr->join();
    }

    void rfid_gridMap::do_stuff(){
        type_measurement reading;
        double x, y, rh, rxPower_dB, rxPhase_rad, rxFreq_Hz, numDetections, txPower_dB;

        while (ros::ok()){
            readings_queue_.consume(reading);

            // only for debug purposes
            // ROS_DEBUG_STREAM("Processing reading # " << reading.numDetections << 
            //                  " from tag ("<<reading.tagID <<" @ "<< reading.txPower_dB << " dB, "<< reading.rxFreq_Hz/1e6 << " MHz.) " <<
            //                   "at pose: (" << reading.x_m << ", " << reading.y_m << ") m. " << reading.th_deg  << " deg. :" << 
            //                    reading.rxPower_dB <<"  dB, " << reading.rxPhase_rad * 180.0/M_PI << " degs  ==> Queue["<< readings_queue_.length() <<"]" );
            model_.addMeasurement(reading.x_m, reading.y_m, reading.th_deg, reading.rxPower_dB, reading.rxPhase_rad, reading.rxFreq_Hz, reading.tagNum, reading.txPower_dB);
            model_.saveProbMapDebug("/tmp/test/",reading.tagNum, reading.numDetections,reading.x_m,reading.y_m, reading.th_deg * M_PI/180.0);
            // make a video out of this shit            
            // ffmpeg -r 10 -f image2 -s 1920x1080 -start_number 000 -i T0_S%03d_tempMap.png -vcodec libx264 -crf 25 -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2"  -pix_fmt yuv420p test0.mp4
        }
    }

    void rfid_gridMap::showROSParams(){
      
      //...........................................
      ROS_DEBUG_STREAM("Current logger is:["<< ROSCONSOLE_DEFAULT_NAME <<"]");

      ROS_DEBUG("Configuration params:");

      ROS_DEBUG("\tGlobal Config ________________________");      

      ROS_DEBUG("\t\tMap Resolution: %2.1f", map_resolution_);

      ROS_DEBUG("\tROS Params________________________");
      ROS_DEBUG("\t\tstatic map topic name: %s", map_topic_name_.c_str());
      ROS_DEBUG("\t\tstatic map service name: %s", map_service_name_.c_str());
      ROS_DEBUG("\t\trfid readings topic name: %s", rfid_readings_topic_name_.c_str());
      ROS_DEBUG("\t\trfid belief maps service name: %s", rfid_belief_srv_name_.c_str());
      ROS_DEBUG("\trobot frame id: %s", robot_frame_.c_str());

    }

    void rfid_gridMap::getMapDimensions(){
        isMapLoaded_=false;
        bool failed_once=false;
        // connect to map server and get dimensions and resolution (meters)
        while (!isMapLoaded_){
            // Try getting map properties from topic
            boost::shared_ptr<nav_msgs::OccupancyGrid const> mapPointer;

            mapPointer=ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic_name_,nodeHandle_,ros::Duration(0.5));

            if(mapPointer==NULL){
                ROS_ERROR("Topic \"map\" not available, trying service ");
                failed_once=true;
            }else{
                map_topic_callback(*mapPointer);
            }

            if (!isMapLoaded_){
                // Try getting map properties from service...
                if (ros::service::waitForService(map_service_name_,500)) {
                    nav_msgs::GetMap::Request  req;
                    nav_msgs::GetMap::Response resp;
                    ros::service::call(map_service_name_, req, resp);
                    if(failed_once){
                        ROS_ERROR("Service replied!");
                    }
                    map_topic_callback(resp.map);
                } else {
                    ROS_ERROR("Service at \"%s\" unavailable too, trying topic again ...", map_service_name_.c_str());
                }
            }
        }
    }

    void rfid_gridMap::loadROSParams(){
        ros::NodeHandle private_node_handle("~");

        // LOAD ROS PARAMETERS ....................................
        std::string temp;
        
        private_node_handle.param("sigma_power", temp,std::string("2.0"));
        sigma_power_=std::stod(temp);
        private_node_handle.param("sigma_phase", temp,std::string("2.0"));
        sigma_phase_=std::stod(temp);
        private_node_handle.param("rfidgrid_resolution", temp,std::string("0.025"));
        map_resolution_=std::stod(temp);
        
        private_node_handle.param("rfid_readings_topic_name", rfid_readings_topic_name_, std::string("/lastTag"));
        private_node_handle.param("map_topic_name", map_topic_name_, std::string("/map"));
        private_node_handle.param("map_service_name", map_service_name_, std::string("/static_map"));

        private_node_handle.param("robot_frame", robot_frame_, std::string("base_link"));

        private_node_handle.param("rfid_belief_srv_name", rfid_belief_srv_name_, std::string("grid_map"));       
        private_node_handle.param("rfid_belief_topic_name",rfid_belief_topic_name_, std::string("rfid_belief_maps"));       

    }

    rfid_gridMap::~rfid_gridMap(){
    }

    void rfid_gridMap::rfid_readings_topic_callback(const rfid_node::TagReading::ConstPtr& msg){

        if (isReadingEnabled_){
            // tag reading data from msg
            double txPower_dB, rxPower_dB; 
            double rxPhase_rad; 
            double rxFreq_Hz;
            int tagNum, numDetections;
            
            //update robot position
            updateRobotPose();

            if (msg->txP>0){
                txPower_dB=(msg->txP/100) - 30.0 ; //  transmitted power from reader is given in (100*dBm)
            } else {
                // comes from simulation or a weirder place
                txPower_dB=-10.0; // dB
            }

            rxPower_dB=msg->rssi - 30.0 ;  // RSSI is given in dBm
            rxPhase_rad = msg->phase * M_PI/180.0;  // phase is given in degrees
            rxFreq_Hz = msg->frequency*1000.0; // freq is given in KHz

            // ROS_DEBUG_STREAM("Adding reading # " << numDetections_ << 
            //                  " from tag ("<<msg->ID<<" @ "<< txPower_dB << " dB, "<< rxFreq_Hz/1e6 << " MHz.) " <<
            //                  "at pose: (" << robot_x_ << ", " << robot_y_ << ") m. " << robot_h_ * 180.0/M_PI << " deg. :" << 
            //                  rxPower_dB <<"  dB, " << msg->phase<< " degs ==> Queue["<< readings_queue_.length() <<"]" );

            // Find tag number for RadarModel and account for num of detections
            auto search = tagID_detections_map_.find(msg->ID);        
            if (search != tagID_detections_map_.end()) {
                tagID_detections_map_[msg->ID] = tagID_detections_map_[msg->ID] + 1;
            } else {
                tagID_detections_map_[msg->ID] = 1;
                tagID_enumeration_map_[msg->ID] = tagID_enumeration_map_.size()-1; //counts after creating the entry ...
            }        
            numDetections = tagID_detections_map_[msg->ID];
            tagNum = tagID_enumeration_map_[msg->ID];            

            // store for further process
            type_measurement reading;
            reading.x_m = robot_x_; 
            reading.y_m = robot_y_; 
            reading.th_deg =robot_h_ * 180.0/M_PI; 
            reading.rxPower_dB = rxPower_dB; 
            reading.rxPhase_rad = rxPhase_rad; 
            reading.rxFreq_Hz = rxFreq_Hz; 
            reading.txPower_dB = txPower_dB;
            reading.tagID = msg->ID;
            reading.tagNum = tagNum;
            reading.numDetections = numDetections;

            readings_queue_.add(reading);       
        }
    }


    void rfid_gridMap::updateRobotPose(){
        // robot position
        double roll, pitch, yaw;

        try{
            listener_.waitForTransform(map_frame_id_, robot_frame_, ros::Time(0), ros::Duration(0.5) );
            listener_.lookupTransform(map_frame_id_, robot_frame_, ros::Time(0), transform_);
            robot_x_=transform_.getOrigin().x();
            robot_y_=transform_.getOrigin().y();
            tf::Quaternion  q=transform_.getRotation();
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            robot_h_=yaw;            
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    bool rfid_gridMap::rfid_belief_srv_callback(rfid_grid_map::GetBeliefMaps::Request  &req, rfid_grid_map::GetBeliefMaps::Response &res){
      bool isSuccess;
      isSuccess = true;
      
      ros::Time begin = ros::Time::now();      
      // Stop reading more tags..........................................................
      ROS_WARN_STREAM("RFID Belief map service called. Tag reading will be disabled now." );
      isReadingEnabled_ = false;

      // Print queue size................................................................
      ROS_DEBUG_STREAM("Still need to process [" << readings_queue_.length() << "] more readings:" );
      
      for (auto const& x : tagID_detections_map_){
              ROS_DEBUG_STREAM( "\t - Tag num [" << tagID_enumeration_map_[x.first] << "] " <<
                                "ID ["  << x.first << "] " <<
                                "Total detections ["  << x.second << "] ") ;
      }

    
      // Finish Map building process.....................................................
      readings_queue_.waitEmpty();
      ROS_DEBUG_STREAM("Queue is empty, creating reply" );

      // Store gridmaps into response....................................................
      model_._rfid_belief_maps.setTimestamp(ros::Time::now().toNSec());      
      grid_map::GridMapRosConverter::toMessage(model_._rfid_belief_maps, res.rfid_maps);

      // Publish as topic too for easiness
      rfid_belief_topic_pub_.publish(res.rfid_maps);

      // Resume reading..................................................................
      isReadingEnabled_ = true;
      ros::Time end = ros::Time::now();
      // Print queue size................................................................
      ROS_INFO_STREAM("Resuming operations. Service took (" << (end-begin).toSec() << ") secs" );

      return isSuccess;
    }



    // we get information from our global map
    void rfid_gridMap::map_topic_callback(const nav_msgs::OccupancyGrid& msg){
        if (!isMapLoaded_){
            if ((msg.info.width>0.0)&&(msg.info.height>0.0)){
                isMapLoaded_=true;
                mapDesc_=msg.info;
                map_frame_id_=msg.header.frame_id;
                model_ = RadarModelROS(msg, sigma_power_, sigma_phase_, map_resolution_ );

                ROS_DEBUG("Received a %d X %d map @ %.3f m/pix  Origin X %.3f Y %.3f\n",
                        msg.info.width,
                        msg.info.height,
                        msg.info.resolution,
                        msg.info.origin.position.x,
                        msg.info.origin.position.y);

            } else {
                ROS_ERROR("Received an INVALID!! %d X %d map @ %.3f m/pix  Origin X %.3f Y %.3f\n",
                        msg.info.width,
                        msg.info.height,
                        msg.info.resolution,
                        msg.info.origin.position.x,
                        msg.info.origin.position.y);
            }
        } 
    }

} // end of namespace rfid_grid_map