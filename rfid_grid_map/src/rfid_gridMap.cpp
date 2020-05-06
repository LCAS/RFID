#include "rfid_grid_map/rfid_gridMap.hpp"


namespace rfid_grid_map {


    rfid_gridMap::rfid_gridMap(ros::NodeHandle& n)
    : nodeHandle_(n)
    {

        loadROSParams();

        getMapDimensions();

        if (loadGrids_) {
            model_.loadBelief(save_route_+gridmap_image_file_);  
        }

        //debug        
        updateRobotPose();
        ROS_DEBUG_STREAM("Robot starts at pose: (" << robot_x_ << ", " << robot_y_ << ") m. " << robot_h_ * 180.0/M_PI << " deg. ");
        model_.saveProbMapDebug("/tmp/test/",0, 0 ,robot_x_, robot_y_, robot_h_);

        // Param load and setup finished....................................
        showROSParams();

        lastRegion_.name=std::string(" ");

        // publishers .........................................................................................................
        rfid_belief_topic_pub_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(rfid_belief_topic_name_, 1, true);
        
        publishMap();

        prob_topic_pub_ = nodeHandle_.advertise<std_msgs::String>(prob_topic_name_, 1000);

        // subscribers .........................................................................................................

        // tag readings
        rfid_readings_topic_sub_ = nodeHandle_.subscribe(rfid_readings_topic_name_, 1000, &rfid_gridMap::rfid_readings_topic_callback, this);

        // Periodic tasks  .........................................................................................................
        
        // publish rfid belief map
        belief_publishing_timer_ = nodeHandle_.createTimer(ros::Duration(belief_publishing_period_), &rfid_gridMap::belief_publishing_callback,this);
        // publish location probabilities
        prob_publishing_timer_ = nodeHandle_.createTimer(ros::Duration(prob_publishing_period_), &rfid_gridMap::prob_publishing_callback,this);
        // save temporal probability maps
        belief_saving_timer_ = nodeHandle_.createTimer(ros::Duration(belief_saving_period_),  &rfid_gridMap::belief_saving_callback,this);

        ros::AsyncSpinner spinner(0);
        spinner.start();

        // Create a thread using member function
        boost::thread* thr = new boost::thread(boost::bind(&rfid_gridMap::do_stuff, this));
    	
        ros::waitForShutdown();
        thr->join();
    }

    void rfid_gridMap::do_stuff(){
        std::vector<double> reading(8);
        double x, y, rh, rxPower_dB, rxPhase_rad, rxFreq_Hz, numDetections, txPower_dB;

        while (ros::ok()){
            readings_queue_.consume(reading);
            x = reading[0]; 
            y = reading[1]; 
            rh = reading[2]; 
            rxPower_dB = reading[3]; 
            rxPhase_rad = reading[4]; 
            rxFreq_Hz = reading[5]; 
            numDetections = reading[6]; 
            txPower_dB = reading[7];
            ROS_DEBUG_STREAM("Processing reading # " << numDetections << 
                             " from tag ("<<tagID_<<" @ "<< txPower_dB << " dB, "<< rxFreq_Hz/1e6 << " MHz.) " <<
                              "at pose: (" << x << ", " << y << ") m. " << rh * 180.0/M_PI  << " deg. :" << 
                               rxPower_dB <<"  dB, " << rxPhase_rad * 180.0/M_PI << " degs  ==> Queue["<< readings_queue_.length() <<"]" );


            model_.addMeasurement(x, y, rh* 180.0/M_PI, rxPower_dB, rxPhase_rad, rxFreq_Hz, 0, txPower_dB);
            model_.saveProbMapDebug("/tmp/test/",0, (int) numDetections,x,y, rh);
            // make a video out of this shit            
            // ffmpeg -r 10 -f image2 -s 1920x1080 -start_number 000 -i T0_S%03d_tempMap.png -vcodec libx264 -crf 25 -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2"  -pix_fmt yuv420p test0.mp4
        }
    }

    void rfid_gridMap::showROSParams(){
      
      //...........................................
      ROS_DEBUG_STREAM("Current logger is:["<< ROSCONSOLE_DEFAULT_NAME <<"]");

      ROS_DEBUG("Configuration params:");

      ROS_DEBUG("\tGlobal Config ________________________");      
      ROS_DEBUG("\t\tTagID: %s", tagID_.c_str());
      ROS_DEBUG("\t\tTracked object name is: %s", object_name_.c_str());
      ROS_DEBUG("\t\tMap Resolution: %2.1f", map_resolution_);

      ROS_DEBUG("\tStorage Config ________________________");      
      if (loadGrids_){
        ROS_DEBUG("\t\tLoad Saved belief data [TRUE]");
      }else{
        ROS_DEBUG("\t\tLoad Saved belief data [FALSE]");
      }
      ROS_DEBUG("\t\tLoad/save route is [%s]",save_route_.c_str());
      ROS_DEBUG("\t\tLoad/save file is [%s]",gridmap_image_file_.c_str());

      ROS_DEBUG("\tUpdate frequencies ________________________");
      ROS_DEBUG("\t\tbelief_publishing_period: %2.1f", belief_publishing_period_);
      ROS_DEBUG("\t\tprob_publishing_period: %2.1f", prob_publishing_period_);
      ROS_DEBUG("\t\tbelief_saving_period: %2.1f",  belief_saving_period_);

      ROS_DEBUG("\tROS Params________________________");
      ROS_DEBUG("\t\tstatic map topic name: %s", map_topic_name_.c_str());
      ROS_DEBUG("\t\tstatic map service name: %s", map_service_name_.c_str());
      ROS_DEBUG("\t\trfid readings topic name: %s", rfid_readings_topic_name_.c_str());
      ROS_DEBUG("\t\trfid belief map topic name: %s", rfid_belief_topic_name_.c_str());
      ROS_DEBUG("\t\tlocation prob. map topic name: %s", prob_topic_name_.c_str());
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

        private_node_handle.param("tagID", tagID_, std::string("390000010000000000000007"));
        private_node_handle.param("rfid_belief_topic_name", rfid_belief_topic_name_, std::string("grid_map"));
        private_node_handle.param("prob_topic_name", prob_topic_name_, std::string("probs"));

        private_node_handle.param("saveRoute", save_route_,std::string(""));

        private_node_handle.param("loadGrids", temp, std::string("not found"));

        if (boost::iequals(temp, std::string("true"))) {
            loadGrids_=true;
        } else {
            loadGrids_=false;
        }

        if (boost::iequals(temp, std::string("not found"))) {
            loadGrids_=false;
        }

        private_node_handle.param("belief_publishing_period", temp,std::string("5.0"));
        belief_publishing_period_=std::stod(temp);
        private_node_handle.param("prob_publishing_period", temp, std::string("1.0"));
        prob_publishing_period_=std::stod(temp);
        private_node_handle.param("belief_saving_period", temp, std::string("10.0"));
        belief_saving_period_=std::stod(temp);

        numDetections_=0;

        private_node_handle.param("object", object_name_, std::string("noname_object"));

        gridmap_image_file_=object_name_+"_grid.png";

        // load ROS parameters describing zones of interest
        loadZois();
    }

    rfid_gridMap::~rfid_gridMap(){
        ros::TimerEvent ev;
        rfid_gridMap::belief_saving_callback(ev);
    }

    void rfid_gridMap::rfid_readings_topic_callback(const rfid_node::TagReading::ConstPtr& msg){

        // tag reading data from msg
        double txPower_dB, rxPower_dB; 
        double rxPhase_rad; 
        double rxFreq_Hz;
        
       if (msg->ID.compare(tagID_)==0){
        // count as a good one
        numDetections_+=1;
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

        // store last place where robot was when tag was detected
        updateLastDetectionPose(robot_x_,robot_y_);
        std::vector<double> reading(8);
        reading[0]=robot_x_; 
        reading[1]=robot_y_; 
        reading[2]=robot_h_; 
        reading[3]=rxPower_dB; 
        reading[4]=rxPhase_rad; 
        reading[5]=rxFreq_Hz; 
        reading[6]=numDetections_;
        reading[7]=txPower_dB;
        readings_queue_.add(reading);

       }

    }

    void rfid_gridMap::belief_saving_callback(const ros::TimerEvent&){
        model_.saveBelief(save_route_+gridmap_image_file_);
    }

    void rfid_gridMap::belief_publishing_callback(const ros::TimerEvent&){
        publishMap();
     }


/*
 *  Loads zois as poligons.
 *  A subzoi will be identified by having a '_' in the middle of its name: [zoiName]_[subZoiName]
 * */
    void rfid_gridMap::loadZois(){
        XmlRpc::XmlRpcValue zoi_keys;
        std::string submapParam="/mmap/zoi/submap_0/";
        std::string numSubMapParam="/mmap/numberOfSubMaps";
        int numSubmaps;
        std::string zoiPointName;
        std::string zoiName;
        unsigned int zoiPoint_num;
        double px=0;
        double py=0;
        std::size_t slashPos;
        Position p;

        std::map<std::string,rfid_gridMap::type_area>::iterator map_it;

        //ROS_ASSERT_MSG(nodeHandle_.getParam(numSubMapParam, numSubmaps),"Can't determine number of sub maps from rosparam [/mmap/numberOfSubMaps] ");
        if (!nodeHandle_.getParam(numSubMapParam, numSubmaps)){
            numSubmaps=1;
            ROS_ERROR("Can't determine number of sub maps from rosparam [/mmap/numberOfSubMaps]. Assuming 1 ");
        }

        ROS_ASSERT_MSG(numSubmaps==1,"Number of submaps different from 1 [%d]. Aborting",numSubmaps);

        ROS_ASSERT_MSG(nodeHandle_.getParam(submapParam,zoi_keys),"Can't get zoi rosparam [/mmap/zoi/submap_0/] ");

        ROS_ASSERT_MSG(zoi_keys.getType() == XmlRpc::XmlRpcValue::TypeStruct,"YAML ZOI rosparam has unknown format. ZOI rosparam [/mmap/zoi/submap_0/] is not a struct ");

        // this should be a list of tuples like this:
        //            zoiName_pointNumber : { submapName, zoiName, posX, posY  }
        //  Zoi name follows format:
        //             zoi name-zoi subarea
        // subareas are parts of bigger zois, they must have same 'zoi name' before the slash.
        //  i.e.
        //          kitchen-table
        //          kitchen-cook    both are kitchen zoi, subareas cook and table
        for(XmlRpc::XmlRpcValue::iterator itr = zoi_keys.begin(); itr != zoi_keys.end(); itr++){
            try{
                zoiPointName = itr->first;
                //std::cout<< "point name: " << zoi_point_name<< "\n";

                // zoi point number is last thing after '_'
                slashPos = zoiPointName.find_last_of("_");
                if (slashPos!=std::string::npos)
                {
                    //std::cout << " zoi name: " << zoiPointName.substr(0,slashPos) << '\n';
                    //std::cout << " point num: " << zoiPointName.substr(slashPos+1) << '\n';
                    zoiPoint_num=std::stoi(zoiPointName.substr(slashPos+1));

                    //region
                    zoiName = static_cast<std::string>(itr->second[1]);
                    px =  itr->second[2];
                    py =  itr->second[3];

                    map_it = mapAreas.find(zoiName);
                    if (map_it == mapAreas.end())
                    {
                        type_area area;
                        area.name=zoiName;
                        area.prob=0.0;
                        mapAreas[zoiName]=area;
                    }

                    p=Position(px,py);
                    mapAreas[zoiName].polygon.addVertex(p);

                    /* lets hope vertex are added propertly...

                    // check if we have space
                    if (zoiPoint_num+1>mapAreas[zoiName].polygon.vertices_.size())
                    {
                        mapAreas[zoiName].polygon.vertices_.resize(zoiPoint_num+3);
                    }
                    mapAreas[zoiName].polygon.vertices_[zoiPoint_num]=p;
                    */


                }
                else
                {// something is wrong with the zoiPoint name, does not have slash?
                    ROS_ERROR("Ommiting zoi point name: It does not contain '_': [%s]", zoiPointName.c_str());
                }
            }
            catch(XmlRpc::XmlRpcException e){
                std::cout <<  "ERROR:" << e.getMessage ()<<" .... \n";
            }

        }

        //ROS_DEBUG("Loaded [%lu] zones:", mapAreas.size());
        //for (std::map<std::string,rfid_gridMap::type_area>::iterator mapIt=mapAreas.begin(); mapIt!=mapAreas.end(); ++mapIt)
        //{
        //    ROS_DEBUG("- [%s]: %lu points", mapIt->first.c_str(),mapIt->second.polygon.nVertices() );
        //}

    }

    bool rfid_gridMap::isSubregion(std::string zoiName,std::string &parent){
        std::string posibleParent;
        for (std::map<std::string,rfid_gridMap::type_area>::iterator mapIt=mapAreas.begin(); mapIt!=mapAreas.end(); ++mapIt)
        {
            posibleParent = mapIt->first;
            std::size_t found = zoiName.find(posibleParent);
            if ((found!=std::string::npos)&&( posibleParent.compare(zoiName) != 0 ))
            {
                parent = posibleParent;
                //ROS_DEBUG("[%s] is subregion of [%s], at pos %lu",zoiName.c_str(),parent.c_str(),found);
                return true;
            }
        }
        return false;
    }

    void rfid_gridMap::prob_publishing_callback(const ros::TimerEvent&){
        double total=0;

        double val=0;
        std::stringstream sstream;
        int i;
        std::string father;
        total=0;

        // subzois indexed by zoi name
        std::map<std::string,double>::iterator reg_it;
        std::map<std::string,double> regionsCount;

        double min_d=10000.0;
        double d=0.0;

        //ROS_DEBUG("Region weights:  " );
        //count weigh in each region
        for (std::map<std::string,rfid_gridMap::type_area>::iterator mapIt=mapAreas.begin(); mapIt!=mapAreas.end(); ++mapIt){
             val=countValuesInArea(mapIt->second.polygon);
             mapIt->second.prob=val;

             // if it's not a subregion, cumulated weight goes to total
             if (!isSubregion(mapIt->first,father)){
                total+=val;
                //ROS_DEBUG("- [%s]: %3.3f  ", mapIt->first.c_str(),val );

             //if it's a subregion, cumulated weight goes to its sublist
             }else{
                //ROS_DEBUG("x [%s]: %3.3f  ", mapIt->first.c_str(),val );

                reg_it = regionsCount.find(father);
                if (reg_it == regionsCount.end()){
                    regionsCount[father]=0.0;
                }
                regionsCount[father]+=val;
            }

             // also check if robot is inside this region
             // use polygon method to check points inside
             wasHere(mapIt->second);

             // use minimum distance to centroid: works better
             d = (mapIt->second.polygon.getCentroid() - lastDetectPose_).norm();
             if (d<=min_d) {
                 lastRegion_=mapIt->second;
                 min_d=d;
            }
        }

       // first element in published probs is latest region with invalid prob.
       sstream<<lastRegion_.name+",-1";
        //ROS_DEBUG("Total weight: %3.3f  ", total );
        //ROS_DEBUG("Region probs:  " );

        //then, each region with its probability
        //for (std::size_t i=0;i<mapAreas.size();i++)
       for (std::map<std::string,rfid_gridMap::type_area>::iterator mapIt=mapAreas.begin(); mapIt!=mapAreas.end(); ++mapIt)
       {

            if (total>0.0)
            {
                if (!isSubregion(mapIt->first,father)){
                    mapIt->second.prob=mapIt->second.prob/total;
                } else {
                    // this would be relative probability inside its region
                    mapIt->second.prob=mapIt->second.prob/regionsCount[father];
                    // now we turn this into global probability...
                    mapIt->second.prob*= mapAreas[father].prob;
                }
            }else{
                mapIt->second.prob=0;
            }
            //ROS_DEBUG("- [%s]: %3.3f  ", mapIt->first.c_str(),mapIt->second.prob );
            sstream<<","<<mapIt->second.name<<","<<mapIt->second.prob;

        }

        // Publish stream of probs.
        std_msgs::String msg;
        msg.data=sstream.str();
        prob_topic_pub_.publish(msg);

        //ROS_DEBUG("- [total]: %3.3f ", total );


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

    void rfid_gridMap::publishMap(){

      model_._rfid_belief_maps.setTimestamp(ros::Time::now().toNSec());

      grid_map_msgs::GridMap message;
      grid_map::GridMapRosConverter::toMessage(model_._rfid_belief_maps, message);
      rfid_belief_topic_pub_.publish(message);

    }

    double rfid_gridMap::countValuesInArea(Polygon pol){
      double total=0.0;
      grid_map::PolygonIterator iterator(model_._rfid_belief_maps, pol);

      total = model_.getTotalWeight(iterator, 0);

      return total;
    }

    void rfid_gridMap::updateLastDetectionPose(double x, double y){
        lastDetectPose_=Position(x,y);
    }

    void rfid_gridMap::wasHere(type_area area){
        ////ROS_DEBUG("Point (%3.3f,%3.3f)", lastDetectPose_.x(),lastDetectPose_.y());
        if (area.polygon.isInside(lastDetectPose_)){
            lastRegion_=area;
            ////ROS_DEBUG("We are in area %s", area.name.c_str());
        } else {
            ////ROS_DEBUG("We are NOT in area %s", area.name.c_str());
        }

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
        // else {
        //     map_topic_sub_.shutdown();
        // }
    }

} // end of namespace rfid_grid_map