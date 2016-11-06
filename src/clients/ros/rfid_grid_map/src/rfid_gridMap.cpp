    
/**
 * ToDo: parametrize decay time, prob publish time, intensity
 * 
 * 
 * */


#include "rfid_grid_map/rfid_gridMap.hpp"


namespace rfid_grid_map {
    rfid_gridMap::rfid_gridMap(ros::NodeHandle& n)
    : nodeHandle_(n)
    {
      ros::NodeHandle private_node_handle("~");
        
      //ROS_INFO("HI WORLD");
      
      // map resolution
      double resolution;
      // grid map topic name
      std::string grid_map_name;
            
      // region description yaml file
      std::string regions_file;
          
      std::string prob_pub_name;
      
      bool loadGrids;
      
      //2d position of the grid map in the grid map frame [m].  
      double orig_x=0;
      double orig_y=0;      
      // LOAD ROS PARAMETERS ....................................
      std::string temp;

      private_node_handle.param("global_frame", global_frame, std::string("/map"));    
      private_node_handle.param("robot_frame", robot_frame, std::string("base_link"));
      
      private_node_handle.param("map_resolution", temp,std::string("0.1"));
      resolution=std::stod(temp);

      private_node_handle.param("weight_inc", temp,std::string("0.005"));
      weight_inc=std::stod(temp);

      private_node_handle.param("weight_dec", temp,std::string("0.001"));
      weight_dec=std::stod(temp);


      private_node_handle.param("tagID", tagID, std::string("390000010000000000000007"));
      private_node_handle.param("grid_map_name", grid_map_name, std::string("grid_map"));
      private_node_handle.param("prob_pub_name", prob_pub_name, std::string("probs"));

      private_node_handle.param("saveRoute", save_route,std::string(""));
           
      
      private_node_handle.param("detectRadius", temp,std::string("20.0"));
      detectRadius=std::stod(temp);
      

      private_node_handle.param("loadGrids", temp,std::string("false"));
      if (boost::iequals(temp, std::string("true"))) {
          loadGrids=true;
      }
      
      double saveTime;
      std::string object_name;
      
      private_node_handle.param("saveTime", saveTime, 10.0);
      private_node_handle.param("object", object_name, std::string("noname_object"));
      
      //private_node_handle.param("gridmap_image_file", gridmap_image_file);
      gridmap_image_file=object_name+"_grid.png";
      
      // basically loads a lot of ROS parameters 
      mapAreas=loadAreas();
      //...........................................
      ROS_DEBUG("Configuration params:");
      
      ROS_DEBUG("global_frame: %s", global_frame.c_str());    
      ROS_DEBUG("robot_frame: %s", robot_frame.c_str());
      ROS_DEBUG("tagID: %s", tagID.c_str());
      ROS_DEBUG("grid_map_name: %s", grid_map_name.c_str());      
      ROS_DEBUG("load/save_route is [%s]",save_route.c_str());          
      if (loadGrids){
        ROS_DEBUG("Load Saved data [TRUE]");          
      }else{
        ROS_DEBUG("Load Saved data [FALSE]");          
      }
      
      // Setting up map. 
      
      
      
      //................................................................
      
      ros::Subscriber map_sub_ ;
      isMapLoaded=false;
            
      map_sub_ = n.subscribe("/map", 10,  &rfid_gridMap::mapCallback,this);
          
      ROS_DEBUG("Waiting for map service");      
      if (ros::service::waitForService("/static_map",500)&(!isMapLoaded) ){    
         // don't judge me
         while (!isMapLoaded){
          // connect to map server and get dimensions and resolution (meters)          
          // get map via RPC
          nav_msgs::GetMap::Request  req;
          nav_msgs::GetMap::Response resp;
          ROS_DEBUG("Requesting the map...");
          ros::service::call("/static_map", req, resp);
          mapCallback(resp.map);
          
          ros::Duration(1).sleep(); // sleep for a second
          ROS_DEBUG(".");         
        }; 
      } else{          
          ROS_DEBUG("Service not found. Data got from topic");         
      }
          

      //ROS_DEBUG("Setting gridmap");         

      // map resolution (m/cell)
      // we don't need same resolution than map...
      //resolution=mapDesc.resolution;    
      // map Size (m.)
      size_x=mapDesc.width*resolution;
      size_y=mapDesc.height*resolution;
      

      //2d position of the grid map in the grid map frame [m]. 
      // strange, if I set this, when represented it displaces map origin 
      //orig_x = mapDesc.origin.position.x;
      //orig_y = mapDesc.origin.position.y;
      
      
      
      //................................................................
      
      
      
      //these parameters are hardcoded... 
      layerName="type";
      rosEncoding="mono16";
      lowerValue=0.0;
      upperValue=1.0;



      map_= vector<string>({layerName});
      map_.setGeometry(Length(size_x, size_y), resolution, Position(orig_x, orig_y));
      map_.setFrameId(global_frame);      
      map_.clearAll();
      //ROS_DEBUG("Setting gridmap done");         

      Position p;
      p=map_.getPosition();
      //ROS_DEBUG("Map origin at (%3.3f,%3.3f)",p.x(),p.y());         

      if (loadGrids) {        
        //ROS_DEBUG("Loading previous data");         
        // file to image...
        //cv::Mat imageCV = cv::imread(gridmap_image_file, CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat imageCV = cv::imread((save_route+gridmap_image_file), CV_LOAD_IMAGE_UNCHANGED );
        //ROS_DEBUG("Converting to ROS format");         
        sensor_msgs::ImagePtr imageROS = cv_bridge::CvImage(std_msgs::Header(), rosEncoding, imageCV).toImageMsg();                      
        
        ROS_ASSERT_MSG(GridMapRosConverter::addLayerFromImage(*imageROS, layerName, map_, lowerValue, upperValue,0.5),
        "Mismatch loading layer image (%u w,%u h) Into map (%u r,%u c). Aborting",imageROS->width,imageROS->height,map_.getSize()(0),map_.getSize()(1));         
      } 
      
      
      lastRegion.name=std::string(" ");
      
      ROS_DEBUG("Advertising publishers");         
      gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(grid_map_name, 1, true);      
      publishMap();
      
      prob_pub_ = n.advertise<std_msgs::String>(prob_pub_name, 1000);
        
      ROS_DEBUG("Subscribing...");         
      // get tag readings
      ros::Subscriber sub_ = n.subscribe("/lastTag", 1000, &rfid_gridMap::tagCallback, this);
      
      // Update map periodically
      ros::Timer timer = n.createTimer(ros::Duration(0.5),  &rfid_gridMap::updateMapCallback,this);
      
      // publish updated probabilities every reasonable time.
      ros::Timer timer2 = n.createTimer(ros::Duration(1),  &rfid_gridMap::updateProbs,this);
      
      // publish updated probabilities every reasonable time.
      ros::Timer timer3 = n.createTimer(ros::Duration(saveTime),  &rfid_gridMap::saveMapCallback,this);
      
      ROS_DEBUG("Spin...");         
      ros::spin(); 
       
    }

    rfid_gridMap::~rfid_gridMap(){
        ros::TimerEvent ev;
        rfid_gridMap::saveMapCallback(ev);
    }
    
    
    void rfid_gridMap::tagCallback(const rfid_node::TagReading::ConstPtr& msg)
    {
      //Position p1;
      //p1=map_.getPosition();
      //ROS_DEBUG("Map origin at (%3.3f,%3.3f)",p1.x(),p1.y());         

       
       if (msg->ID.compare(tagID)==0){           
        //ROS_DEBUG("Asking for location");
        updateTransform();
        //ROS_DEBUG("Location updated");       
        //where to plot circle (m)
        double x=transform_.getOrigin().x();
        double y=transform_.getOrigin().y();
        if ((x!=0.0)&&(y!=0.0))
        {
            //ROS_INFO("I'm at %2.2f, %2.2f",x,y);            
            //ROS_DEBUG("got my tag! ");
            //ROS_INFO("got my tag! ");
            updateLastDetectionPose(x,y);

            //ROS_DEBUG("Decreasing map ");
            drawSquare(-size_x/2,-size_y/2,size_x/2,size_y/2,-weight_dec);
            //ROS_DEBUG("Increasing map ");
            drawCircle( x,  y,  detectRadius, weight_inc);        
        } else {
            //ROS_DEBUG("I'm at %2.2f, %2.2f",x,y);
        }
        
       }
       //ROS_DEBUG("Done");
    }
    
    void rfid_gridMap::saveMapCallback(const ros::TimerEvent&){
        
        
        sensor_msgs::Image image;
        cv_bridge::CvImagePtr cv_ptr;
                
        //ROS_INFO("Periodic gridmap storage in [%s]",(save_route+gridmap_image_file).c_str());        
        
        GridMapRosConverter::toImage(map_, layerName, rosEncoding, image);
        // image to file...
        try
        {
          cv_ptr = cv_bridge::toCvCopy(image, rosEncoding);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
        } 
        
        cv::imwrite( (save_route+gridmap_image_file), cv_ptr->image );
    }
    
    void rfid_gridMap::updateMapCallback(const ros::TimerEvent&)
    {
     
     // not decaying on time
     //drawSquare(-size_x/2,-size_y/2,size_x/2,size_y/2,-intensity_);
       
     publishMap();
     }
     
     
     std::vector<rfid_gridMap::type_area> rfid_gridMap::loadAreas(){
       
        
        std::vector<type_area> mapa;
        XmlRpc::XmlRpcValue regionsMap;

        nodeHandle_.getParam("/Regions/",regionsMap);        
        
        ROS_DEBUG("Loaded: %d regions",regionsMap.size());
        ROS_DEBUG("regionsMap Type %d",regionsMap.getType());

        ROS_ASSERT(regionsMap.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
        for(int m=0; m<regionsMap.size(); ++m) {
            string regionName =regionsMap[m]["name"];
            //ROS_DEBUG("Region: %s" , regionName.c_str() );
            
            type_area area;  
            area.name=regionName;
            area.prob=0.0;
                                 
            XmlRpc::XmlRpcValue points;            
            points=regionsMap[m]["points"];
            
            ROS_ASSERT(points.getType() == XmlRpc::XmlRpcValue::TypeArray);
            //ROS_DEBUG("#%d",points.size());
            
            for(int j=0; j<points.size(); ++j) {
                        double px,py;
                        //ROS_DEBUG("#%d",points[j].getType());                        
                                                
                        if ( j % 2 == 0 ){
                            px=points[j];
                        }else {                        
                            py=points[j];
                            Position p=Position(px,py);
                            area.polygon.addVertex(p);
                        }
                        //ROS_DEBUG("%3.3f,%3.3f",px,py);                        
            }
            mapa.push_back(area);
            
            if (regionsMap[m].hasMember("subregions")) {
                //ROS_DEBUG("Region %s has subregions!!" , regionName.c_str() );
                std::vector<rfid_gridMap::type_area> mapSubs;
                    
                for(int s=0; s<regionsMap[m]["subregions"].size(); ++s) {                    
                    string subRegionName =regionsMap[m]["subregions"][s]["name"];
                    //ROS_DEBUG("subregion: %s" , subRegionName.c_str() );

                    type_area area;  
                    area.name=subRegionName;
                    area.prob=0.0;
                    XmlRpc::XmlRpcValue points;

                    points=regionsMap[m]["subregions"][s]["points"];

                    ROS_ASSERT(points.getType() == XmlRpc::XmlRpcValue::TypeArray);
                    //ROS_DEBUG("#%d",points.size());
                    //ROS_DEBUG("#n");
                    for(int j=0; j<points.size(); ++j) {
                        //ROS_DEBUG("#%d",points[j].getType());
                        double px,py;
                                                
                        if ( j % 2 == 0 ){
                            px=points[j];
                        }else {                        
                            py=points[j];
                            Position p=Position(px,py);
                            area.polygon.addVertex(p);
                        }
                        //ROS_DEBUG("%3.3f,%3.3f",px,py);        
                    }
                    mapSubs.push_back(area);

                }               
                mapSubAreas[area.name]=mapSubs;                 
            }
            
        }
        return mapa;
      
      }
    
    void rfid_gridMap::updateProbs(const ros::TimerEvent&)
    {
		double total=0;

		double val=0;
		std::stringstream sstream;
		int i;
		
		total=0;
        
        double min_d=10000.0;
        double d=0.0;
		for (std::size_t i=0;i<mapAreas.size();i++) { 
             
             // use polygon method to check points inside
             wasHere(mapAreas[i]);
                          
             // use minimum distance to centroid
             d = (mapAreas[i].polygon.getCentroid() - lastP).norm();     
             if (d<=min_d) {
                 lastRegion=mapAreas[i];
                 min_d=d;
            }
             
             val=countValuesInArea(mapAreas[i].polygon);
             mapAreas[i].prob=val;
             total+=val;		 
              
//...................
             std::map<std::string,std::vector<rfid_gridMap::type_area>>::iterator it = mapSubAreas.find(mapAreas[i].name);
             if (it != mapSubAreas.end()){
                 double sub_area_total=0;
                 //std::vector<rfid_gridMap::type_area> subAreasVec=it->second;
                 //ROS_DEBUG("-------------------->   Region [%s] has [%lu] subregions",mapAreas[i].name.c_str(),it->second.size());
                 for (std::size_t k=0;k<it->second.size();k++) {                     
                     val=countValuesInArea(it->second[k].polygon);
                     it->second[k].prob=val;
                     sub_area_total+=val;	                     	                      
                     //ROS_DEBUG("SUB Region [%s] has [%3.3f] weight",it->second[k].name.c_str(),it->second[k].prob);
                 }                 
                 for (std::size_t k=0;k<it->second.size();k++) { 
                     if (sub_area_total>0.0)
                         it->second[k].prob=it->second[k].prob/sub_area_total;
                     else
                         it->second[k].prob=0;		                     
                     //ROS_DEBUG("SUB Region [%s] has rel prob. [%3.3f]",it->second[k].name.c_str(),it->second[k].prob);
                 }                                  
             }    
//...................
             
		}
          
        
        
       sstream<<lastRegion.name+",-1";
        
        for (std::size_t i=0;i<mapAreas.size();i++){
            
            if (total>0.0)
                mapAreas[i].prob=mapAreas[i].prob/total;
            else
                mapAreas[i].prob=0;			
            sstream<<","<<mapAreas[i].name<<","<<mapAreas[i].prob;			 
            
//...................
             std::map<std::string,std::vector<rfid_gridMap::type_area>>::iterator it = mapSubAreas.find(mapAreas[i].name);
             if (it != mapSubAreas.end()){
                 //std::vector<rfid_gridMap::type_area> subAreasVec=it->second;
                 //ROS_DEBUG("Region [%s] has Prob. [%3.3f]",mapAreas[i].name.c_str(),mapAreas[i].prob);
                 for (std::size_t k=0;k<it->second.size();k++) { 
                     if (it->second[k].prob>0.0)
                         it->second[k].prob=it->second[k].prob * mapAreas[i].prob;
                     //ROS_DEBUG("SUB Region [%s] has ABS prob. [%3.3f]",it->second[k].name.c_str(),it->second[k].prob);
                     sstream<<","<<it->second[k].name<<","<<it->second[k].prob;			 
                 }                                  
             }    
//...................            
            
        }		
        
        std_msgs::String msg;
        
        msg.data=sstream.str();
        prob_pub_.publish(msg);
        //ROS_DEBUG("------------------------------------>  %d",__LINE__);
    }
    
    void rfid_gridMap::updateTransform()
    {
        // TODO these are NOT frame ids 
        try{
            listener_.waitForTransform(global_frame, robot_frame, ros::Time(0), ros::Duration(0.5) );
            listener_.lookupTransform(global_frame, robot_frame, ros::Time(0), transform_);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }    
    }
    
    
    void rfid_gridMap::publishMap()
    {
      
      map_.setTimestamp(ros::Time::now().toNSec());
      
      grid_map_msgs::GridMap message;
      grid_map::GridMapRosConverter::toMessage(map_, message);
      gridMapPublisher_.publish(message);
      //ROS_DEBUG("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
    }


double rfid_gridMap::countValuesInArea(Polygon pol)
{ 
  double total=0.0;
  for (grid_map::PolygonIterator iterator(map_, pol); !iterator.isPastEnd(); ++iterator) {     
        if (!isnan(map_.at(layerName, *iterator)))
        {
            total+=map_.at(layerName, *iterator);
        }
  }

  return total;
}



void rfid_gridMap::updateLastDetectionPose(double x, double y){
	lastP=Position(x,y);
}


void rfid_gridMap::wasHere(type_area area)
{            
    //ROS_DEBUG("Point (%3.3f,%3.3f)", lastP.x(),lastP.y());
    if (area.polygon.isInside(lastP)){
		lastRegion=area;
        //ROS_DEBUG("We are in area %s", area.name.c_str());
    } else {
        //ROS_DEBUG("We are NOT in area %s", area.name.c_str());
    }
        
}

// we get information from our global map
void rfid_gridMap::mapCallback(const nav_msgs::OccupancyGrid& msg)
    {
       if (!isMapLoaded)
        if ((msg.info.width>0.0)&&(msg.info.height>0.0)){
        isMapLoaded=true;
        mapDesc=msg.info;
        mapFrame=msg.header.frame_id;
        ROS_DEBUG("Received a %d X %d map @ %.3f m/pix  Origin X %.3f Y %.3f\n",
                msg.info.width,
                msg.info.height,
                msg.info.resolution,
                msg.info.origin.position.x,
                msg.info.origin.position.y);
        } else {           
            ROS_DEBUG("Received an INVALID!! %d X %d map @ %.3f m/pix  Origin X %.3f Y %.3f\n",
                msg.info.width,
                msg.info.height,
                msg.info.resolution,
                msg.info.origin.position.x,
                msg.info.origin.position.y);
        }
        
}

void rfid_gridMap::drawPolygon(const grid_map::Polygon poly,double value)
{      
  for (grid_map::PolygonIterator iterator(map_, poly); !iterator.isPastEnd(); ++iterator) {     
        map_.at(layerName, *iterator) =  value +map_.at(layerName, *iterator);              
        if (isnan(map_.at(layerName, *iterator)))
        {
            map_.at(layerName, *iterator) =  value;
        }
    if (map_.at(layerName, *iterator)<lowerValue)
        map_.at(layerName, *iterator) =  lowerValue;
    if (map_.at(layerName, *iterator)>upperValue)
        map_.at(layerName, *iterator) = upperValue;       
  }
}


void rfid_gridMap::drawSquare(double start_x,double start_y,double end_x,double end_y,double value)
{
    grid_map::Polygon poly;
    Position p;
    
    p=Position(start_x,start_y);  
    poly.addVertex(p);
    p=Position(end_x,start_y);  
    poly.addVertex(p);
    p=Position(end_x,end_y);  
    poly.addVertex(p);
    p=Position(start_x,end_y);  
    poly.addVertex(p);
    
    for (grid_map::PolygonIterator iterator(map_, poly); !iterator.isPastEnd(); ++iterator) {
        map_.at(layerName, *iterator) =  value + map_.at(layerName, *iterator);              
        if (isnan(map_.at(layerName, *iterator)))
        {
            map_.at(layerName, *iterator) =  value;
        }
        if (map_.at(layerName, *iterator)<lowerValue)
            map_.at(layerName, *iterator) =  lowerValue;
        if (map_.at(layerName, *iterator)>upperValue)
            map_.at(layerName, *iterator) = upperValue;

    }
}



// based on demoCircleIterator
void rfid_gridMap::drawCircle(double x, double y, double radius, double value)
{
  //ROS_INFO("Plotting circle at (%2.2f,%2.2f)",x,y);
  
  Position center(x, y);
  
  for (grid_map::CircleIterator iterator(map_, center, radius);
      !iterator.isPastEnd(); ++iterator) {
    map_.at(layerName, *iterator) =  value +map_.at(layerName, *iterator);
    
    if (isnan(map_.at(layerName, *iterator)))
    {
        map_.at(layerName, *iterator) =  value;
    }    
    if (map_.at(layerName, *iterator)<lowerValue)
        map_.at(layerName, *iterator) =  lowerValue;
    if (map_.at(layerName, *iterator)>upperValue)
        map_.at(layerName, *iterator) = upperValue;
  }
}
    
    
} // end of namespace rfid_grid_map






/*
void rfid_gridMap::drawSquare0(double start_x,double start_y,double end_x,double end_y,double value)
{
  Index submapStartIndex;
  Index submapEndIndex;
  Index submapBufferSize;
  
  Index lowerCorner(0,0);
  Index upperCorner(0,0);
    
  Position position;
  
  Position submapStartPosition(end_x, end_y);
  Position submapEndPosition(start_x, start_y);  
  
  Size mapSize=map_.getSize();
  ROS_DEBUG("Map is (%d w %d h)",mapSize(0),mapSize(1));
  upperCorner(0)=  mapSize(0)-1;
  upperCorner(1)=  mapSize(1)-1;  
  
  if (!map_.isInside(submapStartPosition)){
      ROS_DEBUG("Start Position is out of map");
      map_.getPosition( lowerCorner, position ) ;
        
      //rounding
      if (submapStartPosition(0)<position(0))
            submapStartPosition(0)=position(0);
      if (submapStartPosition(1)<position(1))
            submapStartPosition(1)=position(1);
      
      ROS_DEBUG("Accessing  from position (%2.2f,%2.2f) ",submapStartPosition(0),submapStartPosition(1) );    
  }
  if (!map_.isInside(submapEndPosition)){
      ROS_DEBUG("End Position is out of map");
       map_.getPosition( upperCorner, position ) ;
        
      //rounding
      if (submapEndPosition(0)<position(0))
            submapEndPosition(0)=position(0);
      if (submapEndPosition(1)<position(1))
            submapEndPosition(1)=position(1);
      
      ROS_DEBUG("Accessing up to position (%2.2f,%2.2f)",submapEndPosition(0),submapEndPosition(1) );    
  }
  
  map_.getIndex(submapStartPosition,submapStartIndex);
  map_.getIndex(submapEndPosition,submapEndIndex);
 ROS_DEBUG("Accessing  from index (%d,%d) to (%d,%d)",submapStartIndex(0),submapStartIndex(1),submapEndIndex(0),submapEndIndex(1) );    
      
  submapBufferSize=abs(submapEndIndex-submapStartIndex);
  
 ROS_DEBUG("Accessing (%d width x %d high) indexes",submapBufferSize(0),submapBufferSize(1));    
  for (grid_map::SubmapIterator iterator(map_, submapStartIndex, submapBufferSize);
      !iterator.isPastEnd(); ++iterator) {     
        
        map_.at(layerName, *iterator) =  value +map_.at(layerName, *iterator);              
        if (isnan(map_.at(layerName, *iterator)))
        {
            map_.at(layerName, *iterator) =  value;
        }
        if (map_.at(layerName, *iterator)<lowerValue)
            map_.at(layerName, *iterator) =  lowerValue;
        if (map_.at(layerName, *iterator)>upperValue)
            map_.at(layerName, *iterator) = upperValue;
  }

}
*/
