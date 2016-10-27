    
/**
 * ToDo: parametrize decay time, prob publish time, intensity
 * 
 * 
 * */


#include "rfid_grid_map/rfid_gridMap.hpp"


namespace rfid_grid_map {
    rfid_gridMap::rfid_gridMap(ros::NodeHandle& n)
    : nodeHandle_(n),
      map_(vector<string>({"type"}))
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
      
      //2d position of the grid map in the grid map frame [m].  
      double orig_x=0;
      double orig_y=0;      
      // LOAD ROS PARAMETERS ....................................
      
      private_node_handle.param("map_size_x", size_x, 10.0);
      private_node_handle.param("map_size_y", size_y, 10.0);
      
      private_node_handle.param("orig_x", orig_x, 0.0);
      private_node_handle.param("orig_y", orig_y, 0.0);
      
      private_node_handle.param("map_resolution", resolution, 0.1);
      private_node_handle.param("global_frame", global_frame, std::string("odom_combined"));    
      private_node_handle.param("robot_frame", robot_frame, std::string("base_link"));
      private_node_handle.param("tagID", tagID, std::string("390000010000000000000007"));
      private_node_handle.param("grid_map_name", grid_map_name, std::string("grid_map"));
      private_node_handle.param("prob_pub_name", prob_pub_name, std::string("probs"));
      
      //ROS_ASSERT(nodeHandle_.getParam("regions_file", regions_file));          
      
      // basically loads a lot of ROS parameters 
      mapAreas=loadAreas();
      //...........................................
      ROS_DEBUG("Configuration params:");
      
      ROS_DEBUG("map_size_x: %.2f", size_x);
      ROS_DEBUG("map_size_y: %.2f", size_y);      
      ROS_DEBUG("orig_x: %.2f", orig_x);
      ROS_DEBUG("orig_y: %.2f", orig_y);      
      ROS_DEBUG("map_resolution: %.2f", resolution);
      ROS_DEBUG("global_frame: %s", global_frame.c_str());    
      ROS_DEBUG("robot_frame: %s", robot_frame.c_str());
      ROS_DEBUG("tagID: %s", tagID.c_str());
      ROS_DEBUG("grid_map_name: %s", grid_map_name.c_str());      
      ROS_DEBUG("regions_file: %s", regions_file.c_str());          
      
      
      intensity_=0.001;
  
      // Setting up map. 
      map_.setGeometry(Length(size_x, size_y), resolution, Position(orig_x, orig_y));
      map_.setFrameId(global_frame);
      map_.clearAll();
      lastRegion.name=std::string(" ");
      
      gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(grid_map_name, 1, true);      
      publishMap();
      
      prob_pub_ = n.advertise<std_msgs::String>(prob_pub_name, 1000);
        
      // get tag readings
      ros::Subscriber sub_ = n.subscribe("/lastTag", 1000, &rfid_gridMap::tagCallback, this);
      
      // Update map periodically
      ros::Timer timer = n.createTimer(ros::Duration(0.5),  &rfid_gridMap::updateMapCallback,this);
      
      // publish updated probabilities every reasonable time.
      ros::Timer timer2 = n.createTimer(ros::Duration(1),  &rfid_gridMap::updateProbs,this);
      
      ros::spin(); 
       
    }

    rfid_gridMap::~rfid_gridMap(){}
    
    
    void rfid_gridMap::tagCallback(const rfid_node::TagReading::ConstPtr& msg)
    {
       
       if (msg->ID.compare(tagID)==0){
        //ROS_INFO("Asking for location");
        updateTransform();
        //ROS_INFO("Location updated");       
        //where to plot circle (m)
        double x=transform_.getOrigin().x();
        double y=transform_.getOrigin().y();
        //ROS_INFO("I'm at %2.2f, %2.2f",x,y);
		
        //how big we want it (m)
        double radius=2;            
        //ROS_INFO("got my tag! ");
        //ROS_INFO("got my tag! ");
        updateLastDetectionPose(x,y);
        drawSquare(-size_x/2,-size_y/2,size_x/2,size_y/2,-intensity_);
        drawCircle( x,  y,  radius, 5*intensity_);        
       }
       
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
                    mapa.push_back(area);

                }               
                
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
		for (std::size_t i=0;i<mapAreas.size();i++) { 
             wasHere(mapAreas[i]);                
             val=countValuesInArea(mapAreas[i].polygon);
             mapAreas[i].prob=val;
             total+=val;		 
		}
        
        sstream<<lastRegion.name+",-1";
        
        for (std::size_t i=0;i<mapAreas.size();i++){
            
            if (total>0.0)
                mapAreas[i].prob=mapAreas[i].prob/total;
            else
                mapAreas[i].prob=0;			
            sstream<<","<<mapAreas[i].name<<","<<mapAreas[i].prob;			 
        }		
        
        std_msgs::String msg;
        
        msg.data=sstream.str();
        prob_pub_.publish(msg);
         
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
        if (!isnan(map_.at("type", *iterator)))
        {
            total+=map_.at("type", *iterator);
        }
  }

  return total;
}



void rfid_gridMap::updateLastDetectionPose(double x, double y){
	lastP=Position(x,y);
}


void rfid_gridMap::wasHere(type_area area)
{            
    if (area.polygon.isInside(lastP)){
		lastRegion=area;
    }		
}

void rfid_gridMap::drawPolygon(const grid_map::Polygon poly,double value)
{      
  for (grid_map::PolygonIterator iterator(map_, poly); !iterator.isPastEnd(); ++iterator) {     
        map_.at("type", *iterator) =  value +map_.at("type", *iterator);              
        if (isnan(map_.at("type", *iterator)))
        {
            map_.at("type", *iterator) =  value;
        }
        if (map_.at("type", *iterator)<0.0)
            map_.at("type", *iterator) =  0.0;        
  }
}

void rfid_gridMap::drawSquare(double start_x,double start_y,double end_x,double end_y,double value)
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
  //ROS_INFO("Map is (%d w %d h)",mapSize(0),mapSize(1));
  upperCorner(0)=  mapSize(0)-1;
  upperCorner(1)=  mapSize(1)-1;  
  
  if (!map_.isInside(submapStartPosition)){
      //ROS_INFO("Start Position is out of map");
      map_.getPosition( lowerCorner, position ) ;
        
      //rounding
      if (submapStartPosition(0)<position(0))
            submapStartPosition(0)=position(0);
      if (submapStartPosition(1)<position(1))
            submapStartPosition(1)=position(1);
      
      //ROS_INFO("Accessing  from position (%2.2f,%2.2f) ",submapStartPosition(0),submapStartPosition(1) );    
  }
  if (!map_.isInside(submapEndPosition)){
      //ROS_INFO("End Position is out of map");
       map_.getPosition( upperCorner, position ) ;
        
      //rounding
      if (submapEndPosition(0)<position(0))
            submapEndPosition(0)=position(0);
      if (submapEndPosition(1)<position(1))
            submapEndPosition(1)=position(1);
      
      //ROS_INFO("Accessing up to position (%2.2f,%2.2f)",submapEndPosition(0),submapEndPosition(1) );    
  }
  
  map_.getIndex(submapStartPosition,submapStartIndex);
  map_.getIndex(submapEndPosition,submapEndIndex);
 //ROS_INFO("Accessing  from index (%d,%d) to (%d,%d)",submapStartIndex(0),submapStartIndex(1),submapEndIndex(0),submapEndIndex(1) );    
      
  submapBufferSize=abs(submapEndIndex-submapStartIndex);
  
 // ROS_INFO("Accessing (%d width x %d high) indexes",submapBufferSize(0),submapBufferSize(1));    
  for (grid_map::SubmapIterator iterator(map_, submapStartIndex, submapBufferSize);
      !iterator.isPastEnd(); ++iterator) {     
        
        map_.at("type", *iterator) =  value +map_.at("type", *iterator);              
        if (isnan(map_.at("type", *iterator)))
        {
            map_.at("type", *iterator) =  value;
        }
        if (map_.at("type", *iterator)<0.0)
            map_.at("type", *iterator) =  0.0;
  }

}


// based on demoCircleIterator
void rfid_gridMap::drawCircle(double x, double y, double radius, double value)
{
  //ROS_INFO("Plotting circle at (%2.2f,%2.2f)",x,y);
  
  Position center(x, y);
  
  for (grid_map::CircleIterator iterator(map_, center, radius);
      !iterator.isPastEnd(); ++iterator) {
    map_.at("type", *iterator) =  value +map_.at("type", *iterator);
    
    if (isnan(map_.at("type", *iterator)))
    {
        map_.at("type", *iterator) =  value;
    }    
    if (map_.at("type", *iterator)<0.0)
        map_.at("type", *iterator) =  0.0;
    
  }
}
    
    
} // end of namespace rfid_grid_map
