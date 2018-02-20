    
/**
 * 
 * 
 * 
 * */


#include "define_polygon_areas/define_polygon_areas.hpp"


namespace define_polygon_areas {

        
    define_polygon_areas::define_polygon_areas(ros::NodeHandle& n)
    : nodeHandle_(n),
      map_(vector<string>({"type"}))
    {
        
      ROS_DEBUG("HI WORLD");
          
      areaCounter=0;
      
      ros::Subscriber map_sub_ ;
      isMapLoaded=false;
            
      map_sub_ = n.subscribe("/map", 10,  &define_polygon_areas::mapCallback,this);
          
      ROS_DEBUG("Waiting for map service");      
      if (ros::service::waitForService("/static_map",500)){    
          // connect to map server and get dimensions and resolution (meters)          
          // get map via RPC
          nav_msgs::GetMap::Request  req;
          nav_msgs::GetMap::Response resp;
          ROS_DEBUG("Requesting the map...");
          ros::service::call("static_map", req, resp);
          mapCallback(resp.map);
      } else{          
          ROS_DEBUG("Service not found. Data got from topic");         
      }
            
     // don't judge me
     while (!isMapLoaded){
         ros::Duration(0.5).sleep(); // sleep for half a second
         ROS_DEBUG(".");         
         }; 
      
      
      // map resolution (m/cell)
      double resolution=mapDesc.resolution;    
      // map Size (m.)
      double size_y=2*mapDesc.height*resolution;
      double size_x=2*mapDesc.width*resolution;

      //2d position of the grid map in the grid map frame [m]. 
      // we consider them alligned 
      double orig_x=0;
      double orig_y=0;
      orig_x = mapDesc.origin.position.x;
      orig_y = mapDesc.origin.position.y;
    
  
      // Setting up map. 
      map_.setGeometry(Length(size_x, size_y), resolution, Position(orig_x, orig_y));
      map_.setFrameId("map");
      map_.clearAll();
      
      gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);      
      publishMap();
      ROS_DEBUG("First grid published");
      vis_pub = nodeHandle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  
      //subscribe to clicks
      numberOfClicks=-1;
      
      plg =grid_map::Polygon();
      plg.setFrameId(mapFrame);
      
      ros::Subscriber sub_ = n.subscribe("/clicked_point", 1000, &define_polygon_areas::clickCallback, this);




      /*if (false){
        std::string regions_file;
      
        ROS_ASSERT(nodeHandle_.getParam("regions_file", regions_file));          
        //nodeHandle_.param("regions_file", regions_file,std::string("/home/mfernandezcarmona/catkin_ws/src/ENRICHME/codes/ais/LibMercuryRFID/src/clients/ros/rfid_grid_map/config/LCAS.yaml"));    
        ROS_DEBUG("Loading config from %s",regions_file.c_str());

        // got to find a better way to do this...
        std::list<type_area> areas;      
        areas=loadAreas(regions_file);
        int i=0;             
      
        for (std::list<type_area>::iterator area=areas.begin(); area != areas.end(); ++area)   
        {          
          ROS_DEBUG("Publishin region %s",area->name.c_str());
          drawSquare(area->startX,area->startY,area->endX,area->endY,0.5);
          plotMarker(  ( area->endX - area->startX ) / 2 ,( area->endY - area->startY) / 2, area->name );
          publishMap();
          ros::Duration d(2);
          d.sleep();
        }      
        ROS_DEBUG("All regions published");
      }*/
      std::list<type_area> areas;      
      areas=loadAreas("/home/mfernandezcarmona/catkin_ws/second.yaml");
      int i=0;             

      for (std::list<type_area>::iterator area=areas.begin(); area != areas.end(); ++area)   
      {          
          ROS_DEBUG("Publishin region %s",area->name.c_str());
          drawPolygon(area->polygon,0.5);          
          publishMap();
          ros::Duration d(2);
          d.sleep();
      }      
      ROS_DEBUG("All regions published");
      
      ROS_DEBUG("Start clicking");
      ros::spin(); 
       
    }

    void define_polygon_areas::timerCallback(const ros::TimerEvent&)
    {
     //ROS_INFO("UPDATE MAP NEEDED!");
     //drawSquare(-5,-5,5,5,-intensity_);

     //publishMap();
     }
     
     
     
     std::list<define_polygon_areas::type_area> define_polygon_areas::loadAreas(std::string regions_file){
       
        
        std::list<type_area> mapa;
        
        //////////////////////////////////////////////////////////
        XmlRpc::XmlRpcValue regionsMap;

        nodeHandle_.getParam("/Regions/",regionsMap);        
        
        ROS_DEBUG("Loaded: %d regions",regionsMap.size());
        ROS_DEBUG("regionsMap Type %d",regionsMap.getType());

        ROS_ASSERT(regionsMap.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
        for(int m=0; m<regionsMap.size(); ++m) {
            string regionName =regionsMap[m]["name"];
            ROS_DEBUG("Region: %s" , regionName.c_str() );
            
            type_area area;  
            area.name=regionName;
                                 
            XmlRpc::XmlRpcValue points;            
            points=regionsMap[m]["points"];
            
            ROS_ASSERT(points.getType() == XmlRpc::XmlRpcValue::TypeArray);
            //ROS_DEBUG("#%d",points.size());
            
            for(int j=0; j<points.size(); ++j) {
                        double px=0;
                        double py=0;
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
            
            Position ct=area.polygon.getCentroid();
            ROS_DEBUG("centroid: %3.3f,%3.3f" , ct.x(),ct.y() );
            
            mapa.push_back(area);
            
            if (regionsMap[m].hasMember("subregions")) {
                ROS_DEBUG("Region %s has subregions!!" , regionName.c_str() );
                    
                    //..........................
                for(int s=0; s<regionsMap[m]["subregions"].size(); ++s) {                    
                    string subRegionName =regionsMap[m]["subregions"][s]["name"];
                    ROS_DEBUG("subregion: %s" , subRegionName.c_str() );

                    type_area area;  
                    area.name=subRegionName;

                    XmlRpc::XmlRpcValue points;

                    points=regionsMap[m]["subregions"][s]["points"];

                    ROS_ASSERT(points.getType() == XmlRpc::XmlRpcValue::TypeArray);
                    //ROS_DEBUG("subregion has %d points",points.size());                    
                    for(int j=0; j<points.size(); ++j) {
                        //ROS_DEBUG("Point type %d",points[j].getType());
                        ROS_ASSERT(points[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                        double px=0;
                        double py=0;            
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
				    Position sct=area.polygon.getCentroid();
                    ROS_DEBUG("centroid: %3.3f,%3.3f" , sct.x(),sct.y() );
                }
                    //...........................
                
                
            }
            
        }
        
        
        /* int i=0;
        
        for (std::size_t i=0;i<config["Regions"].size();i++) {            
            type_area area;            
            
            ROS_DEBUG("-Region %s\n",area.name.c_str());
            ROS_DEBUG("  - name: %s\n    min:\n      x: %.2f\n      y: %.2f\n    max:\n      x: %.2f\n      y: %.2f\n\n",area.name.c_str(),area.startX,area.startY,area.endX,area.endY);    
            
            if (config["Regions"][i]["subregions"]){
                int j;
                for (std::size_t j=0;j<config["Regions"][i]["subregions"].size();j++) {
                    type_area area;
                    area.name=config["Regions"][i]["subregions"][j]["name"].as<std::string>();;
                    area.startX=config["Regions"][i]["subregions"][j]["min"]["x"].as<double>();;
                    area.startY=config["Regions"][i]["subregions"][j]["min"]["y"].as<double>();;							
                    area.endX=config["Regions"][i]["subregions"][j]["max"]["x"].as<double>();;
                    area.endY=config["Regions"][i]["subregions"][j]["max"]["y"].as<double>();;
                    mapa.push_back(area);
                    
                    ROS_DEBUG("-Subregion %s\n",area.name.c_str());
                    ROS_DEBUG("min:\nx: %.2f\ny: %.2f\n max:\nx: %.2f\ny: %.2f\n\n",area.startX,area.startY,area.endX,area.endY);    
                }	 
            }
            ROS_DEBUG("...................................................................");
        }
        ROS_DEBUG("///////////////////////////////////////////////////////////////////////");
        */
        return mapa;
      
      }
      
      

      void define_polygon_areas::yamlSave(){
          //file is stored in .ros folder
          YAML::Node config;
        // write regions yaml file      
        ROS_DEBUG("...................................................................");
        ROS_DEBUG("Saving regions:");
        for (unsigned i=0; i < definedAreas.size(); i++) {        
            type_area area;            
            area=definedAreas[i];
            vector<Position> positions=area.polygon.getVertices();
            ROS_DEBUG("Region %s:",area.name.c_str());
            
            for (unsigned j=0; j < positions.size(); j++) {        
                YAML::Node point;
                Position p=positions[j];
                point.push_back(p.x());
                point.push_back(p.y());
                config["Regions"][i].push_back(point);
                
                ROS_DEBUG("(%3.3f,%3.3f)",p.x(),p.y());
            }
            
        }
        
        std::ofstream fout("file.yaml");
        fout << config;
        ROS_DEBUG("///////////////////////////////////////////////////////////////////////");
      }



     
    define_polygon_areas::~define_polygon_areas(){
        
        
    }
    
    void define_polygon_areas::plotMarker(double x, double  y, string text )
    {
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.text=text;
        marker.pose.position.x = 1;
        marker.pose.position.y = 1;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime= ros::Duration(0);
        vis_pub.publish( marker );

    }
    
    // we get information from our global map
    void define_polygon_areas::mapCallback(const nav_msgs::OccupancyGrid& msg)
    {
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
    
    /* we will start drawing squares using clicks*/
    void define_polygon_areas::clickCallback(const geometry_msgs::PointStamped::ConstPtr msg)
    {
        double dist=100000.0;
        double px =msg->point.x;
        double py=msg->point.y;
                
        ROS_DEBUG("Clicked on %.2f, %.2f\n",msg->point.x,msg->point.y);
        
        if (plg.nVertices()>0){
            Position startPoint=plg.getVertex(0);
            dist= sqrt(  pow(startPoint.x()-px,2)+
                       pow(startPoint.y()-py,2));
        }
        
        if (dist>0.5)
        {                      
            Position p=Position(px,py);
            ROS_DEBUG("Adding point... \n");                                
            plg.addVertex(p);
            ROS_DEBUG("Point added... \n");                     
        } 
        else 
        {
            ROS_DEBUG("Poligon Closed... \n");          
            drawPolygon(plg,1);
            publishMap();    
            
            //save for later
            type_area newArea;
            newArea.name=to_string(areaCounter++);
            newArea.polygon=plg;            
            definedAreas.push_back(newArea);
                  
            //publish as rosparams
            //////////////////////////////////////////////////////////
            
            // Set and get a map of strings                                                          
            vector<Position> positions=newArea.polygon.getVertices();
            std::vector<double> flatPointList;
            
            ROS_DEBUG("Region %s:",newArea.name.c_str());
            
            for (unsigned j=0; j < positions.size(); j++) {                        
                Position p=positions[j];
                                
                flatPointList.push_back(roundf(p.x() * 100) / 100);
                flatPointList.push_back(roundf(p.y() * 100) / 100);
                                
                ROS_DEBUG("(%3.2f,%3.2f)",p.x(),p.y());
            }
            nodeHandle_.setParam(std::string("Regions/")+newArea.name, flatPointList);

            //////////////////////////////////////////////////////////
            
            
            // get ready for next one
            plg =grid_map::Polygon();
            plg.setFrameId(mapFrame);
        }
            
        
    }
    
    
    void define_polygon_areas::publishMap()
    {
      
      map_.setTimestamp(ros::Time::now().toNSec());
      
      grid_map_msgs::GridMap message;
      grid_map::GridMapRosConverter::toMessage(map_, message);
      gridMapPublisher_.publish(message);
      ROS_DEBUG("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
    }


void define_polygon_areas::drawSquare(double start_x,double start_y,double end_x,double end_y,double value)
{
  //ROS_DEBUG("Drawing an square \n min:\nx: %.2f\ny: %.2f\n max:\nx: %.2f\ny: %.2f\n\n",start_x,start_y,end_x,end_y);
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


void define_polygon_areas::drawPolygon(const grid_map::Polygon poly,double value)
{
  //ROS_INFO("Accessing position");
  for (grid_map::PolygonIterator iterator(map_, poly); !iterator.isPastEnd(); ++iterator) {
      
        //Position currentPosition;
        //map_.getPosition(*iterator, currentPosition);
        //ROS_INFO("(%3.3f,%3.3f)",currentPosition(0),currentPosition(1) );    


        map_.at("type", *iterator) =  value +map_.at("type", *iterator);              
        if (isnan(map_.at("type", *iterator)))
        {
            map_.at("type", *iterator) =  value;
        }
        if (map_.at("type", *iterator)<0.0)
            map_.at("type", *iterator) =  0.0;
        
  }

}

 
 



    
} // end of namespace define_polygon_areas

