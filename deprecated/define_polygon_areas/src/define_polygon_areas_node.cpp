    
/*
 */

#include "define_polygon_areas/define_polygon_areas.hpp"
#include <ros/console.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "define_polygon_areas_node");
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
  {
   ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle nd("~");
  define_polygon_areas::define_polygon_areas rg(nd);

  ros::requestShutdown();
  return 0;
}

