#include "RadarModelROS.hpp"

using namespace std;
using namespace grid_map;

RadarModelROS::RadarModelROS(const nav_msgs::OccupancyGrid& nav_map, const double sigma_power, const double sigma_phase ){

        _sigma_power = sigma_power;
        _sigma_phase = sigma_phase;
        
        // NO TAG COORDS HERE!
        //_tags_coords ;
        _numTags = 1;

        initRefMap(nav_map);

        // build spline to interpolate antenna gains;
        std::vector<double> xVec(ANTENNA_ANGLES_LIST, ANTENNA_ANGLES_LIST + 25);
        Eigen::VectorXd xvals = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(xVec.data(), xVec.size());
        std::vector<double> yVec(ANTENNA_LOSSES_LIST, ANTENNA_LOSSES_LIST + 25);
        Eigen::VectorXd yvals= Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(yVec.data(), yVec.size());        
        _antenna_gains= SplineFunction(xvals, yvals);
      
        // rfid beliefs global map: One layer per tag
        std::string layerName;
        layerName = getTagLayerName(0);
        _rfid_belief_maps.add(layerName, 0.5);  // the cells need to have a uniform distribution at the beginning

        clearObstacleCellsRFIDMap();
        normalizeRFIDMap();
        debugInfo();

        // mesh grid layers
        _rfid_belief_maps.add("X", 0);  
        _rfid_belief_maps.add("Y", 0);  
        Position point;
        Index ind;
        for (grid_map::GridMapIterator iterator(_rfid_belief_maps); !iterator.isPastEnd(); ++iterator) {
            // matrix indexes...
            ind = *iterator;
            // get cell center of the cell in the map frame.            
            _rfid_belief_maps.getPosition(ind, point);
            // that is where the tag supposedly is in map coordinates
            _rfid_belief_maps.at("X",*iterator) = point.x();
            _rfid_belief_maps.at("Y",*iterator) = point.y();
        }

}



void RadarModelROS::initRefMap(const nav_msgs::OccupancyGrid& nav_map){
        std::cout<<"\nIniting Ref map from occGrid."  <<std::endl;

        _resolution = nav_map.info.resolution;

        //FIXME! col == height? and row == width??
        _Ncol = nav_map.info.height; // radar model total x-range space (cells).
        _Nrow = nav_map.info.width; // radar model total y-range space (cells).
        
        // Convert to grid map.
        //FIXME! will be alligned ???
        GridMapRosConverter::fromOccupancyGrid(nav_map, "ref_map", _rfid_belief_maps);
        _free_space_val = _rfid_belief_maps.get("ref_map").maxCoeffOfFinites();


        std::cout << " Input map has " <<   _rfid_belief_maps.getSize()(1) << " cols by " <<  _rfid_belief_maps.getSize()(0) <<" rows "  <<std::endl;
        std::cout << " Orig at: (" << _rfid_belief_maps.getPosition().x() << ", " << _rfid_belief_maps.getPosition().y() <<") m. " <<std::endl;
        std::cout << " Size: (" << _rfid_belief_maps.getLength().x() << ", " << _rfid_belief_maps.getLength().y() <<") m. " <<std::endl;
        std::cout << " Values range: (" << _rfid_belief_maps.get("ref_map").minCoeffOfFinites() << ", " << _free_space_val <<")   " <<std::endl;
        std::cout << " Using : (" << _free_space_val <<") value as free space value  " <<std::endl;

        grid_map::Position p;       
        grid_map::Index index;

        std::cout<<"\nTesting boundaries: " <<std::endl;
        index =grid_map::Index(0,0);
        std::cout<<"P: Index("  << 0 << ", " << 0 << ") " <<std::endl;
        if (_rfid_belief_maps.getPosition(index,p)){  
            std::cout<<"P: Cell("  << index(0) << ", " << index(1) << ") is at (" << p(0) << ", " << p(1)<<") m. " <<std::endl;
        } else {
          std::cout<<" Cell("  << index(0) << ", " << index(1) << ") is out bounds!" <<std::endl;  
        }

        index =grid_map::Index(_Nrow-1,0);
        std::cout<<"P: Index("  << (_Nrow-1) << ", " << 0 << ") " <<std::endl;
        if (_rfid_belief_maps.getPosition(index,p)){  
            std::cout<<"P: Cell("  << index(0) << ", " << index(1) << ") is at (" << p(0) << ", " << p(1)<<") m. " <<std::endl;
        } else {
          std::cout<<" Cell("  << index(0) << ", " << index(1) << ") is out bounds!" <<std::endl;  
        }

        index =grid_map::Index(0,_Ncol-1);
        std::cout<<"P: Index("  << (0) << ", " << (_Ncol-1) << ") " <<std::endl;
        if (_rfid_belief_maps.getPosition(index,p)){  
            std::cout<<"P: Cell("  << index(0) << ", " << index(1) << ") is at (" << p(0) << ", " << p(1)<<") m. " <<std::endl;
        } else {
          std::cout<<" Cell("  << index(0) << ", " << index(1) << ") is out bounds!" <<std::endl;  
        }

        index =grid_map::Index(_Nrow-1,_Ncol-1);
        std::cout << "P: Index("  << (_Nrow-1) << ", " << (_Ncol-1) << ") " <<std::endl;        
        if (_rfid_belief_maps.getPosition(index,p)){  
            std::cout<<"P: Cell("  << index(0) << ", " << index(1) << ") is at (" << p(0) << ", " << p(1)<<") m. " <<std::endl;
        } else {
          std::cout<<" Cell("  << index(0) << ", " << index(1) << ") is out bounds!" <<std::endl;  
        }
        std::cout << "............................. " << std::endl << std::endl;        
}


void RadarModelROS::loadBelief(const std::string imageURI) {
  std::cout << "\nLoading belief map." << std::endl;

  cv::Mat _imageCV = cv::imread(imageURI, CV_LOAD_IMAGE_UNCHANGED);

  // cell value ranges
  double minValue;
  double maxValue;

  std::string layerName;
  layerName = getTagLayerName(0);

  cv::minMaxLoc(_imageCV, &minValue, &maxValue);  
  GridMapCvConverter::addLayerFromImage<unsigned char, 3>(_imageCV, layerName, _rfid_belief_maps, minValue, maxValue);

}  

void RadarModelROS::saveBelief(const std::string imageURI) {
  std::string layerName;
  layerName = getTagLayerName(0);
  getImage(layerName, imageURI);
}  