#include "RadarModelROS.hpp"
#include <unsupported/Eigen/SpecialFunctions>


// http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt

//////////////////  SPLINE FUNCTIONS   //////////////////

// The spline is used to interpolate antenna gain values, as we only have the
// graphs
SplineFunction::SplineFunction() {}

SplineFunction::SplineFunction(Eigen::VectorXd const &x_vec,
                               Eigen::VectorXd const &y_vec)
    : x_min(x_vec.minCoeff()), x_max(x_vec.maxCoeff()), y_min(y_vec.minCoeff()),
      y_max(y_vec.maxCoeff()),
      // Spline fitting here. X values are scaled down to [0, 1] for this.
      spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(
          y_vec.transpose(), std::min<int>(x_vec.rows() - 1, 6),
          scaled_values(
              x_vec))) // No more than cubic spline, but accept short vectors.
{}

// x values need to be scaled down in extraction as well.
double SplineFunction::interpDeg(double x) const {
  double y;
  y = spline_(scaled_value(x))(0);

  // interpolation may produce values bigger and lower than our limits ...
  y = max(min(y, y_max), y_min);
  return y;
}

double SplineFunction::interpRad(double x) const {
  return interpDeg(x * 180.0 / M_PI);
}

float SplineFunction::interpRadf(float x) const {
    return (float) interpDeg(( (double) x) *180.0/M_PI); 
}

// Helpers to scale X values down to [0, 1]
double SplineFunction::scaled_value(double x) const {
  return (x - x_min) / (x_max - x_min);
}

Eigen::RowVectorXd
SplineFunction::scaled_values(Eigen::VectorXd const &x_vec) const {
  return x_vec.unaryExpr([this](double x) { return scaled_value(x); })
      .transpose();
}

//////////////////

RadarModelROS::RadarModelROS(){};


RadarModelROS::RadarModelROS(const nav_msgs::OccupancyGrid& nav_map, const double sigma_power, const double sigma_phase, const double resolution ){

        _sigma_power = sigma_power;
        _sigma_phase = sigma_phase;
        _resolution = resolution;
        


        initRefMap(nav_map);

        // build spline to interpolate antenna gains;
        std::vector<double> xVec(ANTENNA_ANGLES_LIST, ANTENNA_ANGLES_LIST + 25);
        Eigen::VectorXd xvals = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(xVec.data(), xVec.size());
        std::vector<double> yVec(ANTENNA_LOSSES_LIST, ANTENNA_LOSSES_LIST + 25);
        Eigen::VectorXd yvals= Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(yVec.data(), yVec.size());        
        _antenna_gains= SplineFunction(xvals, yvals);

        // faster approach: build a cache
        _antenaGainVector.reserve(6284); // [-pi,pi] in steps of 0.001 rads        
        float gain_i, rad_i;
        for(int milirad_index=0; milirad_index<6284; milirad_index++) {
          rad_i =  (((float) milirad_index)/1000.0) - M_PI; // from range [0,6283] to [-pi,pi]
          gain_i = _antenna_gains.interpRadf( rad_i );
          _antenaGainVector.push_back(gain_i);
        }

        // no tags yet.   
        _numTags = 0;

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

void RadarModelROS::addTagLayer(int tagNum){
    std::string layerName;
    layerName = getTagLayerName(tagNum);
    _rfid_belief_maps.add(layerName, 0.5);  // the cells need to have a uniform distribution at the beginning

    clearObstacleCellsRFIDLayer(layerName);
    normalizeRFIDLayer(layerName);
    _numTags = _numTags+1;
}

void RadarModelROS::initRefMap(const nav_msgs::OccupancyGrid& nav_map){
  ROS_DEBUG_STREAM("Initing Ref map from occGrid.");
  GridMap tempGrid;
  grid_map::Matrix tempMat;
  GridMap resizedMap;
  grid_map::Position pos;       
      
  // find occupied space index boundaries          
  Index ind, ind2, submapTopLeftIndex, submapBufferSize;
  unsigned int minr, minc, maxr, maxc;
  double minx, miny, maxx, maxy,maxV, countOnes, countZeros;
  float occ_space, free_space;
        
  // we crop input occupancy grid depending on obstacles
  minr = nav_map.info.height; 
  minc = nav_map.info.width;
  maxr = 0;
  maxc = 0;
  GridMapRosConverter::fromOccupancyGrid(nav_map, "ref_map", tempGrid);
  tempMat = tempGrid.get("ref_map");          
  // clean the input occupancy grid:
  // remove NANs          
  tempMat = (tempMat.array().isFinite()).select(tempMat,0.0f); 
  //Binarize and Invert: 0s are obstacles now and 1 free space
  occ_space = 0.0f;
  free_space = 1.0f;
  tempMat = (tempMat.array()<tempMat.maxCoeff()).select(free_space, grid_map::Matrix::Constant(tempMat.rows(),tempMat.cols(),occ_space)); 
  tempGrid["ref_map"] = tempMat;
  
  // find boundaries defined by obstacles
  for (GridMapIterator iterator(tempGrid); !iterator.isPastEnd(); ++iterator) {
    ind = *iterator;
    if (tempGrid.at("ref_map", ind) == occ_space){
        if (ind(0)<minr){
          minr = ind(0);
        } else if (ind(0)>maxr) {
          maxr = ind(0);
        }
        if (ind(1)<minc){
          minc = ind(1);
        } else if (ind(1)>maxc) {
          maxc = ind(1);
        }
    }
  }

  tempGrid.getPosition(Index(minr,minc),pos);
  maxx = pos.x();
  maxy = pos.y();
  tempGrid.getPosition(Index(maxr,maxc),pos);
  minx = pos.x();
  miny = pos.y();

  ROS_DEBUG_STREAM( " ..............................................................."  );
  ROS_DEBUG_STREAM( " Obstacles are between indexes (" << minr << ", " << minc << ") and (" << maxr << ", " << maxc << ")"  );
  ROS_DEBUG_STREAM( "                        points (" << minx << ", " << miny << ") and (" << maxx << ", " << maxy << ")"  );
  ROS_DEBUG_STREAM( "                        Centre (" << (maxx+minx)/2.0 << ", " << (maxy+miny)/2.0 << ")"  );
  ROS_DEBUG_STREAM( " ..............................................................."  );

  // resize to those boundaries ...
  bool wasOk;
  resizedMap = tempGrid.getSubmap(Position(maxx+minx,maxy+miny)/2.0, Length(maxx-minx, maxy-miny), wasOk);

  // change resolution
  GridMapCvProcessing::changeResolution(resizedMap, resizedMap, _resolution);

  _rfid_belief_maps = resizedMap;
  //_rfid_belief_maps = tempGrid;
  //_resolution = nav_map.info.resolution;
  _Ncol = _rfid_belief_maps.get("ref_map").cols();
  _Nrow = _rfid_belief_maps.get("ref_map").rows();
  _free_space_val =  _rfid_belief_maps.get("ref_map").maxCoeffOfFinites(); // should be 1

  countOnes = _rfid_belief_maps.get("ref_map").sum();
  countZeros = _rfid_belief_maps.get("ref_map").rows() * _rfid_belief_maps.get("ref_map").cols() - countOnes;

  ROS_DEBUG_STREAM(100*countOnes/(countOnes+countZeros) << " % of cells are empty");
  ROS_DEBUG_STREAM(100*countZeros/(countOnes+countZeros) << " % of cells are obstacles ");
  ROS_DEBUG_STREAM( " Using : (" << _free_space_val <<") value as free space value  " );

  debugInfo(&tempGrid, "Loaded occ grid","ref_map");
  debugInfo(&resizedMap, "Resized ref map", "ref_map");

  // ////////////////////////////////////
  // // Convert to image.

  // cv::Mat image;
  // GridMapCvConverter::toImage<unsigned char, 3>(_rfid_belief_maps, "ref_map", CV_8UC3, _rfid_belief_maps.get("ref_map").minCoeffOfFinites(), _rfid_belief_maps.get("ref_map").maxCoeffOfFinites(), image);  
  // //overlayRobotPose(_rfid_belief_maps.getPosition().x(), _rfid_belief_maps.getPosition().y(), 0, image);
  // //overlayRobotPose(resizedMap.getPosition().x(), resizedMap.getPosition().y(), 0, image);
  // overlayRobotPose(minx, miny, 0, image);
  // overlayRobotPose(maxx, miny, 0, image);
  // overlayRobotPose(maxx, maxy, 0, image);
  // overlayRobotPose(minx, maxy, 0, image);          
  // //overlayRobotPose(0, 0, 0, image);

  // overlayMapEdges(image);

  // cv::flip(image, image, -1);
  // cv::imwrite( "/tmp/orig_map.png", image );
  // ROS_DEBUG_STREAM(" MAP SAVED! At line: "<<__LINE__);


  // GridMapCvConverter::toImage<unsigned char, 3>(resizedMap, "ref_map", CV_8UC3, resizedMap.get("ref_map").minCoeffOfFinites(), resizedMap.get("ref_map").maxCoeffOfFinites(), image);  
  // //overlayRobotPose(&resizedMap, resizedMap.getPosition().x(), resizedMap.getPosition().y(), 0, image);
  // //overlayRobotPose(&resizedMap, _rfid_belief_maps.getPosition().x(), _rfid_belief_maps.getPosition().y(), 0, image);
  // overlayRobotPose(&resizedMap,minx, miny, 0, image);
  // overlayRobotPose(&resizedMap,maxx, miny, 0, image);
  // overlayRobotPose(&resizedMap,maxx, maxy, 0, image);
  // overlayRobotPose(&resizedMap,minx, maxy, 0, image);          
  // //overlayRobotPose(&resizedMap,0, 0, 0, image);

  // cv::flip(image, image, -1);
  // cv::imwrite( "/tmp/red_map.png", image );

  // ////////////////////////////////////


}

void RadarModelROS::debugInfo(){
  debugInfo(&_rfid_belief_maps, "RFID reference map","ref_map");
}

void RadarModelROS::debugInfo(GridMap *gm, std::string mapName, std::string baseLayer ){


        grid_map::Position p;       
        grid_map::Index index;
        int nrow, ncol;

        nrow = (*gm).getSize().x();
        ncol = (*gm).getSize().y();

        ROS_DEBUG_STREAM( " xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
        ROS_DEBUG_STREAM( " Testing map: [" << mapName << "]");
        ROS_DEBUG_STREAM( " Map has " << nrow << " rows by " << ncol << " cols (i,j axis)");
        ROS_DEBUG_STREAM( " Size: (" << (*gm).getLength().x() << ", " << (*gm).getLength().y() <<") m. " );
        ROS_DEBUG_STREAM( " Orig at: (" << (*gm).getPosition().x() << ", " << (*gm).getPosition().y() <<") m., @frame_id: ["+ (*gm).getFrameId()+"]" );
        ROS_DEBUG_STREAM( " Values range: (" << (*gm).get(baseLayer).minCoeffOfFinites() << ", " << (*gm).get(baseLayer).maxCoeffOfFinites() <<")   " );
        ROS_DEBUG_STREAM( " Resolution is: " << (*gm).getResolution() << " m. /cell" );

        ROS_DEBUG_STREAM(" " );
        ROS_DEBUG_STREAM("Testing boundaries: " );
        index =grid_map::Index(0,0);

        ROS_DEBUG_STREAM("P: Index("  << 0 << ", " << 0 << ") " );
        if ((*gm).getPosition(index,p)){  
            ROS_DEBUG_STREAM("P: Cell("  << index(0) << ", " << index(1) << ") is at (" << p(0) << ", " << p(1)<<") m. " );
        } else {
          ROS_DEBUG_STREAM(" Cell("  << index(0) << ", " << index(1) << ") is out bounds!" );  
        }

        index =grid_map::Index(nrow-1,0);
        ROS_DEBUG_STREAM("P: Index("  << (nrow-1) << ", " << 0 << ") " );
        if ((*gm).getPosition(index,p)){  
            ROS_DEBUG_STREAM("P: Cell("  << index(0) << ", " << index(1) << ") is at (" << p(0) << ", " << p(1)<<") m. " );
        } else {
          ROS_DEBUG_STREAM(" Cell("  << index(0) << ", " << index(1) << ") is out bounds!" );  
        }

        index =grid_map::Index(0,ncol-1);
        ROS_DEBUG_STREAM("P: Index("  << (0) << ", " << (ncol-1) << ") " );
        if ((*gm).getPosition(index,p)){  
            ROS_DEBUG_STREAM("P: Cell("  << index(0) << ", " << index(1) << ") is at (" << p(0) << ", " << p(1)<<") m. " );
        } else {
          ROS_DEBUG_STREAM(" Cell("  << index(0) << ", " << index(1) << ") is out bounds!" );  
        }

        index =grid_map::Index(nrow-1,ncol-1);
        ROS_DEBUG_STREAM( "P: Index("  << (nrow-1) << ", " << (ncol-1) << ") " );        
        if ((*gm).getPosition(index,p)){  
            ROS_DEBUG_STREAM("P: Cell("  << index(0) << ", " << index(1) << ") is at (" << p(0) << ", " << p(1)<<") m. " );
        } else {
          ROS_DEBUG_STREAM(" Cell("  << index(0) << ", " << index(1) << ") is out bounds!" );  
        }
        ROS_DEBUG_STREAM( " xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
}

//////////////////////////  GETTERS
///////////////////////////////////////////////////////
Eigen::MatrixXf  RadarModelROS::getPowProbCond(double rxPw,  double f_i, double txtPower){
  return getPowProbCond( rxPw,  0,  0,  0,  f_i, txtPower);
}

Eigen::MatrixXf  RadarModelROS::getPowProbCond(double rxPw, double x_m, double y_m, double orientation_deg, double f_i, double txtPower){
  Eigen::MatrixXf PW_mat = getFriisMat(x_m,y_m,orientation_deg, f_i, txtPower);
  Eigen::MatrixXf ans = getProbCond(PW_mat, rxPw, _sigma_power);
  return ans;
}


Eigen::MatrixXf  RadarModelROS::getPhaseProbCond(double rxPw,  double f_i){
  return getPhaseProbCond( rxPw,  0,  0,  0,  f_i);
}

Eigen::MatrixXf  RadarModelROS::getPhaseProbCond(double ph_i, double x_m, double y_m, double orientation_deg, double f_i){
    Eigen::MatrixXf PH_mat = getPhaseMat(x_m,y_m,orientation_deg, f_i);
    Eigen::MatrixXf ans = getProbCond(PH_mat, ph_i, _sigma_phase);
    
    return ans;
} 

Eigen::MatrixXf  RadarModelROS::getProbCond(Eigen::MatrixXf X_mat, double x, double sig){
      
  Eigen::MatrixXf likl_mat;
  
  // gaussian pdf
  // fddp(x,mu,sigma) = exp( -0.5 * ( (x - mu)/sigma )^2 )   /   (sigma * sqrt(2 pi )) 
  likl_mat = ( x - X_mat.array() ) /_sigma_power;
  likl_mat = -0.5 * ( likl_mat.array().pow(2.0) );
  likl_mat = likl_mat.array().exp() / ( _sigma_power * sqrt( 2.0 * M_PI ) ) ;

  return likl_mat;
}

Eigen::MatrixXf RadarModelROS::getFriisMat(double x_m, double y_m, double orientation_deg, double freq, double txtPower){
  if (useFast)
    return getFriisMatFast(x_m, y_m, orientation_deg, freq, txtPower);
  else
    return getFriisMatSlow(x_m, y_m, orientation_deg, freq, txtPower);
}


Eigen::MatrixXf RadarModelROS::getFriisMatSlow(double x_m, double y_m, double orientation_deg, double freq, double txtPower){
  Eigen::MatrixXf rxPw_mat;
  double tag_x, tag_y, rxP;
  Position glob_point;
  Index ind;

  Size siz = _rfid_belief_maps.getSize();
  rxPw_mat = Eigen::MatrixXf(siz(0), siz(1));
  double orientation_rad = orientation_deg * M_PI / 180.0;

  // Obtain rel dist and friis to all points
  // MFC: can I turn this iteration into matrix operations?
  for (grid_map::GridMapIterator iterator(_rfid_belief_maps);
       !iterator.isPastEnd(); ++iterator) {
    // matrix indexes...
    ind = *iterator;
    // get cell center of the cell in the map frame.
    _rfid_belief_maps.getPosition(ind, glob_point);
    // that is where the tag supposedly is in map coordinates
    tag_x = glob_point.x();
    tag_y = glob_point.y();

    rxP = received_power_friis_with_obstacles(x_m, y_m, orientation_rad, tag_x,
                                              tag_y, 0, freq, txtPower);
    rxPw_mat(ind(0), ind(1)) = rxP;
  }
  return rxPw_mat;
}

float RadarModelROS::gainValue(int x){
  return (float) _antenaGainVector.at(x);

}

Eigen::MatrixXf RadarModelROS::getFriisMatFast(double x_m, double y_m, double orientation_deg, double freq, double txtPower){

  Eigen::MatrixXf X, Y, R, A, propL, antL,totalLoss, rxPower;
  Eigen::VectorXf x,y;
  Index i00,i0M,iNM,iN0, iRobot;
  double lambda =  C/freq;
  double orientation_rad = orientation_deg * M_PI/180.0;
  
  ros::Time begin, end;
  begin = ros::Time::now();
  //1. rotate and translate
  Eigen::MatrixXf X0 = _rfid_belief_maps["X"];
  Eigen::MatrixXf Y0 = _rfid_belief_maps["Y"];
  
  double cA =cos(orientation_rad);
  double sA =sin(orientation_rad);

  X =   ( X0 * cA + Y0 * sA).array() - (x_m*cA + y_m*sA);
  Y =   (-X0 * sA + Y0 * cA).array() + (x_m*sA - y_m*cA);  

  //2. create R,Ang matrixes
  R = (X.array().square() + Y.array().square()).array().sqrt();
  A = Y.binaryExpr(X, std::ptr_fun(atan2f)).array();

  //3. Create a friis losses propagation matrix without taking obstacles        
  // auto funtor = std::bind(&SplineFunction::interpRadf, _antenna_gains, std::placeholders::_1) ;
  //antL =  TAG_LOSSES + A.unaryExpr( funtor ).array();   

  //2.1 cast matrix A to contain rad indexes
  A = (A.array() + M_PI) * 1000.0; 
  Eigen::MatrixXi Ai = A.cast<int>();
    
  auto funtor = std::bind(&RadarModelROS::gainValue, this, std::placeholders::_1) ;

  antL =  TAG_LOSSES + Ai.unaryExpr( funtor ).array();   

  propL = LOSS_CONSTANT - (20.0 * (R * freq).array().log10()).array() ;

  //4. final rxPower: signal goes from antenna to tag and comes back again, so we double the losses
  totalLoss =  2.0*antL + 2.0*propL ;
  
  rxPower = totalLoss.array() + txtPower;
  // this should remove points where friis is not applicable
  rxPower = (R.array()>2.0*lambda).select(rxPower,txtPower); 
  
  //5. Obstacle losses: Create a NaN filled matrix 
  _rfid_belief_maps.add("obst_losses",NAN);

  // iterate over four lines to fill obst_losses layer ...................  
  // line 1: (0,0) to (0,M)
  i00 =grid_map::Index(0,0);
  i0M =grid_map::Index(0,_Ncol-1);
  _rfid_belief_maps.getIndex(Position( x_m, y_m), iRobot);
  addLossesTillEdgeLine(i00, i0M, iRobot );

  // line 2: (0,M) to (N,M)
  iNM =grid_map::Index(_Nrow-1,_Ncol-1);
  addLossesTillEdgeLine(i0M, iNM, iRobot );

  // line 3: (N,M) to (N,0)
  iN0 =grid_map::Index(_Nrow-1,0);
  addLossesTillEdgeLine(iNM, iN0, iRobot );

  // line 4: (N,0) to (0,0)
  addLossesTillEdgeLine(iN0, i00, iRobot );

  // And finally add obstacle losses and propagation losses  
  rxPower = rxPower   - _rfid_belief_maps.get("obst_losses");
  //ROS_DEBUG_STREAM( "Still running at line: " << __LINE__);

  // this should remove points where received power is too low
  rxPower = (rxPower.array()>SENSITIVITY).select(rxPower,SENSITIVITY); 

  // mfc trick used to see temp matrixes as images
  //_rfid_belief_maps.add("obst_losses",rxPower);
  _rfid_belief_maps.erase("obst_losses");
  return rxPower;

}

void RadarModelROS::addLossesTillEdgeLine(Index edge_index_start,   Index edge_index_end,   Index antenna_index){
  Index edge_index;
  
  double obst_loss_ray, obst_cell_inc;

  // Each "wall" adds around 3dB losses. A wall is ~15cm thick, then each cell adds  (3 * resolution / 0.15) dB losses
  obst_cell_inc = 20.0 * _resolution; // db 

  // move along the map edge defined by those two indexes
  for (grid_map::LineIterator edge_iterator(_rfid_belief_maps, edge_index_start, edge_index_end); !edge_iterator.isPastEnd(); ++edge_iterator) {
    edge_index = *edge_iterator; 
    //  - initializate cummulated losses to 0.
    obst_loss_ray = 0;

    //Now iterate from xm,ym to the point xi,yi in the edge
    for (grid_map::LineIterator loss_ray_iterator(_rfid_belief_maps, antenna_index, edge_index); !loss_ray_iterator.isPastEnd(); ++loss_ray_iterator) {

      // if the cell is obstacle, add L to cummulated_L
      if (( _rfid_belief_maps.at("ref_map", *loss_ray_iterator) != _free_space_val  )){
        obst_loss_ray += obst_cell_inc;
      }

      // obstacles losses in cell is cummulated_L. Avoid multiple edits   
      if (( _rfid_belief_maps.at("obst_losses", *loss_ray_iterator) != NAN  )){
        _rfid_belief_maps.at("obst_losses", *loss_ray_iterator) = obst_loss_ray; 
      }      

    }
  }

}

Eigen::MatrixXf RadarModelROS::getPhaseMat(double x_m, double y_m, double orientation_deg, double freq){
  
  Eigen::MatrixXf rxPh_mat;
  double tag_x, tag_y, tag_r, tag_h, rxPh;
  Position glob_point;
  Index ind;

  Size siz = _rfid_belief_maps.getSize();
  rxPh_mat = Eigen::MatrixXf(siz(0), siz(1));
  double orientation_rad = orientation_deg * M_PI / 180.0;

  // Obtain rel dist and to all points
  // MFC: can I turn this iteration into matrix operations?
  for (grid_map::GridMapIterator iterator(_rfid_belief_maps);
       !iterator.isPastEnd(); ++iterator) {
    // matrix indexes...
    ind = *iterator;
    // get cell center of the cell in the map frame.
    _rfid_belief_maps.getPosition(ind, glob_point);
    // that is where the tag supposedly is in map coordinates
    tag_x = glob_point.x();
    tag_y = glob_point.y();

    // now get robot - tag relative pose!
    double delta_x = (tag_x - x_m);
    double delta_y = (tag_y - y_m);
    // rotate
    tag_x = delta_x * cos(orientation_rad) + delta_y * sin(orientation_rad);
    tag_y = -delta_x * sin(orientation_rad) + delta_y * cos(orientation_rad);

    getSphericCoords(tag_x, tag_y, tag_r, tag_h);

    rxPh = phaseDifference(tag_r, tag_h, freq);

    rxPh_mat(ind(0), ind(1)) = rxPh;
  }
  return rxPh_mat;
}

void RadarModelROS::getImage(std::string layerName, std::string fileURI){
    //ROS_DEBUG_STREAM( "Still running at line: " << __LINE__);

    getImage(&_rfid_belief_maps, layerName, fileURI);
}

void RadarModelROS::getImage(GridMap* gm,std::string layerName, std::string fileURI){
  // Convert to image.
  cv::Mat image = layerToImage(gm, layerName);
  cv::imwrite( fileURI, image );
}

double RadarModelROS::getTotalWeight(int tag_i) {
  return getTotalWeight((_Ncol - 1) * (_resolution / 2.0),
                        (_Nrow - 1) * (_resolution / 2.0), 0,
                        _Ncol * _resolution, _Nrow * _resolution, tag_i);
}

double RadarModelROS::getTotalWeight(double x, double y, double orientation,
                                  double size_x, double size_y, int tag_i) {
  // TODO: I'm not using the orientation. Maybe it would be better to use a
  // polygon iterator,
  //     so we can rotate edges around the center and have a more flexible thing

  // submapStartIndex the start index of the submap, typically top-left index.
  grid_map::Index submapStartIndex, submapEndIndex, submapBufferSize;
  grid_map::Position submapStartPosition(x + (size_x / 2), y + (size_y / 2));
  grid_map::Position submapEndPosition(x - (size_x / 2), y - (size_y / 2));

  if (!_rfid_belief_maps.getIndex(submapStartPosition, submapStartIndex)) {
    submapStartIndex = grid_map::Index(0, 0);
    // ROS_DEBUG_STREAM("Clip start!" );
  }

  if (!_rfid_belief_maps.getIndex(submapEndPosition, submapEndIndex)) {
    Size siz = _rfid_belief_maps.getSize();
    submapEndIndex = grid_map::Index(siz(0) - 1, siz(1) - 1);
    // ROS_DEBUG_STREAM("Clip end!" );
  }

  submapBufferSize = submapEndIndex - submapStartIndex;

  grid_map::SubmapIterator iterator(_rfid_belief_maps, submapStartIndex,
                                    submapBufferSize);

  // ROS_DEBUG_STREAM("\nGet prob.:" );
  // ROS_DEBUG_STREAM(" Centered at Position (" << x << ", " << y << ") m. / Size ("
  //                    << size_x << ", " << size_y << ")" ); ROS_DEBUG_STREAM(" Start pose ("
  //                    << submapStartPosition(0) << ", " << submapStartPosition(1) << ") m. to pose " << submapEndPosition(0) << ", " << submapEndPosition(1) << ") m."); 
  // ROS_DEBUG_STREAM(" Start Cell ("  << submapStartIndex(0) << ", " <<
  //                    submapStartIndex(1) << ") to cell("  << submapEndIndex(0) << ", " <<
  //                    submapEndIndex(1) << ")" );

  return getTotalWeight(iterator, tag_i);
}

double RadarModelROS::getTotalWeight(grid_map::SubmapIterator iterator,
                                  int tag_i) {

  double total_weight;
  Position point;

  std::string tagLayerName = getTagLayerName(tag_i);

  total_weight = 0;
  for (iterator; !iterator.isPastEnd(); ++iterator) {
    _rfid_belief_maps.getPosition(*iterator, point);
    // check if is inside global map
    if (_rfid_belief_maps.isInside(point)) {
      // We don't add belief from positions considered obstacles...
      if (_rfid_belief_maps.atPosition("ref_map", point) == _free_space_val) {
        total_weight += _rfid_belief_maps.atPosition(tagLayerName, point);
      }
    }
  }
  return total_weight;
}

double RadarModelROS::getTotalWeight(grid_map::PolygonIterator iterator,
                                  int tag_i) {

  double total_weight;
  Position point;

  std::string tagLayerName = getTagLayerName(tag_i);

  total_weight = 0;
  for (iterator; !iterator.isPastEnd(); ++iterator) {
    _rfid_belief_maps.getPosition(*iterator, point);
    // check if is inside global map
    if (_rfid_belief_maps.isInside(point)) {
      // We don't add belief from positions considered obstacles...
      if (_rfid_belief_maps.atPosition("ref_map", point) == _free_space_val) {
        total_weight += _rfid_belief_maps.atPosition(tagLayerName, point);
      }
    }
  }
  return total_weight;
}

void RadarModelROS::getImageDebug(GridMap *gm, std::string layerName,
                               std::string fileURI) {
  // plots circles in the edges of the corresponding layer

  // Convert to image.
  cv::Mat image = layerToImage(gm, layerName);
  
  cv::Scalar green( 0, 255, 0 );
  cv::Scalar blue( 255, 0, 0 );      
  cv::Scalar red( 0, 0, 255 );
  cv::Scalar yellow( 0, 255, 255 );
  
  grid_map::Index index;

  double maxX = (*gm).getLength().x() / 2;
  double maxY = (*gm).getLength().y() / 2;

  grid_map::Position p(maxX, maxY);
  (*gm).getIndex(p, index);
  cv::Point gree(index.x(), index.x());
  ROS_DEBUG_STREAM( "Green: (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" );

  p = Position(maxX, -maxY);
  (*gm).getIndex(p, index);
  cv::Point blu(index.y(), index.x());
  ROS_DEBUG_STREAM( "Blue (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" );

  p = Position(-maxX, -maxY);
  (*gm).getIndex(p, index);
  cv::Point re(index.y(), index.x());
  ROS_DEBUG_STREAM( "Red (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" );

  p = Position(-maxX, maxY);
  (*gm).getIndex(p, index);
  cv::Point yell(index.y(), index.x());
  ROS_DEBUG_STREAM( "Yellow (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" );

  cv::circle(image, gree, 20, green, -1);
  cv::circle(image, blu, 20, blue, -1);
  cv::circle(image, re, 20, red, -1);
  cv::circle(image, yell, 20, yellow, -1);

  cv::Point triang_points[1][3];
  double h = 0.2;

  p = Position(h / 2.0, 0);
  (*gm).getIndex(p, index);
  triang_points[0][0] = cv::Point(index.y(), index.x());
  ROS_DEBUG_STREAM( "p (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" );

  p = Position(-h / 2.0, -h / 2.0);
  (*gm).getIndex(p, index);
  triang_points[0][1] = cv::Point(index.y(), index.x());
  ROS_DEBUG_STREAM( "p (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" );

  p = Position(-h / 2.0, h / 2.0);
  (*gm).getIndex(p, index);
  triang_points[0][2] = cv::Point(index.y(), index.x());
  ROS_DEBUG_STREAM( "p (" << p(0) << ", " << p(1) << ") m. == Cell(" << index(0)
            << ", " << index(1) << ")" );

  const cv::Point *ppt[1] = {triang_points[0]};
  int npt[] = {3};
  cv::fillPoly(image, ppt, npt, 1, red, 8);

  // Rotate 90 Degrees Clockwise To get our images to have increasing X to right
  // and increasing Y up
  // cv::transpose(image, image);
  // cv::flip(image, image, 1);
  cv::flip(image, image, -1);

  cv::imwrite(fileURI, image);
}

std::string RadarModelROS::getPowLayerName(double freq_i) {
  return "P_" + getLayerName(freq_i / 1e6);
}

std::string RadarModelROS::getPhaseLayerName(double freq_i) {
  return "D_" + getLayerName(freq_i / 1e6);
}

std::string RadarModelROS::getTagLayerName(int tag_num) {
  return std::to_string(tag_num);
}

std::string RadarModelROS::getLayerName(double x) {

  // Create an output string stream
  std::ostringstream streamObj3;

  // Set Fixed -Point Notation
  streamObj3 << std::fixed;

  // Set precision to 2 digits
  streamObj3 << std::setprecision(2);

  // Add double to stream
  streamObj3 << x;

  // Get string from output string stream
  std::string strObj3 = streamObj3.str();

  return strObj3;
}

cv::Point RadarModelROS::getPoint(double x_m, double y_m) {
  return getPoint(&_rfid_belief_maps,x_m, y_m);
}



cv::Point RadarModelROS::getPoint(  GridMap* gm, double x_m, double y_m) {

  Length mlen = (*gm).getLength();
  double resol = (*gm).getResolution();
  int nrow = (*gm).getSize().x();
  int ncol = (*gm).getSize().y();
  grid_map::Position orig = (*gm).getPosition();

  double min_x = orig(0) - mlen(0) / 2 + resol;
  double max_x = orig(0) + mlen(0) / 2 - resol;
  double min_y = orig(1) - mlen(1) / 2 + resol;
  double max_y = orig(1) + mlen(1) / 2 - resol;

  grid_map::Index index;

  grid_map::Position p(x_m, y_m);
  if (!(*gm).getIndex(p, index)) {
    x_m = std::min(std::max(x_m, min_x), max_x);
    y_m = std::min(std::max(y_m, min_y), max_y);
    p = grid_map::Position(x_m, y_m);
    (*gm).getIndex(p, index);
  }

  // cast from gridmap indexes to opencv indexes
  int cv_y = (nrow - 1) - index.x();
  int cv_x = (ncol - 1) - index.y();

  return cv::Point(cv_x, cv_y);
  //return cv::Point(index.y(), index.x());
}

void RadarModelROS::getSphericCoords(double x, double y, double &r, double &phi) {
  r = sqrt(x * x + y * y);
  phi = atan2(y, x);
}

//////////////////////////  Print and visualization
///////////////////////////////////////////////////////


void RadarModelROS::PrintRecPower(std::string fileURI, double f_i, double txtPower){
  PrintRecPower(fileURI, 0,0,0,  f_i, txtPower);
}

void RadarModelROS::PrintRecPower(std::string fileURI,double x_m, double y_m, double orientation_deg, double f_i, double txtPower){
  // create a copy of the average values
  //ROS_DEBUG_STREAM( "Still running at line: " << __LINE__);
  Eigen::MatrixXf av_mat = getFriisMat(x_m, y_m, orientation_deg, f_i, txtPower);

  //ROS_DEBUG_STREAM( "Still running at line: " << __LINE__);
  PrintProb(fileURI, &av_mat);
  //ROS_DEBUG_STREAM( "Still running at line: " << __LINE__);
}

void RadarModelROS::PrintPhase(std::string fileURI,  double f_i){            
  PrintPhase(fileURI, 0,0,0, f_i);    
}

void RadarModelROS::PrintPhase(std::string fileURI,double x_m, double y_m, double orientation_deg,  double f_i){        
  // create a copy of the average values
  Eigen::MatrixXf av_mat = getPhaseMat(x_m, y_m, orientation_deg, f_i);
  PrintProb(fileURI, &av_mat);
}

void RadarModelROS::PrintPowProb(std::string fileURI, double rxPw, double x_m, double y_m, double orientation_deg, double f_i, double txtPower){
  Eigen::MatrixXf prob_mat = getPowProbCond(rxPw, x_m, y_m, orientation_deg, f_i, txtPower);
  PrintProb(fileURI, &prob_mat);
}

void RadarModelROS::PrintPowProb(std::string fileURI, double rxPw, double f_i, double txtPower){
  PrintPowProb(fileURI, rxPw, 0,0,0, f_i, txtPower);
}

void RadarModelROS::PrintPhaseProb(std::string fileURI, double phi, double f_i){
  PrintPhaseProb(fileURI, phi, 0, 0, 0, f_i);
}

void RadarModelROS::PrintPhaseProb(std::string fileURI, double phi, double x_m, double y_m, double orientation_deg, double f_i){
  Eigen::MatrixXf prob_mat = getPhaseProbCond(phi, x_m, y_m, orientation_deg, f_i);
  PrintProb(fileURI, &prob_mat);
}

void RadarModelROS::PrintBothProb(std::string fileURI, double rxPw, double phi, double f_i, double txtPower){
  PrintBothProb(fileURI, rxPw, phi, 0,0,0, f_i, txtPower);
}

void RadarModelROS::PrintBothProb(std::string fileURI, double rxPw, double phi, double x_m, double y_m, double orientation_deg, double f_i, double txtPower){
  Eigen::MatrixXf prob_mat = getPowProbCond(rxPw, x_m, y_m, orientation_deg, f_i, txtPower).cwiseProduct(getPhaseProbCond(phi, x_m, y_m, orientation_deg, f_i));
  prob_mat = prob_mat/prob_mat.sum();
  PrintProb(fileURI, &prob_mat);
}

void RadarModelROS::PrintProb(std::string fileURI, Eigen::MatrixXf *prob_mat) {
  PrintProb(fileURI, prob_mat, prob_mat->rows(), prob_mat->cols(), _resolution);
}

void RadarModelROS::PrintProb(std::string fileURI, Eigen::MatrixXf* prob_mat, double sX, double sY, double res){
  GridMap tempMap;      
  tempMap.setGeometry(Length(sX, sY), res);
  tempMap.add("res", *prob_mat);
  getImage(&tempMap, "res", fileURI);  
}


void RadarModelROS::overlayMapEdges(cv::Mat image) {
  cv::Point square_points[1][4];
  cv::Scalar yellow(0, 255, 255);
  int cv_y, cv_x;

  cv_y = (_Nrow - 1);
  cv_x = (_Ncol - 1);

  square_points[0][0] = cv::Point(0, 0);
  square_points[0][1] = cv::Point(0, cv_y);
  square_points[0][2] = cv::Point(cv_x, cv_y);
  square_points[0][3] = cv::Point(cv_x, 0);

  const cv::Point *pts[1] = {square_points[0]};
  int npts[] = {4};
  cv::polylines(image, pts, npts, 1, true, yellow);
}


void RadarModelROS::overlayRobotPoseT(double robot_x, double robot_y,
                                   double robot_head, cv::Mat &image) {
  grid_map::Index index;
  cv::Point center;
  grid_map::Position p;
  cv::Point pentag_points[1][5];

  cv::Scalar red(0, 0, 255);
  center = getPoint(robot_x, robot_y);

  // create a pentagone pointing x+

  int h =  std::ceil(0.2/(_resolution ));; // pixels?

  pentag_points[0][0] = cv::Point(center.x - h, center.y - h);
  pentag_points[0][1] = cv::Point(center.x - h, center.y + h);
  pentag_points[0][2] = cv::Point(center.x, center.y + 2 * h);
  pentag_points[0][3] = cv::Point(center.x + h, center.y + h);
  pentag_points[0][4] = cv::Point(center.x + h, center.y - h);
  const cv::Point *pts[1] = {pentag_points[0]};
  rotatePoints(pentag_points[0], 5, center.x, center.y, robot_head);
  int npts[] = {5};
  cv::fillPoly(image, pts, npts, 1, red, 8);
}

void RadarModelROS::overlayRobotPose(double robot_x, double robot_y, double robot_head, cv::Mat &image) {
  return overlayRobotPose(&_rfid_belief_maps, robot_x, robot_y, robot_head, image);
}

void RadarModelROS::overlayRobotPose(GridMap *gm, double robot_x, double robot_y, double robot_head, cv::Mat &image){

  cv::Point center;
  int radius = std::ceil(0.25/(_resolution ));
  cv::Scalar red(0, 0, 255);
  center = getPoint(gm, robot_x, robot_y);
  cv::circle(image, center, 5, red, 1);
}

void RadarModelROS::rotatePoints(cv::Point *points, int npts, int cxi, int cyi,
                              double ang) {
  double offsetx, offsety, cx, cy, px, py, cosA, sinA;

  cx = (double)cxi;
  cy = (double)cyi;

  cosA = cos(ang);
  sinA = sin(ang);

  for (int i = 0; i < npts; i++) {
    px = points[i].x;
    py = points[i].y;

    offsetx = (cosA * (px - cx)) + (sinA * (py - cy));
    offsety = -(sinA * (px - cx)) + (cosA * (py - cy));

    points[i].x = ((int)offsetx) + cxi;
    points[i].y = ((int)offsety) + cyi;
  }
}

cv::Mat RadarModelROS::rfidBeliefToCVImg(std::string layer_i){
  return layerToImage(&_rfid_belief_maps,layer_i);
}

cv::Mat  RadarModelROS::layerToImage(GridMap* gm,std::string layerName){
  cv::Mat image;
  const float minValue = (*gm)[layerName].minCoeff();
  const float maxValue = (*gm)[layerName].maxCoeff();
  GridMapCvConverter::toImage<unsigned char, 3>(*gm, layerName, CV_8UC3, minValue, maxValue, image);  
  cv::flip(image, image, -1);
  return image;
}

grid_map::Position RadarModelROS::fromPoint(cv::Point cvp) {
  grid_map::Position p;
  int gm_x, gm_y;

  // cast from opencv indexes to  gridmap indexes
  gm_x = (_Nrow - 1) - cvp.y;
  gm_y = (_Ncol - 1) - cvp.x;

  grid_map::Index index(gm_x, gm_y);

  if (!_rfid_belief_maps.getPosition(index, p)) {
    // ROS_DEBUG_STREAM(" Index ("  << index(0) << ", " << index(1) << ") is out of
    // _rfid_belief_maps bounds!!" );
    gm_x = std::min(std::max(gm_x, 0), _Nrow - 1);
    gm_y = std::min(std::max(gm_y, 0), _Ncol - 1);
    index = grid_map::Index(gm_x, gm_y);
    _rfid_belief_maps.getPosition(index, p);
  }

  return p;
}

//////////////////////////// Other functions ////////////////////////////


void RadarModelROS::saveProbMapDebug(std::string savePATH, int tag_num, int step,
                                  double robot_x, double robot_y,
                                  double robot_head) {
  // Some color  ...
  //cv::Scalar green(0, 255, 0);
  char buffer[10];
  int n;
  cv::Mat image;

  std::string fileURI = savePATH + "T";

  n = sprintf(buffer, "%01d", tag_num);
  fileURI += std::string(buffer) + "_S";
  n = sprintf(buffer, "%03d", step);
  fileURI += std::string(buffer) + "_tempMap.png";

  std::string layerName = getTagLayerName(tag_num);

  if (!_rfid_belief_maps.exists(layerName)){
    // create dummy layer:
    Eigen::MatrixXf layer_mat = grid_map::Matrix::Constant(_Nrow,_Ncol,0.5);
    // remove obstacles:
    Eigen::MatrixXf obst_mat = _rfid_belief_maps["ref_map"];
    layer_mat = (obst_mat.array() == _free_space_val).select(layer_mat, 0.0);
    // normalize prob:   
    layer_mat = layer_mat / layer_mat.sum();

    //get opencv img
    _rfid_belief_maps.add("tempLayer",layer_mat);
    GridMapCvConverter::toImage<unsigned char, 3>(_rfid_belief_maps, "tempLayer", CV_8UC3, _rfid_belief_maps["tempLayer"].minCoeff(), _rfid_belief_maps["tempLayer"].maxCoeff(), image);  
    cv::flip(image, image, -1);
    _rfid_belief_maps.erase("tempLayer");    
  } else{
    // Convert to image.
    image = rfidBeliefToCVImg(layerName);
  }

  // overlay obstacles in green
  cv::Mat image2;
  grid_map::Matrix obstMat = _rfid_belief_maps["ref_map"];
  
  obstMat = 1.0f - obstMat.array(); 
  _rfid_belief_maps.add("tempLayer",obstMat);

  const float minValue = _rfid_belief_maps["tempLayer"].minCoeff();
  const float maxValue = _rfid_belief_maps["tempLayer"].maxCoeff();

  GridMapCvConverter::toImage<unsigned char, 3>(_rfid_belief_maps, "tempLayer", CV_8UC3, minValue, maxValue, image2);  
  cv::flip(image2, image2, -1);
  _rfid_belief_maps.erase("tempLayer");    

  std::vector<cv::Mat> channels,channels2;
  cv::split(image, channels);
  cv::split(image2, channels2);

  // b,g,r
  std::vector<cv::Mat> newChannes = { channels[0], channels[1]+channels2[2], channels[2] };
  cv::merge(newChannes, image);

  /// overlay tag position


  cv::Point tag_center;

  /// overlay robot position
  /// .................................................................................................
  overlayRobotPoseT(robot_x, robot_y, robot_head, image);

  // and save
  cv::imwrite( fileURI, image );

  // mfc trick used to see temp matrixes as images
  // add on ......................................................................................................
  // if (_rfid_belief_maps.exists("obst_losses")){
  //   image = rfidBeliefToCVImg("obst_losses");
  //   // overlay tag position 
  //   cv::circle(image, tag_center , 5, green, 1);
  //   // overlay robot position 
  //   overlayRobotPoseT(robot_x, robot_y, robot_head, image);
    
  //   fileURI = savePATH + "T";
  //   n=sprintf (buffer, "%01d", tag_num);
  //   fileURI += std::string(buffer) +"_rxPower_S";
  //   n=sprintf (buffer, "%03d", step);
  //   fileURI += std::string(buffer)+ ".png";

  //   cv::imwrite( fileURI, image );
  // }
}

void RadarModelROS::clearObstacles(cv::Mat &image) {
  // MFC this method does not work .... dont use it!
  int b = 0;
  int a = 1 / b;
  //////////////////////////////////

  cv::Vec<unsigned char, 3> red = {0, 0, 255};

  // Initialize result image.
  cv::Mat result = image.clone().setTo(cv::Scalar(255, 255, 255));

  // Convert to image.
  cv::Mat ref_img = rfidBeliefToCVImg("ref_map");

  uint8_t ref_val;
  // Copy pixels from background to result image, where pixel in mask is 0.
  for (int x = 0; x < image.size().width; x++) {
    for (int y = 0; y < image.size().height; y++) {
      ref_val = ref_img.at<uint8_t>(y, x);
      if (ref_val == 0) {
        result.at<cv::Vec3b>(y, x) = red;
      } else {
        result.at<cv::Vec3b>(y, x) = image.at<cv::Vec3b>(y, x);
      }
      if ((ref_val != 0) && (ref_val != 255)) {
        ROS_DEBUG_STREAM( "Map image NOT BINARY AT (" << x << ", " << y
                  << ") = " << ref_val );
      }
    }
  }

  // there you go ...
  image = result;

  // ROS_DEBUG_STREAM("Map image (" << ref_img.size().width << ", "
  // <<ref_img.size().height<<") pixels" ); ROS_DEBUG_STREAM("Out image ("
  // << image.size().width << ", " <<image.size().height<<") pixels"
  // ); and save
  cv::imwrite("/tmp/refMap.png", ref_img);

  // cv::Mat rgba;

  // // First create the image with alpha channel
  // cv::cvtColor(image, rgba , cv::cv::COLOR_GRAY2RGBA);

  // // Split the image for access to alpha channel
  // std::vector<cv::Mat>channels(4);
  // cv::split(rgba, channels);

  // // Assign the mask to the last channel of the image
  // channels[3] = alpha_data;

  // // Finally concat channels for rgba image
  // cv::merge(channels, 4, rgba);
}

std::vector<double> RadarModelROS::range(double start, double stop, double step) {
  std::vector<double> ans;
  double currVal = start;
  while (currVal <= stop) {
    ans.push_back(currVal);
    currVal += step;
  }
  return ans;
}

double RadarModelROS::antennaPlaneLoss(double angleRad) {
  double g_ans;
  double g_i = -22.6;
  double g_p = -22.6;
  int i;
  int p;
  double ang_i = 0;
  double ang_p = 0;
  double m = 0;
  double g_o = 0;
  double angleDeg = angleRad * 180.0 / M_PI;

  if (fabs(angleDeg) > 180.0) {
    ROS_DEBUG_STREAM( "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" );
    ROS_DEBUG_STREAM( "Angle (" << angleDeg << ") deg. " );
    ROS_DEBUG_STREAM( "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" );
  }

  // gain list entries start at -180 degrees to 180 in steps of 15.
  double fIndex = (angleDeg + 165.0) / 15.0;

  // we dont want to index out of the range ...
  i = std::max(std::min((int)ceil(fIndex), 24), 0);
  p = std::max(std::min((int)floor(fIndex), 24), 0);

  ang_i = (i * 15.0) - 180.0;
  ang_p = (p * 15.0) - 180.0;

  g_i = ANTENNA_LOSSES_LIST[i];
  g_p = ANTENNA_LOSSES_LIST[p];

  if (i != p) {
    m = (g_i - g_p) / (ang_i - ang_p);
  }

  g_o = g_i - (m * ang_i);
  g_ans = m * angleDeg + g_o;

  //

  return g_ans;
}

float RadarModelROS::sign(float x) {
  if (x > 0.0)
    return 1.0;
  if (x < 0.0)
    return -1.0;
  return 0.0;
}

double RadarModelROS::received_power_friis_with_obstacles(
    double antenna_x, double antenna_y, double antenna_h, double tag_x,
    double tag_y, double tag_h, double freq, double txtPower) {
  return received_power_friis_with_obstacles(antenna_x, antenna_y, antenna_h,
                                             tag_x, tag_y, tag_h, freq,
                                             txtPower, _antenna_gains);
}

double RadarModelROS::received_power_friis_with_obstacles(
    double antenna_x, double antenna_y, double antenna_h, double tag_x,
    double tag_y, double tag_h, double freq, double txtPower,
    SplineFunction antennaGainsModel) {

  double rel_tag_x, rel_tag_y, rel_tag_r, rel_tag_h, rxP, wall_losses, delta_x,
      delta_y;
  int count_obs_cell;

  // Get robot - tag relative pose
  delta_x = (tag_x - antenna_x);
  delta_y = (tag_y - antenna_y);

  // rotate
  rel_tag_x = delta_x * cos(antenna_h) + delta_y * sin(antenna_h);
  rel_tag_y = -delta_x * sin(antenna_h) + delta_y * cos(antenna_h);

  getSphericCoords(rel_tag_x, rel_tag_y, rel_tag_r, rel_tag_h);

  rxP = received_power_friis_polar(rel_tag_r, rel_tag_h, freq, txtPower,
                                   antennaGainsModel);

  if (rxP > SENSITIVITY) {
    // Check if there is line of sight between antenna and tag
    Index antenna_index;
    Index tag_index;
    _rfid_belief_maps.getIndex(Position(antenna_x, antenna_y), antenna_index);
    _rfid_belief_maps.getIndex(Position(tag_x, tag_y), tag_index);

    count_obs_cell = 0;
    wall_losses = 0;

    for (grid_map::LineIterator iterator(_rfid_belief_maps, antenna_index,
                                         tag_index);
         !iterator.isPastEnd(); ++iterator) {
      if ((_rfid_belief_maps.at("ref_map", *iterator) != _free_space_val)) {
        count_obs_cell++;
      }
    }

    // Each "wall" adds around 3dB losses. A wall is ~15cm thick, then each cell
    // adds  3 * resolution / 0.15 dB losses
    wall_losses = 20.0 * _resolution * count_obs_cell;

    rxP = rxP - wall_losses;
  }

  if (rxP < SENSITIVITY) {
    rxP = SENSITIVITY;
  }

  return rxP;
}

double
RadarModelROS::received_power_friis_polar(double tag_r, double tag_h, double freq,
                                       double txtPower,
                                       SplineFunction antennaGainsModel) {
  double rxPower = txtPower;
  double ant1, antL, propL;
  double lambda = C / freq;
  // otherwise friis approach does not apply
  if (tag_r > 2 * lambda) {
    /*
    SIMPLIFICATION!!! TAG is OMNIDIRECTIONAL
    (a.k.a. don't have tag radiation pattern and
    Here they say it's ok
    https://www.hindawi.com/journals/ijap/2013/194145/tab4/
    */
    ant1 = antennaGainsModel.interpRad(tag_h);

    antL = TAG_LOSSES + ant1;

    // propagation losses
    propL = LOSS_CONSTANT - (20 * log10(tag_r * freq));
    // signal goes from antenna to tag and comes back again, so we double the
    // losses
    rxPower += 2 * antL + 2 * propL;
  }

  if (rxPower > txtPower) {
    ROS_DEBUG_STREAM( "ERROR! MORE POWER RECEIVED THAN TRANSMITTED!!!!!!!!!!!!! ");
    ROS_DEBUG_STREAM( "Relative tag polar pose: (" << tag_r << " m.,"  << tag_h * 180 / M_PI << " deg.)" );
    ROS_DEBUG_STREAM( "At Freq.: (" << freq / 1e6 << " MHz.)" );
    ROS_DEBUG_STREAM( "Lambda: (" << lambda << " m.)" );
    ROS_DEBUG_STREAM( "Tx Pw.: (" << txtPower << " dB)" );
    ROS_DEBUG_STREAM( "Rx Pw.: (" << rxPower << " dB)" );
    ROS_DEBUG_STREAM( "Antenna losses: (" << antL << " dB)" );
    ROS_DEBUG_STREAM( "Propagation losses: (" << propL << " dB)" );
    ROS_DEBUG_STREAM( "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " );
  }

  return rxPower;
}

double RadarModelROS::phaseDifference(double tag_x, double tag_y, double freq) {
  double phi;
  double r;

  getSphericCoords(tag_x, tag_y, r, phi);
  double phase = PHASE_CONSTANT * freq * r;
  phase = fmod(phase, M_PI);
  return phase;
}

std::pair<int, std::pair<int, int>>
RadarModelROS::findTagFromBeliefMap(int num_tag) {

  // Access the belief map of every tag
  std::string layerName = getTagLayerName(num_tag);
  GridMap::Matrix &grid = _rfid_belief_maps[layerName];

  std::pair<int, int> tag(0, 0);
  double powerRead = 0;
  // for(int row=0; row < grid.getLength().x(); row++)
  // {
  //   for(int col=0; col < grid.getLength().y(); col++)
  //   {

  for (GridMapIterator iterator(_rfid_belief_maps); !iterator.isPastEnd();
       ++iterator) {
    const Index index(*iterator);
    // For every cell, analyse the surrounding area
    double tmp_power = 0.0;
    tmp_power = grid(index(0), index(1));
    int buffer_size = 3;
    if (index(0) > buffer_size and
        index(0) <= _rfid_belief_maps.getLength().x() - buffer_size) {
      if (index(1) > buffer_size and
          index(1) <= _rfid_belief_maps.getLength().y() - buffer_size) {
        // ROS_DEBUG_STREAM( "I: " << index(0) << "," << index(1) );
        Index submapStartIndex(index(0) - buffer_size, index(1) - buffer_size);
        Index submapBufferSize(buffer_size, buffer_size);
        for (grid_map::SubmapIterator sub_iterator(
                 _rfid_belief_maps, submapStartIndex, submapBufferSize);
             !sub_iterator.isPastEnd(); ++sub_iterator) {
          Index sub_index(*sub_iterator);
          // ROS_DEBUG_STREAM( "I: " << sub_index(0) << "," << sub_index(1) );
          tmp_power += grid(sub_index(0), sub_index(1));
        }
      }
    }

    // for (int i = -3; i <= 3; i++){
    //   for (int j = -3; j <= 3; j++){
    //     tmp_power = tmp_power + grid.getCell(row, col);
    //   }
    // }
    // tmp_power = grid.getCell(row, col);
    if (tmp_power > powerRead) {
      powerRead = tmp_power;

      // Normalise the tag coordinate to follow Ricc's system
      tag.first = _rfid_belief_maps.getLength().x() - index(0);
      tag.second = _rfid_belief_maps.getLength().y() - index(1);
    }

    // }
  }
  std::pair<int, std::pair<int, int>> final_return(powerRead, tag);

  // ROS_DEBUG_STREAM( "Value read: " << powerRead );
  return final_return;
}

void RadarModelROS::normalizeRFIDLayer(std::string layerName) {
  double totalW = _rfid_belief_maps[layerName].sum();
  if (totalW > 0) {
    _rfid_belief_maps[layerName] = _rfid_belief_maps[layerName] / totalW;
  }

  // double minW = _rfid_belief_maps[layerName].minCoeff();
  // double maxW = _rfid_belief_maps[layerName].maxCoeff();
  // _rfid_belief_maps.add("min", minW);
  // _rfid_belief_maps.add("max", minW);
  // _rfid_belief_maps[layerName] = (_rfid_belief_maps[layerName] -
  // _rfid_belief_maps["min"])/ (maxW - minW);
}

void RadarModelROS::clearObstacleCellsRFIDLayer(std::string layerName) {
    // this should remove prob at obstcles
    Eigen::MatrixXf layer_mat = _rfid_belief_maps[layerName];
    Eigen::MatrixXf obst_mat = _rfid_belief_maps["ref_map"];
    layer_mat = (obst_mat.array() == _free_space_val).select(layer_mat, 0.0);
     _rfid_belief_maps[layerName] = layer_mat;
}

void RadarModelROS::addTagID(std::string tagID, int i) {
  std::string tagLayerName;

  tagLayerName = getTagLayerName(i);
  if (!_rfid_belief_maps.exists(tagLayerName)){
    addTagLayer(i);
  }
}
  
///////////////  ADD MEASUREMENT METHOD ////////////////////////////////////////

// So, robot at pr (x,y,orientation) (long, long, int) receives
// rxPower,phase,freq from tag i .
// TODO: this is a simplistic model that just "ignores" walls: but they do have
// absortion and reduce received power at their locations...
void RadarModelROS::addMeasurement(double x_m, double y_m, double orientation_deg,
                                double rxPower, double phase, double freq,
                                int i, double txtPower) {

  Eigen::MatrixXf rxPw_mat, likl_mat;
  std::string tagLayerName;

  tagLayerName = getTagLayerName(i);
  if (!_rfid_belief_maps.exists(tagLayerName)){
    addTagLayer(i);
  }

  Size siz = _rfid_belief_maps.getSize();
  rxPw_mat = Eigen::MatrixXf(siz(0), siz(1));

  // get the expected power at each point
  rxPw_mat = getFriisMat(x_m, y_m, orientation_deg, freq,txtPower);

  // get the likelihood of the received power at each point
  likl_mat = getProbCond(rxPw_mat, rxPower, _sigma_power);

  // Where X_mat is < than SENSITIVITY, the tag wont be... prob 0
  // likl_mat = (likl_mat.array()<=SENSITIVITY).select(0,likl_mat);

  // this should remove prob at obstcles
  Eigen::MatrixXf obst_mat = _rfid_belief_maps["ref_map"];
  likl_mat = (obst_mat.array() == _free_space_val).select(likl_mat, 0);

  // normalize in this space:
  double bayes_den = likl_mat.sum();
  if (bayes_den > 0) {
    likl_mat = likl_mat / bayes_den;
    // now do bayes ...  everywhere
    _rfid_belief_maps[tagLayerName] =
        _rfid_belief_maps[tagLayerName].cwiseProduct(likl_mat);
  }
  normalizeRFIDLayer(tagLayerName);
  // ROS_DEBUG_STREAM( "Max belief: [" <<  _rfid_belief_maps.get(tagLayerName).maxCoeffOfFinites() <<"]" );
  // ROS_DEBUG_STREAM( "Belief sum: [" <<  _rfid_belief_maps[tagLayerName].sum() <<"]" );

}

double RadarModelROS::getTotalEntropy(double x, double y, double orientation,
                                   double size_x, double size_y, int tag_i) {
  // TODO: I'm not using the orientation. Maybe it would be better to use a
  // polygon iterator,
  //     so we can rotate edges around the center and have a more flexible thing

  // submapStartIndex the start index of the submap, typically top-left index.
  grid_map::Index submapStartIndex, submapEndIndex, submapBufferSize;
  grid_map::Position submapStartPosition(x + (size_x / 2), y + (size_y / 2));
  grid_map::Position submapEndPosition(x - (size_x / 2), y - (size_y / 2));

  if (!_rfid_belief_maps.getIndex(submapStartPosition, submapStartIndex)) {
    submapStartIndex = grid_map::Index(0, 0);
    // ROS_DEBUG_STREAM("Clip start!" );
  }

  if (!_rfid_belief_maps.getIndex(submapEndPosition, submapEndIndex)) {
    Size siz = _rfid_belief_maps.getSize();
    submapEndIndex = grid_map::Index(siz(0) - 1, siz(1) - 1);
    // ROS_DEBUG_STREAM("Clip end!" );
  }

  submapBufferSize = submapEndIndex - submapStartIndex;

  grid_map::SubmapIterator iterator(_rfid_belief_maps, submapStartIndex,
                                    submapBufferSize);
  return getTotalEntropy(x, y, orientation, iterator, tag_i);
}

double RadarModelROS::getTotalEntropy(double x, double y, double orientation,
                                   grid_map::SubmapIterator iterator,
                                   int tag_i) {

  double total_entropy;
  Position point;
  double likelihood, neg_likelihood, log2_likelihood, log2_neg_likelihood = 0.0;

  std::string tagLayerName = getTagLayerName(tag_i);

  total_entropy = 0;
  for (iterator; !iterator.isPastEnd(); ++iterator) {
    _rfid_belief_maps.getPosition(*iterator, point);
    // check if is inside global map
    if (_rfid_belief_maps.isInside(point)) {
      // We don't add belief from positions considered obstacles...
      if (_rfid_belief_maps.atPosition("ref_map", point) == _free_space_val) {
        likelihood = _rfid_belief_maps.atPosition(tagLayerName, point);
        if (isnan(likelihood))
          likelihood = 0.0;
        neg_likelihood = 1 - likelihood;
        if (isnan(neg_likelihood))
          neg_likelihood = 0.0;

        log2_likelihood = log2(likelihood);
        if (isinf(log2_likelihood))
          log2_likelihood = 0.0;
        log2_neg_likelihood = log2(neg_likelihood);
        if (isinf(log2_neg_likelihood))
          log2_neg_likelihood = 0.0;
        // ROS_DEBUG_STREAM( " l: " << log2_likelihood );
        // likelihood =
        // rfid_tools->rm.getBeliefMaps().atPosition(layerName,rel_point);
        total_entropy += -likelihood * log2_likelihood -
                         neg_likelihood * log2_neg_likelihood;
      }
    }
  }
  return total_entropy;
}

double RadarModelROS::getTotalEntropyEllipse(double antennaX, double antennaY, double antennaHeading_rad , double maxX, double minX, int tag_i){
                                     //1.-  Get elipsoid iterator.
  // Antenna is at one of the focus of the ellipse with center at antennaX, antennaY, tilted antennaHeading .
  // http://www.softschools.com/math/calculus/finding_the_foci_of_an_ellipse/
  // if a is mayor axis and b is minor axis
  // a-c= minX
  // a+c= maxX
  // a = (maxX + minX)/2
  // c  = maxX/2 + minX
  // b  = sqrt(a^2-c^2)


  double a =  (abs(maxX) + abs(minX))/2.0;
  double c =  (abs(maxX) - abs(minX))/2;
  double b = sqrt((a*a)-(c*c));
  double xc = antennaX + (c*cos(antennaHeading_rad));
  double yc = antennaY + (c*sin(antennaHeading_rad));

  Position center(xc, yc); // meters
  Length length(2*a, 2*b);
  grid_map::EllipseIterator el_iterator(_rfid_belief_maps, center, length, antennaHeading_rad);
  return getTotalEntropyEllipse(el_iterator, tag_i);
}

double RadarModelROS::getTotalEntropyEllipse(grid_map::EllipseIterator iterator,
                                   int tag_i) {

  double total_entropy;
  Position point;
  double likelihood, neg_likelihood, log2_likelihood, log2_neg_likelihood = 0.0;

  std::string tagLayerName = getTagLayerName(tag_i);

  total_entropy = 0;
  for (iterator; !iterator.isPastEnd(); ++iterator) {
    _rfid_belief_maps.getPosition(*iterator, point);
    // check if is inside global map
    if (_rfid_belief_maps.isInside(point)) {
      // We don't add belief from positions considered obstacles...
      if (_rfid_belief_maps.atPosition("ref_map", point) == _free_space_val) {
        likelihood = _rfid_belief_maps.atPosition(tagLayerName, point);
        if (isnan(likelihood))
          likelihood = 0.0;
        neg_likelihood = 1 - likelihood;
        if (isnan(neg_likelihood))
          neg_likelihood = 0.0;

        log2_likelihood = log2(likelihood);
        if (isinf(log2_likelihood))
          log2_likelihood = 0.0;
        log2_neg_likelihood = log2(neg_likelihood);
        if (isinf(log2_neg_likelihood))
          log2_neg_likelihood = 0.0;
        // ROS_DEBUG_STREAM( " l: " << log2_likelihood );
        // likelihood =
        // rfid_tools->rm.getBeliefMaps().atPosition(layerName,rel_point);
        total_entropy += -likelihood * log2_likelihood -
                         neg_likelihood * log2_neg_likelihood;
      }
    }
  }
  return total_entropy;
}

void RadarModelROS::printEllipse(double x, double y, double orient_rad, double maxX, double minX){
  std::string fileURI;

  ROS_DEBUG_STREAM( "[ELLIPSE] x: " << x << ", y: " << y << ", orient_rad: " << orient_rad );

  double a =  (abs(maxX) + abs(minX))/2.0;
  double c =  (abs(maxX) - abs(minX))/2;
  double b = sqrt((a*a)-(c*c));
  double xc = x + (c*cos(orient_rad));
  double yc = y + (c*sin(orient_rad));

  Position center(xc, yc);
  Length length(2*a, 2*b);
  grid_map::EllipseIterator iterator(_rfid_belief_maps, center, length, orient_rad);

  _rfid_belief_maps.add("ellipse_test", 0);  

  for (iterator; !iterator.isPastEnd(); ++iterator) {
   _rfid_belief_maps.at("ellipse_test",*iterator) = 1;
  }

 

  fileURI = "/tmp/testEllipse.png";
  getImage(&_rfid_belief_maps, "ellipse_test", fileURI);
  _rfid_belief_maps.erase("ellipse_test");  
}

double RadarModelROS::getTotalKL(double x, double y, double orientation,
                              double size_x, double size_y, int tag_i) {
  // TODO: I'm not using the orientation. Maybe it would be better to use a
  // polygon iterator,
  //     so we can rotate edges around the center and have a more flexible thing

  // submapStartIndex the start index of the submap, typically top-left index.
  grid_map::Index submapStartIndex, submapEndIndex, submapBufferSize;
  grid_map::Position submapStartPosition(x + (size_x / 2), y + (size_y / 2));
  grid_map::Position submapEndPosition(x - (size_x / 2), y - (size_y / 2));

  if (!_rfid_belief_maps.getIndex(submapStartPosition, submapStartIndex)) {
    submapStartIndex = grid_map::Index(0, 0);
    // ROS_DEBUG_STREAM("Clip start!" );
  }

  if (!_rfid_belief_maps.getIndex(submapEndPosition, submapEndIndex)) {
    Size siz = _rfid_belief_maps.getSize();
    submapEndIndex = grid_map::Index(siz(0) - 1, siz(1) - 1);
    // ROS_DEBUG_STREAM("Clip end!" );
  }

  submapBufferSize = submapEndIndex - submapStartIndex;

  grid_map::SubmapIterator iterator(_rfid_belief_maps, submapStartIndex,
                                    submapBufferSize);

  return getTotalKL(x, y, orientation, iterator, tag_i);
}

double RadarModelROS::getTotalKL(double x, double y, double orientation,
                              grid_map::SubmapIterator iterator, int tag_i) {

  double total_KL, tmp_KL = 0.0;
  Position point;
  double prior, posterior = 0.0;

  std::string tagLayerName = getTagLayerName(tag_i);

  for (iterator; !iterator.isPastEnd(); ++iterator) {
    _rfid_belief_maps.getPosition(*iterator, point);
    // check if is inside global map
    if (_rfid_belief_maps.isInside(point)) {
      // We don't add belief from positions considered obstacles...
      if (_rfid_belief_maps.atPosition("ref_map", point) == _free_space_val) {
        prior = _rfid_belief_maps.atPosition(tagLayerName, point);
        posterior = _rfid_belief_maps.atPosition("kl", point);
        tmp_KL = posterior * log(posterior / prior);
        if (isnan(tmp_KL))
          tmp_KL = 0;
        total_KL += tmp_KL;
        // ROS_DEBUG_STREAM( "Prior: " << prior );
        // ROS_DEBUG_STREAM( "Posterior: " << posterior );
        // ROS_DEBUG_STREAM( "totalKL: " << total_KL );
      }
    }
  }
  return total_KL;
}

void RadarModelROS::addTmpMeasurementRFIDCriterion(double x_m, double y_m,
                                                double orientation_deg,
                                                double rxPower, double phase,
                                                double freq, int i,
                                                double len_update) {

  // ROS_DEBUG_STREAM( "computeKL: " << computeKL );
  double rel_x, rel_y, prob_val, orientation_rad;
  double glob_x, glob_y, delt_x, delt_y;
  Position rel_point, glob_point;
  grid_map::Polygon update_edges;

  double prior, posterior, likelihood, bayes_num, bayes_den;
  Eigen::MatrixXf prob_mat;
  std::string tagLayerName = getTagLayerName(i);

  _rfid_belief_maps.add("kl", 0.0);

  // First we get the Probability distribution associated with (
  // rxPower,phase,freq) using our defined active area grids
  if (rxPower > SENSITIVITY) {
    prob_mat = getPowProbCondRFIDCriterion(
        rxPower, freq); //.cwiseProduct(getPhaseProbCond(phase, freq));
                        // } else{
    //   prob_mat = getNegProb(getPowLayerName(freq), rxPower,
    //   _sigma_power);//.cwiseProduct(getPhaseProbCond(phase, freq));
    // //   if (i == 0){
    // //     ROS_DEBUG_STREAM( "Neg: " << prob_mat.sum() );
    // //   }
    // }

    // We store this data matrix in a temporal layer
    this->createTempProbLayerRFIDCriterion(
        prob_mat, x_m, y_m, orientation_deg,
        len_update); // size of the activeArea

    // so we need to translate this matrix to robot pose and orientation
    orientation_rad = orientation_deg * M_PI / 180.0;
    // We ned to normalise the new probability over the entire grid, so
    // we need to multiple all the priors for all the new measurements
    // and this will be our denominator while applying Bayes
    // NB: posterior = likelihood * prior / normalizing_factor
    // where normalisizing_factor is a sum over all the grid of likelihood *
    // prior bayes_den = getNormalizingFactorBayesRFIDActiveArea(x_m, y_m,
    // orientation_rad, tagLayerName);  // computed on the entire activeArea
    // if (bayes_den != 0.0 and !isnan(bayes_den)){
    // Now we can proceed with the update
    update_edges = getSubMapEdges(
        x_m, y_m, orientation_rad,
        len_update); // NOTE: update computed only on a small subset
    for (grid_map::PolygonIterator iterator(_rfid_belief_maps, update_edges);
         !iterator.isPastEnd(); ++iterator) {
      // check if point is an obstacle:
      if (_rfid_belief_maps.at("ref_map", *iterator) == _free_space_val) {
        // get the relative point
        _rfid_belief_maps.getPosition(*iterator, glob_point);
        // rel_point = getRelPoint(glob_point, x_m, y_m, orientation_rad);
        
        // We probably don't need this anymore after dropping activemap
        // rel_point = getSubMapRelPoint(glob_point, x_m, y_m, orientation_rad,
        //                               len_update);
        likelihood =
            _tmp_rfid_c_map.atPosition("temp", rel_point); // the measurement
        prior = _rfid_belief_maps.at(tagLayerName,
                                     *iterator); // the value in the map
        bayes_num = prior * likelihood;
        // posterior = bayes_num / bayes_den;
        _rfid_belief_maps.at("kl", *iterator) = bayes_num;
      } else {
        // this shouldn't be necessary ....
        _rfid_belief_maps.at(tagLayerName, *iterator) = 0;
      }
    }
    // }
  }
}

Eigen::MatrixXf RadarModelROS::getPowProbCondRFIDCriterion(double rxPw,
                                                        double f_i) {
  // probability of receiving less than X1 in every cell
  Eigen::MatrixXf x1_mat = getNegProbRFIDCriterion(rxPw, _sigma_power);
  // ROS_DEBUG_STREAM( "x1_mat: " << x1_mat.sum()/(siz(0)*siz(1)) );

  // probability of receiving between X1 and X2 in every cell
  Eigen::MatrixXf ans = (x1_mat).array().abs();

  return ans;
}

void RadarModelROS::createTempProbLayerRFIDCriterion(Eigen::MatrixXf prob_mat,
                                                  double x_m, double y_m,
                                                  double orientation_deg,
                                                  double len_update) {
  double orientation_rad;
  Position rel_point, glob_point;
  grid_map::Polygon update_edges;

  // We store this data matrix in a temporal layer
  // Here, boundaries are relative to position (0,0,0º): p1 (nx/2,ny/2) ,
  // pcreateTempProbLayerRFIDCriterion2 (-nx/2, ny/2), p3 (-nx/2,-ny/2), p4
  // (nx/2,-ny/2)
  _tmp_rfid_c_map.add("temp", prob_mat);
  // now we remove from this layer probabilities inside obstacle cells
  // so we need to translate this matrix to robot pose and orientation
  orientation_rad = orientation_deg * M_PI / 180.0;
  update_edges = getSubMapEdges(x_m, y_m, orientation_rad, len_update);

  for (grid_map::PolygonIterator iterator(_rfid_belief_maps, update_edges);
       !iterator.isPastEnd(); ++iterator) {
    // check if it's an obstacle:
    if (_rfid_belief_maps.at("ref_map", *iterator) != _free_space_val) {
      _rfid_belief_maps.getPosition(*iterator, glob_point);
      // We don't need this anymore without active area
      // rel_point =
      //     getSubMapRelPoint(glob_point, x_m, y_m, orientation_rad, len_update);
      // remove prob in obstacles
      _tmp_rfid_c_map.atPosition("temp", rel_point) = 0.0;
    }
  }
}

Eigen::MatrixXf RadarModelROS::getNegProbRFIDCriterion(double x, double sigm) {
  Eigen::MatrixXf friis_mat = _tmp_rfid_c_map["temp"];
  // gaussian cdp
  // cdp(x,mu,sigma) = 0.5 * ( 1 + erf( (x - mu) / (sigma * sqrt(2)) )
  Eigen::MatrixXf erf_mat = (x - friis_mat.array()) / (sigm * sqrt(2.0));
  erf_mat = 0.5 + 0.5 * erf_mat.array().erf();
  return erf_mat;
}

grid_map::Polygon RadarModelROS::getSubMapEdges(double robot_x, double robot_y,
                                             double robot_head, double len) {
  grid_map::Polygon polygon;

  // trying not to replicate code, I'm using the cv points one that works
  cv::Point cvrobot = getPoint(robot_x, robot_y);
  cv::Point pts[4];
  pts[0] = getPoint(robot_x - len / 2, robot_y - len / 2);
  pts[1] = getPoint(robot_x - len / 2, robot_y + len / 2);
  pts[2] = getPoint(robot_x + len / 2, robot_y + len / 2);
  pts[3] = getPoint(robot_x + len / 2, robot_y - len / 2);

  rotatePoints(pts, 4, cvrobot.x, cvrobot.y, robot_head);

  // cast to gridmap points and add to polygon
  polygon.setFrameId(_rfid_belief_maps.getFrameId());
  for (int i = 0; i < 4; i++) {
    polygon.addVertex(fromPoint(pts[i]));
  }

  return polygon;
}