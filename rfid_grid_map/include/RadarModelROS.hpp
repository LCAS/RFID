#ifndef RADARMODELROS_H
#define RADARMODELROS_H

//ROS MSGS
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

//GRIDMAP
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_core/GridMap.hpp>

#include <Eigen/Eigen> // AFTER GRIDMAP!
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

// Math
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <math.h>
#include <vector>

// other
#include <iomanip>
#include <string>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace grid_map;
using namespace std::placeholders;
using Eigen::MatrixXf;

// constants ..................................................................
// We mostly use UPM frog 3D.
const double TAG_LOSSES = -4.8;

// 20 * log10 ( c / (4*pi) )
const double LOSS_CONSTANT = 147.55;
// 4*pi/c
const double PHASE_CONSTANT = 4.192e-8;
const double C = 299792458.0;

// This comes from the manufacturer. Azimut
// gain list entries start at -180 degrees to 180 in steps of 15.
const double ANTENNA_LOSSES_LIST[25] = {
    -22.6, -25.2, -25,   -20.2, -17.6, -15.6, -14,  -11.2, -7.8,
    -5.2,  -2.4,  -0.6,  0,     -0.6,  -2.4,  -5.2, -8.8,  -12.2,
    -16.4, -19.2, -20.8, -24.4, -28.2, -24,   -22.6};
const double ANTENNA_ANGLES_LIST[25] = {
    -180.0, -165.0, -150.0, -135.0, -120.0, -105.0, -90.0, -75.0, -60.0,
    -45.0,  -30.0,  -15.0,  0.0,    15.0,   30.0,   45.0,  60.0,  75.0,
    90.0,   105.0,  120.0,  135.0,  150.0,  165.0,  180.0};

// M6e RFID reader Specs
// Minimum required power to identify a tag. Also depends on other factors, but
// this value is a guide
const double SENSITIVITY = -115; // dB

// Max transmitted power may be limited by the Region regulations
// see Power Requirements  in M6e Hardware Guide
const double MIN_TX_POWER = -25;  // dB
const double MAX_TX_POWER = 0;    // dB
const double STEP_TX_POWER = 0.5; // dB

// These freqs ARE limited depending on the region regulations
// see Regional Frequency Quantization in M6e Hardware Guide
const double STEP_FREQ = 25e3;   // Hertzs
const double MIN_FREQ_A = 865e6; // Hertzs
const double MAX_FREQ_A = 869e6; // Hertzs
const double MIN_FREQ_B = 902e6; // Hertzs
const double MAX_FREQ_B = 928e6; // Hertzs

// most likely we will use EU or NA regions....
const double MIN_FREQ_EU = 865.6e6; // Hertzs
const double MAX_FREQ_EU = 867.6e6; // Hertzs
const double STEP_FREQ_EU = 100e3;  // Hertzs

const double MIN_FREQ_NA = 902e6;  // Hertzs
const double MAX_FREQ_NA = 928e6;  // Hertzs
const double STEP_FREQ_NA = 250e3; // Hertzs

//  ..................................................................

/**
 * @brief The spline is used to interpolate antenna gain values,
 * as we only have the graph
 */
class SplineFunction {

public:
  SplineFunction();

  /**
   * @brief Construct a new Spline Function Interpolation for antenna gain
   *
   * @param x_vec reference azimuth angle (deg) points
   * @param y_vec reference gain (db) points
   */
  SplineFunction(Eigen::VectorXd const &x_vec, Eigen::VectorXd const &y_vec);

  /**
   * @brief Interpolate gain from angle
   *
   * @param x azimuth angle (rads)
   * @return double interpolated gain (dB)
   */
  double interpRad(double x) const;
  float interpRadf(float x) const;

  /**
   * @brief Interpolate gain from angle
   *
   * @param x azimuth angle (degs)
   * @return double interpolated gain (dB)
   */
  double interpDeg(double x) const;

private:
  double scaled_value(double x) const;

  Eigen::RowVectorXd scaled_values(Eigen::VectorXd const &x_vec) const;

  double x_min;
  double x_max;
  double y_min;
  double y_max;

  // Spline of one-dimensional "points."
  Eigen::Spline<double, 1> spline_;
};

//////////////////////////

class RadarModelROS {

public:
  int _Ncol; // number of rows of reference and rfid belief maps (cells)
  int _Nrow; // number of cols of reference and rfid belief maps (cells)
  float _free_space_val; // max value stored in reference map, used as free
                         // space marker
  double _resolution;    // Reference AND belief map resolution (m./cell)

  // we only model here gaussian noise
  double _sigma_power; // noise factor
  double _sigma_phase; // noise factor

  GridMap _rfid_belief_maps; // Prob. beliefs. One layer per tag. Also, one
                             // layer with reference map, mostly for tag layout
                             // representation.
  GridMap _tmp_rfid_c_map;

  std::vector<std::pair<double, double>>  _tags_coords; // tag locations in reference map coords (m.)
  int _numTags;     // rfid tags to consider

  SplineFunction _antenna_gains; // model for antenna power gain depending on
                                 // the angle (dB.)    
  // gains cache for hopefully faster access.
  std::vector<float> _antenaGainVector; 
  float gainValue(int x);
  
  RadarModelROS(const nav_msgs::OccupancyGrid& nav_map, const double sigma_power, const double sigma_phase, const double resolution );
  void initRefMap(const nav_msgs::OccupancyGrid& nav_map);

  void loadBelief(const std::string imageURI);

  void saveBelief(const std::string imageURI);

  /**
   * @brief Construct a new Radar Model object
   *        It is based upon the link budget equation. See D. M. Dobkin, “The RF
   * in RFID: Passive UHF RFID in Practice”, Elsevier, 2007
   * @param resolution   reference map and rfid belief map images resolution in
   * m./cell
   * @param sigma_power  noise std in log model
   * @param sigma_phase  noise std in phase
   * @param txtPower     transmitted power (dB)
   * @param freqs        frequencies used in transmission
   * @param tags_coords  tag positions in map coordinates
   * @param imageFileURI reference map file
   *   TODO We will use a fixed antenna model, Gaussian noise model and will
   * assume tags are isotropic. This may need to be revisited ...
   */
  RadarModelROS(const double resolution, const double sigma_power,
             const double sigma_phase, 
             const std::vector<std::pair<double, double>> tags_coords,
             const std::string imageFileURI);

  RadarModelROS();

  /**
   * @brief Print scenario map with tags
   *
   * @param fileURI
   */
  void PrintRefMapWithTags(std::string fileURI);

  double received_power_friis_with_obstacles(double antenna_x, double antenna_y,
                                             double antenna_h, double tag_x,
                                             double tag_y, double tag_h,
                                             double freq, double txtPower,
                                             SplineFunction antennaGainsModel);

  double received_power_friis_with_obstacles(double antenna_x, double antenna_y,
                                             double antenna_h, double tag_x,
                                             double tag_y, double tag_h,
                                             double freq, double txtPower);
  /**
   * @brief Plots a power distribution
   *
   * @param fileURI save file location
   * @param f_i frequency to consider in the propagation model
   */
  void PrintRecPower(std::string fileURI, double f_i, double txtPower);

  /**
   * @brief Plots a probability distribution, conditioned to a received power
   *
   * @param fileURI save file location
   * @param rxPw received power
   * @param f_i frequency to consider in the propagation model
   */
  void PrintPowProb(std::string fileURI, double rxPw, double f_i, double txtPower);

  void PrintPhase(std::string fileURI, double f_i);
  void PrintPhaseProb(std::string fileURI, double phi, double f_i);
  void PrintBothProb(std::string fileURI, double rxPw, double phi, double f_i, double txtPower);

  void PrintMap(std::string savePath);
  void initRefMap(const std::string imageURI);
  void getImage(std::string layerName, std::string fileURI);
  void addTagID(std::string tagID, int i);
  /**
   * @brief Get layer name corresponding to given frequency (Hz.)
   *
   * @param freq_i
   * @return std::string
   */
  std::string getLayerName(double freq_i);
  /////////////////////
  /**
   * returns a range vector FROM start TO stop (included) in step increments
   * @param  start min Value in vector
   * @param  stop  max Value in vector
   * @param  step  increment
   * @return       range vector
   */
  std::vector<double> range(double start, double stop, double step);

  /**
   * returns MT_242025NRHK antenna losses in a plane
   * @param  angleRad spheric coordinate within the plane (radians)
   * @return          dB losses
   */
  double antennaPlaneLoss(double angleRad);

  /**
   * my Sign function...
   * @param  x
   * @return   sign of x
   */
  float sign(float x);

  /**
   * Returns Spherical coordinates
   * @param x          cartesian coordinate x (m.)
   * @param y          cartesian coordinate y (m.)
   * @param r          spheric coordinate r (m.)
   * @param phi        relative azimut [0,2pi) (angle between XY projection and
   * X) (radians)
   */
  void getSphericCoords(double x, double y, double &r, double &phi);

  /**
   * Received signal estimated phase difference with pi ambiguity
   * @param  tag_x       Tag x coord (m.) with respect to antenna
   * @param  tag_y       Tag y coord (m.) with respect to antenna
   * @param  freq        Transmission frequency (Hertzs)
   * @return      phase difference (radians)
   */
  double phaseDifference(double tag_x, double tag_y, double freq);

  Eigen::MatrixXf getFriisMat(double x_m, double y_m, double orientation_deg,
                              double freq, double txtPower);
  Eigen::MatrixXf getFriisMatFast(double x_m, double y_m,
                                  double orientation_deg, double freq, double txtPower);

  Eigen::MatrixXf getPhaseMat(double x_m, double y_m, double orientation_deg,
                              double freq);
  Eigen::MatrixXf getProbCond(Eigen::MatrixXf X_mat, double x, double sig);
  /**
   * @param  x             x coord (m.) in map coords of the center
   * @param  y             y coord (m.) in map coords of the center
   * @param  orientation   orientation (rad.) in map coords of the center
   * @param  tag_i         tag we want to read
   */
  double getTotalWeight(double x, double y, double orientation, int tag_i);

  double getTotalWeight(double x, double y, double orientation, double size_x,
                        double size_y, int tag_i);

  double getTotalWeight(grid_map::SubmapIterator iterator, int tag_i);

  double getTotalWeight(grid_map::PolygonIterator iterator, int tag_i);
  double getTotalWeight(int tag_i);

  void addMeasurement(double x, double y, double orientation, double rxPower,
                      double phase, double freq, int i, double txtPower);
  
  std::string getPowLayerName(double freq_i);

  std::string getPhaseLayerName(double freq_i);

  std::string getTagLayerName(int tag_num);

  /**
   * Get received power from an OMNIDIRECTIONAL tag,
   * given its relative position to antenna.
   * We assume antenna at 0,0,0, facing X coordinate.
   * See http://www.antenna-theory.com/basics/friis.php
   * Sensitivity is -85 dBm / -115 dB
   *
   * @param  tag_r       Tag r coord (m.) with respect to antenna
   * @param  tag_h       Tag h coord (rad.) with respect to antenna
   * @param  freq        Transmission frequency (Hertzs)
   * @param  txtPower    Transmitted power (dB)
   * @param antennaGainsModel   Antena Gain model
   * @return             Received power (dB)
   */
  double received_power_friis_polar(double tag_r, double tag_h, double freq,
                                    double txtPower,
                                    SplineFunction antennaGainsModel);

  void getImageDebug(GridMap *gm, std::string layerName, std::string fileURI);
  Eigen::MatrixXf getPowProbCond(double rxPw, double f_i, double txtPower);
  Eigen::MatrixXf getPhaseProbCond(double ph_i, double f_i);
  Eigen::MatrixXf getProbCond(std::string layer_i, double x, double sig);

  void saveProbMapDebug(std::string savePATH, int tag_num, int step,
                        double robot_x, double robot_y, double robot_head);
  void createTempProbLayer(Eigen::MatrixXf prob_mat, double x_m, double y_m,
                           double orientation_deg);
  cv::Mat rfidBeliefToCVImg(std::string layer_i);
  void getImage(GridMap *gm, std::string layerName, std::string fileURI);
  Position getRelPoint(Position glob_point, double x_m, double y_m,
                       double orientation_rad);
  Eigen::MatrixXf getIntervProb(std::string layer_i, double x, double sigm);
  void fillFriisMat(Eigen::MatrixXf *rxPw_mat, Eigen::MatrixXf *delay_mat,
                    double freq_i, double offset);

  void overlayRobotPose(GridMap *gm, double robot_x, double robot_y, double robot_head,
                        cv::Mat &image);
  void overlayRobotPose(double robot_x, double robot_y, double robot_head,
                        cv::Mat &image);
                        
  void overlayRobotPoseT(double robot_x, double robot_y, double robot_head,
                         cv::Mat &image);
  void rotatePoints(cv::Point *points, int npts, int cxi, int cyi, double ang);
  void clearObstacles(cv::Mat &image);

  Eigen::MatrixXf getProbCondG(std::string layer_i, double x, double sig);
  Eigen::MatrixXf getProbCondLogN(std::string layer_i, double x, double sig);

  void PrintProb(std::string fileURI, Eigen::MatrixXf *prob_mat, double sX,
                 double sY, double res);
  void saveProbMaps(std::string savePath);

  grid_map::Position fromPoint(cv::Point cvp);
  grid_map::Polygon getActiveMapEdges(double robot_x, double robot_y,
                                      double robot_head);

  void PrintProb(std::string fileURI, Eigen::MatrixXf *prob_mat);

  std::pair<int, std::pair<int, int>> findTagFromBeliefMap(int num_tag);

  void normalizeRFIDLayer(std::string layerName);
  void clearObstacleCellsRFIDLayer(std::string layerName);
  void addTagLayer(int tagNum);


  double getNormalizingFactorBayesRFIDActiveArea(double x_m, double y_m,
                                                 double orientation_rad,
                                                 string tagLayerName);
  Eigen::MatrixXf getNegProb(std::string layer_i, double sensitivity,
                             double sigm);

  void debugInfo();
  void debugInfo(GridMap *gm, std::string mapName,std::string baseLayer);

  cv::Point getPoint(double x, double y);
  cv::Point getPoint(  GridMap* gm, double x_m, double y_m);

  void overlayActiveMapEdges(double robot_x, double robot_y, double robot_head,
                             cv::Mat image);
  void overlayMapEdges(cv::Mat image);

  /**
   * Calculate the KL-divergence between posterior and prior distribution over
   * tags position
   *
   * @param x: x-coord (m.) of the center
   * @param y: y-coord (m.) of the center
   * @param orientation: orientation (rad.) in map coords of the center
   * @param size_x: size-x of the active area
   * @param size_y: size-y of the active area
   * @param tag_i: id of the tag emitting the signal
   */
  double getTotalKL(double x, double y, double orientation, double size_x,
                    double size_y, int tag_i);
  /**
   * Calculate the KL-divergence between posterior and prior distribution over
   * tags position
   *
   * @param x: x-coord (m.) of the center
   * @param y: y-coord (m.) of the center
   * @param orientation: orientation (rad.) in map coords of the center
   * @param iterator: iterator over a submap
   * @param tag_i: id of the tag emitting the signal
   */
  double getTotalKL(double x, double y, double orientation,
                    grid_map::SubmapIterator iterator, int tag_i);
  /**
   * Calculate the entropy of the tag position over the map
   *
   * @param x: x-coord (m.) of the center
   * @param y: y-coord (m.) of the center
   * @param orientation: orientation (rad.) in map coords of the center
   * @param size_x: size-x of the active area
   * @param size_y: size-y of the active area
   * @param tag_i: id of the tag emitting the signal
   */
  double getTotalEntropy(double x, double y, double orientation, double size_x,
                         double size_y, int tag_i);
  /**
   * Calculate the entropy of the tag position over the map
   *
   * @param x: x-coord (m.) of the center
   * @param y: y-coord (m.) of the center
   * @param orientation: orientation (rad.) in map coords of the center
   * @param iterator: iterator over a submap
   * @param tag_i: id of the tag emitting the signal
   */
  double getTotalEntropy(double x, double y, double orientation,
                         grid_map::SubmapIterator iterator, int tag_i);
  /**
   * Calculate the entropy of the tag position over the map
   *
   * @param target: the robot pose
   * @param maxX: distance from one focal distance to the further edge
   * @param minX: distance from one focal distance to the clostest edge
   * @param tag_i: id of the tag emitting the signal
   */
  double getTotalEntropyEllipse(double x, double y, double h_rad, double maxX, double minX,
                                int tag_i);
  /**
   * Calculate the entropy of the tag position over the map
   *
   * @param target: the robot pose
   * @param iterator: iterator over an ellipse
   * @param tag_i: id of the tag emitting the signal
   */
  double getTotalEntropyEllipse(grid_map::EllipseIterator iterator,
                                int tag_i);

  void printEllipse(double x, double y, double orient_rad, double maxX, double minX);

  void addTmpMeasurementRFIDCriterion(double x, double y, double orientation,
                                      double rxPower, double phase, double freq,
                                      int i, double len_update);
  Eigen::MatrixXf getPowProbCondRFIDCriterion(double rxPw, double f_i);
  Eigen::MatrixXf getNegProbRFIDCriterion(double sensitivity, double sigm);
  void createTempProbLayerRFIDCriterion(Eigen::MatrixXf prob_mat, double x_m,
                                        double y_m, double orientation_deg,
                                        double len_update);
  grid_map::Polygon getSubMapEdges(double robot_x, double robot_y,
                                   double robot_head, double len);
  // Position getSubMapRelPoint(Position glob_point, double x_m, double  y_m,
  // double orientation_rad, double len);

  template <typename Scalar>
  void meshgrid(const Eigen::Matrix<Scalar, -1, 1> &x,
                const Eigen::Matrix<Scalar, -1, 1> &y,
                Eigen::Matrix<Scalar, -1, -1> &X,
                Eigen::Matrix<Scalar, -1, -1> &Y);

  template <typename Scalar>
  void meshgrid(const Eigen::Matrix<Scalar, 1, -1> &x,
                const Eigen::Matrix<Scalar, 1, -1> &y,
                Eigen::Matrix<Scalar, -1, -1> &X,
                Eigen::Matrix<Scalar, -1, -1> &Y);

  void addLossesTillEdgeLine(grid_map::Index edge_index_start,
                             grid_map::Index edge_index_end,
                             grid_map::Index antenna_index);
  bool useFast = true;
  Eigen::MatrixXf getFriisMatSlow(double x_m, double y_m,
                                  double orientation_deg, double freq, double txtPower);
  Eigen::MatrixXf getPhaseProbCond(double ph_i, double x_m, double y_m,
                                   double orientation_deg, double f_i);
  Eigen::MatrixXf getPowProbCond(double rxPw, double x_m, double y_m,
                                 double orientation_deg, double f_i, double txtPower);

  void PrintRecPower(std::string fileURI, double x_m, double y_m,
                     double orientation_deg, double f_i, double txtPower);
  void PrintPhase(std::string fileURI, double x_m, double y_m,
                  double orientation_deg, double f_i);
  void PrintPowProb(std::string fileURI, double rxPw, double x_m, double y_m,
                    double orientation_deg, double f_i, double txtPower);
  void PrintPhaseProb(std::string fileURI, double phi, double x_m, double y_m,
                      double orientation_deg, double f_i);
  void PrintBothProb(std::string fileURI, double rxPw, double phi, double x_m,
                     double y_m, double orientation_deg, double f_i, double txtPower);
  cv::Mat layerToImage(GridMap *gm, std::string layerName);

}; // end class

#endif