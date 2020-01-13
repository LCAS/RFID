#ifndef RADARMODEL_H
#define RADARMODEL_H


#include <grid_map_core/GridMap.hpp>
#include "Eigen/Eigen"  // AFTER GRIDMAP!



#include <Eigen/Core>
#include <unsupported/Eigen/Splines>


#include "grid_map_core/iterators/GridMapIterator.hpp"
#include <iostream>
#include <string>

// Math
#include <math.h>
#include <cmath>
#include <iostream>
#include <vector>

#include <iomanip>


#include "grid_map_cv/grid_map_cv.hpp"

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace grid_map;

// constants ..................................................................
// We mostly use UPM frog 3D.
const double TAG_LOSSES = -4.8;

// 20 * log10 ( c / (4*pi) )
const double LOSS_CONSTANT = 147.55;
// 4*pi/c
const double PHASE_CONSTANT = 4.192e-8;

// This comes from the manufacturer. Azimut
// gain list entries start at -180 degrees to 180 in steps of 15.
const double ANTENNA_LOSSES_LIST [25] = {  -22.6, -25.2, -25, -20.2, -17.6, -15.6, -14, -11.2, -7.8, -5.2, -2.4, -0.6, 0, -0.6, -2.4, -5.2, -8.8, -12.2, -16.4, -19.2, -20.8, -24.4, -28.2, -24, -22.6};
const double ANTENNA_ANGLES_LIST [25] = {-180.0, -165.0, -150.0, -135.0, -120.0, -105.0, -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0, 120.0, 135.0, 150.0, 165.0, 180.0};




// M6e RFID reader Specs
// Minimum required power to identify a tag. Also depends on other factors, but
// this value is a guide
 const double SENSITIVITY = -115; // dB

// Max transmitted power may be limited by the Region regulations
// see Power Requirements  in M6e Hardware Guide
 const double MIN_TX_POWER = -25; // dB
 const double MAX_TX_POWER = 0; // dB
 const double STEP_TX_POWER = 0.5; // dB

// These freqs ARE limited depending on the region regulations
// see Regional Frequency Quantization in M6e Hardware Guide
const double STEP_FREQ = 25e3; // Hertzs
const double MIN_FREQ_A= 865e6; //Hertzs
const double MAX_FREQ_A= 869e6; //Hertzs
const double MIN_FREQ_B= 902e6; //Hertzs
const double MAX_FREQ_B= 928e6; //Hertzs

// most likely we will use EU or NA regions....
const double MIN_FREQ_EU= 865.6e6; //Hertzs
const double MAX_FREQ_EU= 867.6e6; //Hertzs
const double STEP_FREQ_EU = 100e3; // Hertzs

const double MIN_FREQ_NA= 902e6; //Hertzs
const double MAX_FREQ_NA= 928e6; //Hertzs
const double STEP_FREQ_NA = 250e3; // Hertzs

//  ..................................................................

class SplineFunction {
      public:
        SplineFunction();
        SplineFunction(Eigen::VectorXd const &x_vec, Eigen::VectorXd const &y_vec);

        double interpRad(double x) const;
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


class RadarModel
  {
    double _sizeXaa; //  x axis size in active area (m.). Related with number of rows
    double _sizeYaa; //  y axis size in active area (m.). Related with number of cols

    int _Ncol; // number of rows of reference and rfid belief maps (cells)
    int _Nrow; // number of cols of reference and rfid belief maps (cells)
    float _free_space_val; // max value stored in reference map, used as free space marker
    double _resolution; // Active areas, reference, and belief maps resolution (m./cell)
    double _txtPower; // transmitted power by RFID reader (dB)

    // we only model here gaussian noise: flat fadding.
    double _sigma_power;  // noise factor
    double _sigma_phase;  // noise factor
    
    GridMap _active_area_maps;  // active areas. one per phase and power at each frequency.
    GridMap _rfid_belief_maps;  // Prob. beliefs. One layer per tag. Also, one layer with reference map, mostly for tag layout representation.
    //GridMap _ref_map;  
    
    std::vector<std::pair<double,double>> _tags_coords; // tag locations in reference map coords (m.)
    int _numTags;  // rfid tags to consider
    
    std::vector<double> _freqs; // transmission frequencies (Hz.)
    SplineFunction _antenna_gains;  // model for antenna power gain depending on the angle (dB.)

  public:

/**
 * @brief Construct a new RadarModel object. 
 *        It is based upon the link budget equation. See D. M. Dobkin, “The RF in RFID: Passive UHF RFID in Practice”, Elsevier, 2007 
 * 
 * @param nx X axis Size (m.)
 * @param ny Y axis Size (m.)
 * @param resolution  internal grid resolution (m./cell)
 * @param sigma Noise factor standard deviation 
 * @param txtPower Transmited Power (dB)
 * @param freqs Freqs to be considered in model (Hz.)
 * TODO We will use a fixed antenna model, Gaussian noise model and will assume tags are isotropic. This may need to be revisited ...
 */





    RadarModel(const double nx, const double ny,  const double resolution, const double sigma_power, const double sigma_phase, const double txtPower, const std::vector<double> freqs, const std::vector<std::pair<double,double>> tags_coords, const std::string imageFileURI ) ;
    void PrintMap( std::string savePath);
    void initRefMap(const std::string imageURI);
    void getImage(std::string layerName, std::string fileURI);

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
double antennaPlaneLoss(double angleRad );

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
 * @param phi        relative azimut [0,2pi) (angle between XY projection and X) (radians)
 */
void getSphericCoords(double x, double y, double& r, double& phi);

/**
 * Get received power from an OMNIDIRECTIONAL tag,
 * given its relative position to antenna.
 * We assume antenna at 0,0,0, facing X coordinate.
 * See http://www.antenna-theory.com/basics/friis.php
 * Sensitivity is -85 dBm / -115 dB
 *
 * @param  tag_x       Tag x coord (m.) with respect to antenna
 * @param  tag_y       Tag y coord (m.) with respect to antenna
 * @param  freq        Transmission frequency (Hertzs)
 * @param  txtPower    Transmitted power (dB)
 * @return             Received power (dB)
 */
 double received_power_friis(double tag_x, double tag_y, double freq, double txtPower);
 
 double received_power_friis(double tag_x, double tag_y, double freq, double txtPower, SplineFunction antennaGainsModel);

/**
 * Received signal estimated phase difference with pi ambiguity
 * @param  tag_x       Tag x coord (m.) with respect to antenna
 * @param  tag_y       Tag y coord (m.) with respect to antenna
 * @param  freq        Transmission frequency (Hertzs)
 * @return      phase difference (radians)
 */
double phaseDifference(double tag_x, double tag_y, double freq);


/**
 * Get tag detection boundaries for given configuration
 * @param freq        RF signal frequency
 * @param txtPower    Transmitted RF power (dB.)
 * @param sensitivity Minimun power the reader can receive (dB.)
 * @param distStep    distance step for internal power calculations
 * @param minX        minimum X distance where rx power is over sensitivity
 * @param minY        minimum Y distance where rx power is over sensitivity
 * @param maxX        maximum X distance where rx power is over sensitivity
 * @param maxY        maximum Y distance where rx power is over sensitivity
 */
void activeAreaFriis(double freq, double txtPower, double sensitivity, double distStep, double& minX, double& minY, double& maxX, double& maxY);


void addMeasurement(double x, double y, double orientation, double rxPower, double phase, double freq, int i);

std::string getPowLayerName(double freq_i);

std::string getPhaseLayerName(double freq_i);

std::string getTagLayerName(int tag_num);

double received_power_friis_polar(double tag_r, double tag_h, double freq, double txtPower, SplineFunction antennaGainsModel);

void getImageDebug(std::string layerName, std::string fileURI);
Eigen::MatrixXf getPowProbCond(double rxPw, double f_i);
Eigen::MatrixXf  getPhaseProbCond(double ph_i, double f_i);
Eigen::MatrixXf  getProbCond(std::string layer_i, double x, double sig);

void saveProbMapDebug(std::string savePATH, int tag_num, int step, double robot_x, double robot_y, double robot_head);


void getImage(GridMap* gm,std::string layerName, std::string fileURI);

void PrintRefMapWithTags(std::string fileURI);

void PrintPowProb(std::string fileURI, double rxPw, double f_i);
void PrintPhaseProb(std::string fileURI, double phi, double f_i);
void PrintBothProb(std::string fileURI, double rxPw, double phi, double f_i);


void PrintProb(std::string fileURI, Eigen::MatrixXf* prob_mat,  double sX, double sY, double res);
void saveProbMaps(std::string savePath);

void PrintProb(std::string fileURI, Eigen::MatrixXf* prob_mat);


void debugInfo();

}; // end class



#endif