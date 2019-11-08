#include "RadarModel.hpp"


#include <iostream>
#include <vector>

// quick compile: only depends on eigen and grid_map lib
// g++ -o prog test_log_model.cpp  -std=c++11 -I. -I/opt/ros/kinetic/include/ -I/usr/include/eigen3/ -L/opt/ros/kinetic/lib/ -lgrid_map_core


int main(int argc, char **argv)
{


  double nx = 12;
  double ny = 6;
  double resolution = 0.1;
  double sigma_power = 1;
  double sigma_phase = 1;
  double test_rxp = -10.0;
  double test_phase = 2 * M_PI/ 4.0;

  cout <<"You provided: (" << argc-1 << ") arguments"<<  std::endl;

  if (argc > 7) {
            nx = atoi(argv[1]);
            ny = atoi(argv[2]);
            resolution = atof(argv[3]);
            sigma_power = atof(argv[4]);
            sigma_phase = atof(argv[5]);
            test_rxp = atof(argv[6]);
            test_phase = atof(argv[7])* M_PI/180.0;
      cout <<"Map size: ("<< nx <<","<< ny <<") m." << std::endl;
      cout <<"Resolution: ("<< resolution <<") m./cell" << std::endl;
      cout <<"Power Noise std: ("<< sigma_power <<")" << std::endl;
      cout <<"Phase Noise std: ("<< sigma_phase <<")" << std::endl;
      cout <<"Test rxPw: ("<< test_rxp <<") dB." << std::endl;
      cout <<"Test phase: ("<< atof(argv[6]) <<") deg. ==(" << test_phase <<") rad." << std::endl;
  } else {
      cout <<"DEFAULTS!" << std::endl;
  }

  
  double txtPower = -10;
  cout <<"Test TxPw: ("<< txtPower <<") dB." << std::endl;

  double f_i = 9.1e8;
  std::vector<double> freqs{ 9e8, 9.1e8, 9.2e8 }; 

  RadarModel a(nx, ny, resolution, sigma_power, sigma_phase, txtPower, freqs);
  cout <<"Radar model built." << std::endl;

  a.getImage("P_910.00", "/tmp/Average_power_map_f_910.png");            
  a.getImage("D_910.00", "/tmp/Average_phase_map_f_910.png");  
  a.PrintPowProb("/tmp/prob_rec_power_f_910.png", test_rxp, f_i);          
  a.PrintPhaseProb("/tmp/prob_rec_phase_f_910.png", test_phase, f_i);          
  a.PrintBothProb("/tmp/prob_joint_f_910.png", test_rxp, test_phase, f_i);          
  return 0;
}