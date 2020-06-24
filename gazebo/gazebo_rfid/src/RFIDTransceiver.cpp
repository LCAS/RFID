
#include <boost/algorithm/string.hpp>
#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/SensorFactory.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

#include "gazebo_rfid/RFIDTransceiver.h"

using namespace gazebo;
using namespace sensors;

/////////////////////////////////////////////////
// constants definiton

// Frequency Quantization in M6e Hardware Guide
const double RFIDTransceiver::STEP_FREQ = 25e3;   // Hertzs
const double RFIDTransceiver::MIN_FREQ_A = 865e6; // Hertzs
const double RFIDTransceiver::MAX_FREQ_A = 869e6; // Hertzs
const int RFIDTransceiver::N_CHAN_A = 160; // 
const double RFIDTransceiver::MIN_FREQ_B = 902e6; // Hertzs
const double RFIDTransceiver::MAX_FREQ_B = 928e6; // Hertzs
const int RFIDTransceiver::N_CHAN_B = 1040; // 

// EU region
const double RFIDTransceiver::MIN_FREQ_EU = 865.6e6; // Hertzs
const double RFIDTransceiver::MAX_FREQ_EU = 867.6e6; // Hertzs
const double RFIDTransceiver::STEP_FREQ_EU = 100e3;  // Hertzs
const int RFIDTransceiver::N_CHAN_EU = 20; // 

// NA region
const double RFIDTransceiver::MIN_FREQ_NA = 902e6;  // Hertzs
const double RFIDTransceiver::MAX_FREQ_NA = 928e6;  // Hertzs
const double RFIDTransceiver::STEP_FREQ_NA = 250e3; // Hertzs
const int RFIDTransceiver::N_CHAN_NA = 104; // 
      
// This comes from the manufacturer. Azimut axis only ...
// gain list entries start at -180 degrees to 180 in steps of 15.
const double RFIDTransceiver::ANTENNA_LOSSES_LIST[25] = {
    -22.6, -25.2, -25,   -20.2, -17.6, -15.6, -14,  -11.2, -7.8,
    -5.2,  -2.4,  -0.6,  0,     -0.6,  -2.4,  -5.2, -8.8,  -12.2,
    -16.4, -19.2, -20.8, -24.4, -28.2, -24,   -22.6};
const double RFIDTransceiver::ANTENNA_ANGLES_LIST[25] = {
    -180.0, -165.0, -150.0, -135.0, -120.0, -105.0, -90.0, -75.0, -60.0,
    -45.0,  -30.0,  -15.0,  0.0,    15.0,   30.0,   45.0,  60.0,  75.0,
    90.0,   105.0,  120.0,  135.0,  150.0,  165.0,  180.0};

// We mostly use UPM frog 3D tag, which is more or less isotropic.
const double RFIDTransceiver::TAG_LOSSES = -4.8;

// Propagation loss constant = 20 * log10 ( c / (4*pi) )
const double RFIDTransceiver::LOSS_CONSTANT = 147.55;

// Constant to obtain propagation delay = 4*pi/c
const double RFIDTransceiver::PHASE_CONSTANT = 4.192e-8;
const double RFIDTransceiver::STD_DEV_DB_NOISE = 3.92; // tipical office: https://www.gaussianwaves.com/2013/09/log-distance-path-loss-or-log-normal-shadowing-model/
const double RFIDTransceiver::STD_DEV_PH_NOISE = 0.01;

// static const std::string fieldSep_ = (":");
// static const std::string entrySep_ = ("\n");

/////////////////////////////////////////////////
RFIDTransceiver::RFIDTransceiver() : Sensor(sensors::OTHER) {
  this->active = false;
}

/////////////////////////////////////////////////
RFIDTransceiver::~RFIDTransceiver() {
}

//////////////////////////////////////////////////
std::string RFIDTransceiver::Topic() const {
  std::string topicName = "~/";
  topicName += this->ParentName() + "/" + this->Name() + "/transceiver";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void RFIDTransceiver::Load(const std::string &_worldName){
  Sensor::Load(_worldName);

  this->parentEntity = boost::dynamic_pointer_cast<physics::Link>( this->world->EntityByName(this->ParentName()));

  GZ_ASSERT(this->parentEntity.lock() != nullptr, "parentEntity is null");

  this->referencePose = this->pose + this->parentEntity.lock()->WorldPose();

  if (!this->sdf->HasElement("transceiver"))  {
    gzthrow("Transceiver sensor is missing <transceiver> SDF element");
  }

}

//////////////////////////////////////////////////
void RFIDTransceiver::Init(){
  Sensor::Init();
}

/////////////////////////////////////////////////
void RFIDTransceiver::Fini(){
  this->pub.reset();
  this->parentEntity.lock().reset();
  Sensor::Fini();
}

