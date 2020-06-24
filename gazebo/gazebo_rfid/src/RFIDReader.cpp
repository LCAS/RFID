#include <ignition/math/Pose3.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

#include "gazebo_rfid/RFIDReaderPrivate.h"
#include "gazebo_rfid/RFIDReader.h"
#include "gazebo_rfid/RFIDTag.h"

// mfc: I get an undefined symbol when I use those...
//#include "tag_inventory.pb.h"
//#include "tag_reading.pb.h"

using namespace gazebo;
using namespace sensors;

// CUSTOM SENSOR PRELOADER CODE
#include <gazebo/sensors/SensorFactory.hh>

using gazebo::sensors::Sensor;
using gazebo::sensors::SensorFactory;
extern "C"
{
GZ_REGISTER_STATIC_SENSOR("rfid_reader", RFIDReader)
}
// ...........................

namespace gazebo
{
namespace sensors
{


/////////////////////////////////////////////////
RFIDReader::RFIDReader()
: RFIDTransceiver(),
  dataPtr(new RFIDReaderPrivate)
{
}

//////////////////////////////////////////////////
RFIDReader::~RFIDReader()
{
}

//////////////////////////////////////////////////
void RFIDReader::Init()
{
  RFIDTransceiver::Init();
  gzdbg << "INIT" << std::endl;
}

/////////////////////////////////////////////////
void RFIDReader::Load(const std::string &_worldName){
  RFIDTransceiver::Load(_worldName);

  //this->pub = this->node->Advertise<gazebo_rfid_reader::msgs::TagInventory>(this->Topic(), 30);
  this->pub = this->node->Advertise<msgs::GzString>(this->Topic(), 30);
  GZ_ASSERT(this->pub != nullptr, "RFIDReader did not get a valid publisher pointer");

  sdf::ElementPtr transceiverElem = this->sdf->GetElement("transceiver");
  // transceiver has MHzs, we use Hzs
  this->dataPtr->minFreq = transceiverElem->Get<double>("min_frequency") * 1e6;
  // transceiver has MHzs, we use Hzs
  this->dataPtr->maxFreq = transceiverElem->Get<double>("max_frequency")* 1e6;
  // transceiver has dBm, we use dB
  this->dataPtr->sensitivity = transceiverElem->Get<double>("sensitivity") - 30.0 ;
  // transceiver has dBm, we use dB
  this->dataPtr->power = transceiverElem->Get<double>("power") - 30.0 ;

  if (this->dataPtr->minFreq <= 0){
    gzerr << "RFIDReader min frequency "
      << "is [" << this->dataPtr->minFreq << "]. Value must be > 0, "
      << "using a value of 902e6\n";
    this->dataPtr->minFreq = RFIDTransceiver::MIN_FREQ_NA;
  }

  if (this->dataPtr->maxFreq <= 0){
    gzerr << "RFIDReader max frequency "
      << "of [" << this->dataPtr->maxFreq << "]. Value must be > 0, "
      << "using a value of 928e6\n";
    this->dataPtr->maxFreq = RFIDTransceiver::MAX_FREQ_NA;
  }  

  if (this->dataPtr->minFreq > this->dataPtr->maxFreq){
    gzerr << "RFIDReader min frequency[" << this->dataPtr->minFreq << "] " 
      << "is greater than max frequency[" << this->dataPtr->maxFreq << "]. "
      << "Using N.A. values.\n";
    this->dataPtr->minFreq = RFIDTransceiver::MIN_FREQ_NA;
    this->dataPtr->maxFreq = RFIDTransceiver::MAX_FREQ_NA;
  }

  if ( (this->dataPtr->minFreq == RFIDTransceiver::MIN_FREQ_NA) && (this->dataPtr->maxFreq == RFIDTransceiver::MAX_FREQ_NA) ) {
    this->dataPtr->stepFreq = RFIDTransceiver::STEP_FREQ_NA;
    this->dataPtr->nChan = RFIDTransceiver::N_CHAN_NA;
  } else if ( (this->dataPtr->minFreq == RFIDTransceiver::MIN_FREQ_EU) && (this->dataPtr->maxFreq == RFIDTransceiver::MAX_FREQ_EU) ) {
    this->dataPtr->stepFreq = RFIDTransceiver::STEP_FREQ_EU;
    this->dataPtr->nChan = RFIDTransceiver::N_CHAN_EU;
  } else if ( (this->dataPtr->minFreq == RFIDTransceiver::MIN_FREQ_A) && (this->dataPtr->maxFreq == RFIDTransceiver::MAX_FREQ_A) ) {
    this->dataPtr->stepFreq = RFIDTransceiver::STEP_FREQ;
    this->dataPtr->nChan = RFIDTransceiver::N_CHAN_A;
  } else if ( (this->dataPtr->minFreq == RFIDTransceiver::MIN_FREQ_B) && (this->dataPtr->maxFreq == RFIDTransceiver::MAX_FREQ_B) ) {
    this->dataPtr->stepFreq = RFIDTransceiver::STEP_FREQ;
    this->dataPtr->nChan = RFIDTransceiver::N_CHAN_B;
  } else {
    this->dataPtr->stepFreq = RFIDTransceiver::STEP_FREQ_NA;
    this->dataPtr->nChan = std::floor((this->dataPtr->maxFreq -this->dataPtr->minFreq ) / this->dataPtr->stepFreq );
  }

  BuildAntennaGainCache();
  gzdbg << "LOAD" << std::endl;
}

//////////////////////////////////////////////////
bool RFIDReader::UpdateImpl(const bool /*_force*/)
{  
  std::string tag_id;
  //gazebo_rfid_reader::msgs::TagInventory msg;
  msgs::GzString msg;
  std::ostringstream local;

  double rxPower,phase;
  double txFreq;

  this->referencePose = this->pose + this->parentEntity.lock()->WorldPose();

  ignition::math::Pose3d myPos = this->referencePose;
  Sensor_V sensors = SensorManager::Instance()->GetSensors();
  std::string myName = this->parentName;
  for (Sensor_V::iterator it = sensors.begin(); it != sensors.end(); ++it) {

    if ((*it)->Type() == "rfid_tag"){
      std::shared_ptr<gazebo::sensors::RFIDtag> tag_i = std::static_pointer_cast<RFIDtag>(*it);
      
      // RFID readers randomly chooses freq to query tags
      txFreq = this->GetFreq();
      // get the power from the tag:
      
      tag_i->SignalStrength(myName, myPos, txFreq, this->dataPtr->power, this->dataPtr->antenaGainVector,rxPower,phase);

      // Discard if the received signal strengh is lower than the sensivity
      if (rxPower < this->Sensitivity()) {
        continue;
      }

      static const std::string fieldSep_ = (":");
      static const std::string entrySep_ = ("\n");

      tag_id = tag_i->ID();
      local << (tag_id) << fieldSep_ << (txFreq) << fieldSep_ << (rxPower) << fieldSep_ << (this->dataPtr->power) << fieldSep_ << (phase) << entrySep_;

      /**
      gazebo_rfid_reader::msgs::TagReading *tagReading = msg.add_node();
      tagReading->set_id(tag_id);
      tagReading->set_frequency(txFreq);
      tagReading->set_signal_level(rxPower);
      tagReading->set_tx_power(this->Power());
      tagReading->set_rx_power(rxPower);
      tagReading->set_phase(phase);
      */
    }
  }

  msg.set_data(local.str());
  if (local.str().size() > 0)
  {
    this->pub->Publish(msg);
  }
  return true;
}

/////////////////////////////////////////////////
double RFIDReader::Sensitivity() const {
  return this->dataPtr->sensitivity;
}

/////////////////////////////////////////////////
double RFIDReader::GetFreq() const {
  double freq = this->dataPtr->minFreq;

  // get a random channel
  int channel = ignition::math::Rand::IntUniform(	0,this->dataPtr->nChan);
  freq += channel * this->dataPtr->stepFreq;
  // I think this is not necessary ...
  freq = std::min(freq, this->dataPtr->maxFreq);

  return freq;
}

//////////////////////////////////////////////////
void RFIDReader::Fini() {
  RFIDTransceiver::Fini();
  gzdbg << "Fini" << std::endl;

}

//////////////////////////////////////////////////
void RFIDReader::BuildAntennaGainCache(){
      // build spline to interpolate antenna gains;
      std::vector<double> angVec(ANTENNA_ANGLES_LIST, ANTENNA_ANGLES_LIST + 25);
      std::vector<double> gainVec(ANTENNA_LOSSES_LIST, ANTENNA_LOSSES_LIST + 25);

      double ang_min = *std::min_element(angVec.begin(), angVec.end());
      double ang_max = *std::max_element(angVec.begin(), angVec.end());
      double gain_min = *std::min_element(gainVec.begin(), gainVec.end());
      double gain_max = *std::max_element(gainVec.begin(), gainVec.end());

      // ignition spline is 3d, but we only use 1D ...
      ignition::math::Spline gain_spline;
      gain_spline.AutoCalculate(false);
      gain_spline.Clear();

      for (auto gain_i : gainVec) {
        gain_spline.AddPoint( ignition::math::Vector3d(gain_i, 0, 0) );
      }
      gain_spline.RecalcTangents();
     
      // Instead of interpolaing on each query, we use a cache.
      std::vector<double> antenaGainVector;      
      antenaGainVector.reserve(6284); // [-pi,pi] in steps of 0.001 rads        

      double gain_i, rad_i,deg_i, ang_norm;
      ignition::math::Vector3d gainP;
      for (int milirad_index=0; milirad_index<6284; milirad_index++) {
          rad_i =  (( milirad_index)/1000.0) - M_PI; // from range [0,6283] to [-pi,pi]
          deg_i = rad_i*180.0/M_PI;
          ang_norm = ( deg_i - ang_min) / (ang_max - ang_min);

          gainP = gain_spline.Interpolate( ang_norm );
          gain_i = gainP[0];
          // interpolation may produce values bigger and lower than our gain limits ...
          gain_i = std::max(min(gain_i, gain_max), gain_min);
          antenaGainVector.push_back(gain_i);
      }
      this->dataPtr->antenaGainVector = antenaGainVector;

}

}
}