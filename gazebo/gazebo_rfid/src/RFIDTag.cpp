#include <ignition/math/Rand.hh>
#include <ignition/math/Helpers.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

#include "gazebo_rfid/RFIDTagPrivate.h"
#include "gazebo_rfid/RFIDTag.h"

using namespace gazebo;
using namespace sensors;
using namespace physics;

const double RFIDtagPrivate::step = 1.0;


// CUSTOM SENSOR PRELOADER CODE
#include <gazebo/sensors/SensorFactory.hh>

using gazebo::sensors::Sensor;
using gazebo::sensors::SensorFactory;
extern "C"
{
GZ_REGISTER_STATIC_SENSOR("rfid_tag", RFIDtag)
}
// ...........................

namespace gazebo
{
namespace sensors
{

/////////////////////////////////////////////////
RFIDtag::RFIDtag()
: RFIDTransceiver(),
  dataPtr(new RFIDtagPrivate)
{
}

/////////////////////////////////////////////////
RFIDtag::~RFIDtag()
{
}

/////////////////////////////////////////////////
void RFIDtag::Load(const std::string &_worldName)
{
  RFIDTransceiver::Load(_worldName);

  sdf::ElementPtr transceiverElem = this->sdf->GetElement("transceiver");

  this->dataPtr->visualize = this->sdf->Get<bool>("visualize");

  // tag id is the equivalent to essid
  this->dataPtr->id = transceiverElem->Get<std::string>("essid");

  // transceiver has dBm, we use dB
  this->dataPtr->sensitivity = transceiverElem->Get<double>("sensitivity") - 30.0 ;

  // transceiver has dBm, we use dB
  this->dataPtr->gain = transceiverElem->Get<double>("gain") - 30.0 ;

  this->pub = this->node->Advertise<msgs::PropagationGrid>(this->Topic(), 30);
  GZ_ASSERT(this->pub != nullptr,
      "RFIDtagSensor did not get a valid publisher pointer");
}

//////////////////////////////////////////////////
void RFIDtag::Init()
{
  RFIDTransceiver::Init();

  // This ray will be used in SignalStrength() for checking obstacles
  // between the transmitter and a given point.
  this->dataPtr->testRay = boost::dynamic_pointer_cast<RayShape>(
      this->world->Physics()->CreateShape("ray", CollisionPtr()));
}

//////////////////////////////////////////////////
bool RFIDtag::UpdateImpl(const bool /*_force*/){
  this->referencePose = this->pose + this->parentEntity.lock()->WorldPose();

  if ((this->dataPtr->visualize) && (this->dataPtr->hasReaderData)  ){
    msgs::PropagationGrid msg;
    ignition::math::Pose3d pos;
    ignition::math::Pose3d readerPose;
    double strength, phase;
    msgs::PropagationParticle *p;

    // Iterate using a rectangular grid, but only plot points where power is over sensitivity
    double x = this->dataPtr->step;
    double y = this->dataPtr->step;
    do {
      pos.Set(x, y, 0.0, 0, 0, 0);
      readerPose = pos + this->referencePose;

      // For the propagation model assume the receiver antenna has the same
      // gain as the transmitter
      this->SignalStrength("dummy", readerPose, this->dataPtr->txFreq, this->dataPtr->txP, this->dataPtr->txAntennaGainVector,strength,phase );

      if (strength>this->dataPtr->sensitivity){
        // Add a new particle to the grid
        p = msg.add_particle();
        p->set_x(x);
        p->set_y(y);
        p->set_signal_level(strength);
      }
      x += this->dataPtr->step;
      y += this->dataPtr->step;

    } while (strength>this->dataPtr->sensitivity);
    this->pub->Publish(msg);


  }
  return true;
}

/////////////////////////////////////////////////
std::string RFIDtag::ID() const
{
  return this->dataPtr->id;
}

/////////////////////////////////////////////////
double RFIDtag::SignalStrength(
    const std::string _receiverName,
    const ignition::math::Pose3d &_receiver,
    const double _freq,
    const double _txPw,
    const std::vector<double> &_antenaGainVector,
    double &_rxPw,
    double &_phase
    )
{

  std::string entityName="";
  std::string tagName;
  double dist;
  double free_space=0;
  double occ_space=0;
 
  tagName = this->parentName;
  ignition::math::Vector3d end = this->referencePose.Pos();
  ignition::math::Vector3d start = _receiver.Pos();

  // Avoid computing the intersection of coincident points
  // This prevents an assertion in bullet (issue #849)
  if (start == end) {
    end.Z() += 0.00001;
  }
  
  // gzdbg << "Antenna (" << _receiverName << ") is at pose: "<< start << std::endl;
  // gzdbg << "Tag (" << tagName << ") is is at pose: "<< end << std::endl;
  // gzdbg << "Total distance between centers: " << (end - start).Length() << std::endl;

  // Acquire the mutex for avoiding race condition with the physics engine
  boost::recursive_mutex::scoped_lock lock(*(this->world->Physics()->GetPhysicsUpdateMutex()));
  
  // The ray intersects with collision model...
  // Looking for obstacles between start and end points
  ignition::math::Vector3d rayVector = (end - start).Normalize(); 
  double maxDist = (end - start).Length(); 

  bool antennaEdgeFound = false;
  bool tagEdgeFound = false;
  double delta = 0.001;
  std::string prevEntity = "caspa";
  while ( ( maxDist > (occ_space+free_space) ) & (!tagEdgeFound) ) {
        this->dataPtr->testRay->SetPoints(start, end);
        prevEntity = entityName;
        this->dataPtr->testRay->GetIntersection(dist, entityName);        
        tagEdgeFound = tagEdgeFound || (entityName.find(tagName) != std::string::npos);  
        antennaEdgeFound = antennaEdgeFound || (entityName.find(_receiverName) != std::string::npos);                    

        // already found or just found...
        if ((!antennaEdgeFound) || (entityName.find(_receiverName) != std::string::npos)) {
            dist = dist + delta;
        }
        
        // antenna was previously found ...
        if ((antennaEdgeFound) && (entityName.find(_receiverName) == std::string::npos)) {
              if ( ( dist < delta ) || (entityName.compare(prevEntity)==0) ) {
                // move through the obstacle
                dist = dist + delta;
                occ_space += dist;
              } else {
                // free space till obstacle
                free_space += dist;
              }                        
        }   


        // next sampling point 
        start = start + (dist * rayVector);        
  }

  // gzdbg << "Total Free space between antenna and tag is " << free_space << std::endl;      
  // gzdbg << "Total occ space between antenna and tag is " << occ_space << std::endl;      

  // propagation model
  const ignition::math::Pose3d rel_pose = this->referencePose - _receiver;
  double tag_r = rel_pose.Pos().Length();
  // double tag_h = rel_pose.Rot().Yaw() + this->referencePose.Rot().Yaw();
  // gzdbg << std::endl;
  // gzdbg << "Antenna : "<< _receiver.Rot().Yaw() * 180 / M_PI<< std::endl;
  // gzdbg << "          " << _receiver.Rot().Roll() * 180 / M_PI<< std::endl;
  // gzdbg << "          " << _receiver.Rot().Pitch() * 180 / M_PI<< std::endl;

  // gzdbg << "Tag: " << this->referencePose.Rot().Yaw() * 180 / M_PI<< std::endl;
  // gzdbg << "     " << this->referencePose.Rot().Roll() * 180 / M_PI<< std::endl;
  // gzdbg << "     " << this->referencePose.Rot().Pitch() * 180 / M_PI<< std::endl;

  double rel_angle = fmod(_receiver.Rot().Yaw() + 2* M_PI, 2*M_PI);
  double delta_y = _receiver.Pos().Y() - this->referencePose.Pos().Y() ;
  double delta_x = _receiver.Pos().X() - this->referencePose.Pos().X();
  double tag_h = fmod(atan2(delta_y, delta_x) + 2* M_PI, 2*M_PI);
  tag_h = (rel_angle - tag_h) ;
  tag_h = std::abs(tag_h) - M_PI;
  

  double db_noise = ignition::math::Rand::DblNormal(0.0,RFIDTransceiver::STD_DEV_DB_NOISE);
  double ph_noise = ignition::math::Rand::DblNormal(0.0,RFIDTransceiver::STD_DEV_PH_NOISE);

  // propagation delay...
  _phase = PHASE_CONSTANT * _freq * tag_r;
  _phase = fmod(_phase + ph_noise, M_PI);

  _rxPw = _txPw + db_noise;
  double ant1, antL, propL;
  double lambda = common::SpeedOfLight / _freq ;

  // otherwise friis approach does not apply
  // if (tag_r > 2 * lambda) {
    /*
    SIMPLIFICATION!!! TAG is OMNIDIRECTIONAL
    (a.k.a. don't have tag radiation pattern and
    Here they say it's ok
    https://www.hindawi.com/journals/ijap/2013/194145/tab4/
    */

    // relative angle "tag_h" is between -pi, pi 
    // NOTE: when tag and robot looks each other, tag_h = 0. 
    // It must be increased by M_PI because in _antenaGainVector, 
    // the values ranges from (-pi, pi)
    // if (std::abs(tag_h) < 0.01) tag_h = 0.0;
    // tag_h = tag_h * 180/M_PI;
    // gzdbg << tag_h << ", " << tag_h + 180  << ", " << ((tag_h + 180) * 1000.0) << ", " << (int)((tag_h + 180) * 1000.0) << std::endl;
    // tag_h = tag_h * M_PI/180;
    int ang_index = (int) ((tag_h + M_PI) * 1000.0); 
    // int ang_index = (int) ((tag_h) * 1000.0); 
    ant1 = _antenaGainVector.at(ang_index);
    // gzdbg << "Loss: " << ant1 << std::endl;

    antL = this->dataPtr->gain * pow(cos(tag_h), 2) + ant1;
    // propagation losses
    propL = LOSS_CONSTANT - (20 * log10( tag_r * _freq));
    // signal goes from antenna to tag and comes back again, so we double the losses
    _rxPw += 2 * antL + 2 * propL;

    // Each "wall" adds around 3dB losses. A wall is ~15cm thick, then each metter through obstacles adds  (3 * occ_space / 0.15) dB losses
    _rxPw -= 20.0 * occ_space; // db 
  // } else {
  //   _rxPw = _txPw - std::abs(db_noise);
  // }

return 0;
}

}
}