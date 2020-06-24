#ifndef _GAZEBO_SENSORS_RFIDTRANSCEIVER_HH_
#define _GAZEBO_SENSORS_RFIDTRANSCEIVER_HH_

#include <string>
#include <ignition/math/Pose3.hh>

#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class RFIDTransceiver RFIDTransceiver.hh sensors/sensors.hh
    /// \brief Sensor class for receiving RFID signals.
    class GZ_SENSORS_VISIBLE RFIDTransceiver: public Sensor
    {
      // constants declaration:
      public: 
          // Frequency Quantization in M6e Hardware Guide
          static const double STEP_FREQ;   // Hertzs
          static const double MIN_FREQ_A; // Hertzs
          static const double MAX_FREQ_A; // Hertzs
          static const int N_CHAN_A; // 
          static const double MIN_FREQ_B; // Hertzs
          static const double MAX_FREQ_B; // Hertzs
          static const int N_CHAN_B; // 

          // EU region
          static const double MIN_FREQ_EU; // Hertzs
          static const double MAX_FREQ_EU; // Hertzs
          static const double STEP_FREQ_EU;  // Hertzs
          static const int N_CHAN_EU; // 

          // NA region
          static const double MIN_FREQ_NA ;  // Hertzs
          static const double MAX_FREQ_NA ;  // Hertzs
          static const double STEP_FREQ_NA ; // Hertzs
          static const int N_CHAN_NA; // 
                
          // This comes from the manufacturer. Azimut axis only ...
          // gain list entries start at -180 degrees to 180 in steps of 15.
          static const double ANTENNA_LOSSES_LIST[25];
          static const double ANTENNA_ANGLES_LIST[25];

          // We mostly use UPM frog 3D tag, which is more or less isotropic.
          static const double TAG_LOSSES;

          // Propagation loss constant = 20 * log10 ( c / (4*pi) )
          static const double LOSS_CONSTANT;

          // Constant to obtain propagation delay = 4*pi/c
          static const double PHASE_CONSTANT;

          static const double STD_DEV_DB_NOISE;
          static const double STD_DEV_PH_NOISE;

          // used to serialize readings
          // static const std::string fieldSep_;
          // static const std::string entrySep_;

      /// \brief Constructor
      public: RFIDTransceiver();

      /// \brief Constructor
      public: ~RFIDTransceiver();

      // Documentation inherited
      public: virtual std::string Topic() const;

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Publisher to publish propagation model data
      protected: transport::PublisherPtr pub;

      /// \brief Parent entity which the sensor is attached to
      protected: boost::weak_ptr<physics::Link> parentEntity;

      /// \brief Sensor reference pose
      protected: ignition::math::Pose3d referencePose;
    };
    /// \}
  }
}
#endif
