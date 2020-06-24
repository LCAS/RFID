#ifndef _GAZEBO_SENSORS_RFIDTAG_PRIVATE_HH_
#define _GAZEBO_SENSORS_RFIDTAG_PRIVATE_HH_

#include <string>
#include <gazebo/physics/PhysicsTypes.hh>

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Wireless transmitter private data
    class RFIDtagPrivate
    {

      /// \brief Size of the grid used for visualization.
      public: static const double step;

      // \brief When true it will publish the propagation grid to be used
      // by the transmitter visual layer
      public: bool visualize = false;

      /// \brief Tag id is the equivalent to essid
      public: std::string id = "1";

      /// \brief flag to indicate we have Readers data.
      public: bool hasReaderData = false;

      /// \brief RFID reader's sensitivity (dB). Used to limit propagation grid
      public: double sensitivity = -115.0;

      /// \brief RFID Tag antennta gain (dB).
      // We mostly use UPM frog 3D tag, which is more or less isotropic.
      public: double gain = -4.8;

      /// \brief RFID reader's transmission frequency (Hz).
      public: double txFreq = 902e6;

      /// \brief RFID reader's transmitted power (dB).
      public: double txP = 0.0;

      /// \brief RFID reader's antenna Gain vector (dB) list, ranging [-pi,pi] in milirad steps [0,6283]
      public: std::vector<double> txAntennaGainVector; 

      // \brief Ray used to test for collisions when placing entities
      public: physics::RayShapePtr testRay;

    };
  }
}
#endif
