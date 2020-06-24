#ifndef _GAZEBO_SENSORS_RFIDREADER_HH_
#define _GAZEBO_SENSORS_RFIDREADER_HH_

#include <memory>
#include <string>
#include <sstream>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Spline.hh>

#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/util/system.hh>

#include "gazebo_rfid/RFIDTransceiver.h"

namespace gazebo
{
  namespace sensors
  {
    // Forward declare private data
    class RFIDReaderPrivate;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \class RFIDReader RFIDReader.hh sensors/sensors.hh
    /// \brief Sensor class for receiving RFID tag readings.
    class GZ_SENSORS_VISIBLE RFIDReader: public RFIDTransceiver
    {
      /// \brief Constructor
      public: RFIDReader();

      /// \brief Destructor
      public: virtual ~RFIDReader();

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Returns the receiver sensitivity (dBm).
      /// \return Receiver sensitivity (dBm).
      public: double Sensitivity() const;

      /// \brief Returns a random freq in range (Hz).
      /// \return Random Freq. (Hz).
      public: double GetFreq() const;

      /// \brief Builds the Antenna Gain vector in milirads resolution.
      void BuildAntennaGainCache();

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<RFIDReaderPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
