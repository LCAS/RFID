#ifndef _GAZEBO_SENSORS_RFIDTAG_HH_
#define _GAZEBO_SENSORS_RFIDTAG_HH_

#include <memory>
#include <string>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/util/system.hh>

#include "gazebo_rfid/RFIDTransceiver.h"

namespace gazebo
{
  namespace sensors
  {
    // Forward declare private data class.
    class RFIDtagPrivate;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \class RFIDtag RFIDTag.hh sensors/sensors.hh
    /// \brief Transmitter to send wireless signals
    class GZ_SENSORS_VISIBLE RFIDtag: public RFIDTransceiver
    {
      /// \brief Constructor.
      public: RFIDtag();

      /// \brief Destructor.
      public: virtual ~RFIDtag();

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      /// \brief Returns the Service Set Identifier (network name).
      /// \return Service Set Identifier (network name).
      public: std::string ID() const;

      /// \brief Returns reception frequency (MHz).
      /// \return Reception frequency (MHz).
      public: double Freq() const;

      /// \brief Returns the signal strength in a given world's point (dBm).
      /// \param[in] _receiver Pose of the receiver
      /// \param[in] _rxGain Receiver gain value
      /// \return Signal strength in a world's point (dBm).
      public: double SignalStrength( const std::string _receiverName, const ignition::math::Pose3d &_receiver, const double _freq, const double _txPw, const std::vector<double> &_antenaGainVector, double &_rxPw, double &_phase);

      /// \brief Get the std dev of the Gaussian random variable used in the
      /// propagation model.
      /// \return The standard deviation of the propagation model.
      public: double ModelStdDev() const;

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<RFIDtagPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
