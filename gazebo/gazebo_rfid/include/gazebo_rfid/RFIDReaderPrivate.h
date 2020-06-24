#ifndef _GAZEBO_SENSORS_RFIDREADER_PRIVATE_HH_
#define _GAZEBO_SENSORS_RFIDREADER_PRIVATE_HH_

#include "gazebo_rfid/RFIDTransceiver.h"

using namespace std;

namespace gazebo
{
  namespace sensors
  {

    /// \internal
    /// \brief RFID reader private data
    class RFIDReaderPrivate
    {
      /// \brief Reception low filter frequency (Hz).
      public: double minFreq = RFIDTransceiver::MIN_FREQ_NA;

      /// \brief Reception high filter frequency (Hz).
      public: double maxFreq = RFIDTransceiver::MAX_FREQ_NA;

      /// \brief Radio channel step (Hz).
      public: double stepFreq = RFIDTransceiver::STEP_FREQ_NA;

      /// \brief Number of Radio channels.
      public: int nChan = RFIDTransceiver::N_CHAN_NA;

      /// \brief RFID reader's sensitivity (dB).
      public: double sensitivity = -90.0;

      /// \brief RFID reader's transmitted power (dB).
      public: double power = 0.0; 

      /// \brief RFID reader's antenna Gain vector (dB) list, ranging [-pi,pi] in milirad steps [0,6283]
      public: std::vector<double> antenaGainVector;      
    };
  }
}
#endif
