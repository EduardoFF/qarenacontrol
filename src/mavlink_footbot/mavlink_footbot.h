#ifndef MAVLINK_FOOTBOT_H
#define MAVLINK_FOOTBOT_H

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#endif

#include <map>
#include <vector>
#include <mavlink_types.h>



class MAVLinkFootbot
{
  // ROUTINES FOR CHANGING ONBOARD PARAMETERS
  // All paramaters are sent as type (float) 
  // So paramaters have to be converted between type (float) and their normal representation.
  // An explanation of the MAVLink protocol for changing paramaters can be found at:
  // http://www.qgroundcontrol.org/parameter_interface

  struct mavlink_parameter 
    {       
    char name[15] ;  				/// Name that will be displayed in the GCS
    float min ;        				/// Minimum allowed (float) value for parameter
    float max ;        				/// Maximum allowed (float) value for parameter
    float (*value)(void *, unsigned char) ;  	/// Routine to send parameter to GCS after converting to float.
    uint8_t (*type)(void *,unsigned char) ;    /// Routine to convert from float to local type and set
    void (*setvalue)(void *,float, unsigned char) ;    /// Routine to convert from float to local type and set
    char readonly ; 				/// Parameter is readonly (true) or Read / Write (false)
  };                         

  struct mavlink_data_stream
  {
    uint8_t  stream_id;
    uint16_t stream_rate;
    bool     streaming;
  };
  int m_mytestval;
  std::vector< mavlink_parameter > mavlink_parameters_list;
  std::map< uint8_t, mavlink_data_stream > mavlink_data_stream_by_id;

  static float mavlink_param_generic_integer_value( void *, unsigned char i ) ;
  static uint8_t mavlink_param_generic_integer_type(void *, unsigned char i ) ;
  static void mavlink_param_generic_integer_setvalue(void *, float, unsigned char i ) ;

  bool m_isRunning;
  pthread_t m_threadIn, m_threadOut;

  struct sockaddr_in m_serverAddr; 
  struct sockaddr_in m_localAddr;
  int m_sock;

  mavlink_system_t m_system;
 
  /// Threads shared memory
  int mavlink_send_variables;
  int mavlink_send_variables_counter;
  int mavlink_send_specific_variable;
  int mavlink_send_by_index;

  std::string m_serverIp;
  uint8_t mavlinkCheckTarget(uint8_t sysid, uint8_t compid);
  void handleMessage( mavlink_message_t *);
  static void *MAVLinkOutput40Hz(void *arg);
  static void *MAVLinkInput(void *arg);
  void initializeSocket();
  void closeSocket();
  public:
  MAVLinkFootbot();


  /// basic interface
  bool start();
  bool stop();
};
#endif
