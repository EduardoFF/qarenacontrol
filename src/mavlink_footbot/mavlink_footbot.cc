#include <mavlink_footbot.h>

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include <mavlink.h>

#define BUFFER_LENGTH 2000 // just some large value


uint8_t 
MAVLinkFootbot::mavlinkCheckTarget(uint8_t sysid, uint8_t compid)
{
  if (sysid != m_system.sysid)
  {
    return 0;
  }
  else if (compid != m_system.compid)
  {
    //gcs.send_text(SEVERITY_LOW,"component id mismatch");
    return 0; // XXX currently not receiving correct compid from gcs
  }
  else return 1; // no error
}

  void
*MAVLinkFootbot::MAVLinkInput(void *arg)
{
  MAVLinkFootbot *mavl = 
    (MAVLinkFootbot *) arg;
  ssize_t recsize;
  socklen_t fromlen;
  int bytes_sent;
  mavlink_message_t msg;
  uint16_t len;
  int i = 0;

  uint8_t buf[BUFFER_LENGTH];

  for(;;)
  {
    if(!mavl->m_isRunning)
    {
      fprintf(stderr, "goodbye thread\n");
      break;
    }

    /// Now check received messages
    memset(buf, 0, BUFFER_LENGTH);

    recsize = recvfrom(mavl->m_sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&mavl->m_serverAddr, &fromlen);
    if( recsize > 0)
    {
      /// Something received - print out all bytes and parse packet
      mavlink_message_t msg;
      mavlink_status_t status;

      printf("Bytes Received: %d\nDatagram: ", (int)recsize);
      for (i = 0; i < recsize; ++i)
      {
	//temp = buf[i];
	//printf("%02x ", (unsigned char)temp);
	if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
	{
	  // Packet received
	  //printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
	  mavl->handleMessage(&msg);
	}
      }
      //printf("\n");
    }

    //sleep(1);
  }
  pthread_exit(0);
}

  void
*MAVLinkFootbot::MAVLinkOutput40Hz(void *arg)
{
  MAVLinkFootbot *mavl = 
    (MAVLinkFootbot *) arg;


  ssize_t recsize;
  socklen_t fromlen;
  int bytes_sent;
  mavlink_message_t msg;
  uint16_t len;
  int i = 0;

  uint8_t buf[BUFFER_LENGTH];
  uint32_t count_40Hz = 0;

  for(;;)
  {
    if(!mavl->m_isRunning)
    {
      fprintf(stderr, "goodbye thread\n");
      break;
    }

    if( count_40Hz % 80 == 0)
    {
      printf("m_mytestval = %d\n", 
	     mavl->m_mytestval);
      fflush( stdout );
    }

    if( count_40Hz % 40 == 0)
    {
      /// send heartbeat
      mavlink_msg_heartbeat_pack(mavl->m_system.sysid, mavl->m_system.compid, 
				 &msg, mavl->m_system.type, 
				 MAV_AUTOPILOT_GENERIC, 
				 MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
      len = mavlink_msg_to_send_buffer(buf, &msg);
      bytes_sent = sendto(mavl->m_sock, buf, len, 0, (struct sockaddr*)&mavl->m_serverAddr, sizeof(struct sockaddr_in));

      //printf("HEARTBEAT SENT!\n");
      //fflush( stdout );
    }

    if( mavl->mavlink_send_variables == 1 )
    {
      printf("MAVLINK_SENDING VARIABLES\n");
      fflush( stdout );
      if ( mavl->mavlink_send_variables_counter < mavl->mavlink_parameters_list.size() )
      {

	mavlink_parameter &param = mavl->mavlink_parameters_list[mavl->mavlink_send_variables_counter];
	mavlink_msg_param_value_pack(mavl->m_system.sysid, 
				     mavl->m_system.compid, 
				     &msg, 
				     param.name, 
				     param.value(mavl, mavl->mavlink_send_variables_counter), 
				     param.type(mavl, mavl->mavlink_send_variables_counter), 
				     mavl->mavlink_parameters_list.size(),
				     mavl->mavlink_send_variables_counter);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	bytes_sent = 
	  sendto(mavl->m_sock, buf, len, 0, (struct sockaddr*)&mavl->m_serverAddr, sizeof(struct sockaddr_in));

	printf("SENT PARAMETER %d %s %f\n", mavl->mavlink_send_variables_counter,
	       param.name, param.value(mavl, mavl->mavlink_send_variables_counter ));
	fflush( stdout );

	mavl->mavlink_send_variables_counter++ ;
      }
      else 
      {
	mavl->mavlink_send_variables_counter = 0 ;
	mavl->mavlink_send_variables = 0 ;
      }       
    }

    // SEND SPECIFICALLY REQUESTED PARAMETER
    if ( mavl->mavlink_send_specific_variable == 1 )
    {
      if( mavl->mavlink_send_by_index < mavl->mavlink_parameters_list.size())
      {
	mavlink_parameter &param = mavl->mavlink_parameters_list[mavl->mavlink_send_by_index];
	mavlink_msg_param_value_pack(mavl->m_system.sysid, 
				     mavl->m_system.compid, 
				     &msg, 
				     param.name, 
				     param.value(mavl, mavl->mavlink_send_by_index), 
				     param.type(mavl, mavl->mavlink_send_by_index), 
				     mavl->mavlink_parameters_list.size(),
				     mavl->mavlink_send_by_index);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	bytes_sent = 
	  sendto(mavl->m_sock, buf, len, 0, (struct sockaddr*)&mavl->m_serverAddr, sizeof(struct sockaddr_in));

	printf("SENT PARAMETER %d %s %f\n", mavl->mavlink_send_by_index,
	       param.name, param.value(mavl, mavl->mavlink_send_by_index ));
	fflush( stdout );
      }
      else
      {
	fprintf(stderr, "SEND SPECIFIC PARAMETER FAILED - INVALID INDEX\n");
      }

      mavl->mavlink_send_specific_variable = 0 ;
    }    
    count_40Hz++;
    usleep(25000);
  }
  pthread_exit(0);
}

void
MAVLinkFootbot::closeSocket()
{
  close(m_sock);
}


float 
MAVLinkFootbot::mavlink_param_generic_integer_value( void *ptr, unsigned char i ) 
{
  MAVLinkFootbot *mavl = (MAVLinkFootbot *) ptr;
  //(float) mavl->mavlink_parameters_list[i].
  return (float) mavl->m_mytestval;
}

uint8_t
MAVLinkFootbot::mavlink_param_generic_integer_type(void *, unsigned char i ) 
{
  return (uint8_t) MAV_PARAM_TYPE_REAL32;
}

void 
MAVLinkFootbot::mavlink_param_generic_integer_setvalue(void *ptr , float value, unsigned char i ) 
{
  MAVLinkFootbot *mavl = (MAVLinkFootbot *) ptr;
  mavl->m_mytestval = (int) value;
}


void
MAVLinkFootbot::initializeSocket()
{
  m_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

  memset(&m_localAddr, 0, sizeof(m_localAddr));
  m_localAddr.sin_family = AF_INET;
  m_localAddr.sin_addr.s_addr = INADDR_ANY;
  m_localAddr.sin_port = htons(14551);

  /* Bind the socket to port 14551 - standard port for mavlink */
  if (-1 == bind(m_sock,(struct sockaddr *)&m_localAddr, sizeof(struct sockaddr)))
  {
    perror("error bind failed");
    close(m_sock);
    exit(EXIT_FAILURE);
  } 

  /* Attempt to make it non blocking */
  if (fcntl(m_sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
    close(m_sock);
    exit(EXIT_FAILURE);
  }

  memset(&m_serverAddr, 0, sizeof(m_serverAddr));
  m_serverAddr.sin_family = AF_INET;
  m_serverAddr.sin_addr.s_addr = inet_addr(m_serverIp.c_str());
  m_serverAddr.sin_port = htons(14550);
}



MAVLinkFootbot::MAVLinkFootbot():
  m_isRunning(false),
  m_serverIp("127.0.0.1")
{
  m_system.sysid = 27;                   ///< ID 20 for this footbot
  m_system.compid = MAV_COMP_ID_ALL;     ///< The component sending the message is the IMU, it could be also a Linux process
  m_system.type = MAV_TYPE_GROUND_ROVER;   ///< This system is a GROUND ROVER 
 
  mavlink_send_specific_variable = 0;
  mavlink_send_by_index = 0;
  mavlink_send_variables = 0;
  /// initialize dummy parameter
  mavlink_parameter mpar;
  sprintf(mpar.name, "DUMMY_PARAM");
  mpar.min = 0;
  mpar.max = 10;
  mpar.value = &mavlink_param_generic_integer_value;
  mpar.type = &mavlink_param_generic_integer_type;
  mpar.setvalue = &mavlink_param_generic_integer_setvalue;
  mpar.readonly = false;
  mavlink_parameters_list.push_back( mpar );

  m_mytestval = 123;

}

bool
MAVLinkFootbot::stop()
{
  if( m_isRunning )
    return true;
  closeSocket();
  m_isRunning = false;
  /* wait for the second thread to finish */
  if(pthread_join(m_threadIn, NULL)) 
  {
    fprintf(stderr, "Error joining thread\n");
    return 2;
  }
  if(pthread_join(m_threadOut, NULL)) 
  {
    fprintf(stderr, "Error joining thread\n");
    return 2;
  }

  return true;
}


bool
MAVLinkFootbot::start()
{
  if( m_isRunning )
  {
    fprintf(stderr, "MAVLink already running\n");
    return true;
  } 

  m_isRunning = true;
  initializeSocket();
  if( pthread_create(&m_threadIn,NULL,&MAVLinkInput,(void *) this) )
  {
    fprintf(stderr, "Error creating thread\n");
    m_isRunning = false;
    closeSocket();
    return 0;
  }
  if( pthread_create(&m_threadOut,NULL,&MAVLinkOutput40Hz,(void *) this) )
  {
    fprintf(stderr, "Error creating thread\n");
    m_isRunning = false;
    closeSocket();
    return 0;
  }
  return m_isRunning;
}


/// **** --------------------


//#if 0
// Portions of the following code in handlesmessage() are templated off source code written by James Goppert for the
// ArdupilotMega, and are used by his kind permission and also in accordance with the GPS V3 licensing
// of that code.
void MAVLinkFootbot::handleMessage(mavlink_message_t* msg)
  // This is the main routine for taking action against a parsed message from the GCS
{
  // send_text( (const unsigned char*) "Handling message ID ..");
  // mp_mavlink_transmit(( msg->msgid >> 4 ) + 0x30 ) ;
  // mp_mavlink_transmit(( msg->msgid & 0x0f ) + 0x30 ) ;
  // send_text( (unsigned char*) "\r\n");
  printf("handleMessage msgid %d\n", msg->msgid);
  switch (msg->msgid)     
  {
  case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:  
    {
      /// decode
      mavlink_request_data_stream_t packet;
      mavlink_msg_request_data_stream_decode(msg, &packet);
      if (mavlinkCheckTarget(packet.target_system,packet.target_component) == false ) break;

      int freq = 0; // packet frequency

      if (packet.start_stop == 0) freq = 0; // stop sending
      else if (packet.start_stop == 1) freq = packet.req_message_rate; // start sending
      else break;

      mavlink_data_stream dstream;
      dstream.stream_id = packet.req_stream_id;
      dstream.stream_rate = freq;
      dstream.streaming = (packet.start_stop > 0);
      mavlink_data_stream_by_id[ packet.req_stream_id] = dstream;
      break;
    }

  case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
      printf("MAVLINK_MSG_ID_PARAM_REQUEST_LIST\n");
      fflush( stdout );
      // decode
      mavlink_param_request_list_t packet;
      mavlink_msg_param_request_list_decode(msg, &packet);
      if ( mavlinkCheckTarget(packet.target_system, packet.target_component) == true )
      {
	/// Start sending parameters
	mavlink_send_variables = 1 ;
      }else
      {
	printf("CHECK False! %d %d\n", packet.target_system, 
	       packet.target_component);
      }
      break;
    }

  //case MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL:
    //{

      
    //}
    //break;
  //case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT:
    //{
    
    //}
    //break;
  //case MAVLINK_MSG_ID_WAYPOINT_COUNT:
    //{
    
    //}
    //break;
  //case MAVLINK_MSG_ID_WAYPOINT:
    //{
   
    //}
    //break;
  case MAVLINK_MSG_ID_PARAM_SET:
    {
      /// decode
      mavlink_param_set_t packet;
      mavlink_msg_param_set_decode(msg, &packet);
      if (mavlinkCheckTarget(packet.target_system,packet.target_component) == false)
      {
	fprintf(stderr, "failed target system check on parameter set \r\n");
	break;
      }
      else
      {
	/// set parameter
	const char * key = (const char*) packet.param_id;

	printf("TRYING TO SET PARAM %s\n", 
	       packet.param_id);
	fflush( stdout );


	/// iterate known parameters
	unsigned char i = 0 ;
	for(i=0; i< mavlink_parameters_list.size(); i++)
	{
	  
	  /// compare key with parameter name
	  if (!strcmp(key,(const char *) mavlink_parameters_list[i].name))
	  {
	    printf("FOUND Param\n" );
	    mavlink_parameters_list[i].setvalue(this, packet.param_value, i) ;
	    /// After setting parameter, re-send it to GCS as acknowledgement of success.
	    if( mavlink_send_specific_variable == 0 )
	    {
	      mavlink_send_by_index = i ;
	      mavlink_send_specific_variable = 1 ;
	    }
	  }
	}
	
      }
      break;
    } // end case

  } // end switch
} // end handle mavlink

 //#endif
