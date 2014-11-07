#ifndef _HELLOFOOTBOT_H_
#define _HELLOFOOTBOT_H_

#include <iostream>
#include <fstream>

#include <argos2/common/control_interface/ci_controller.h>
#include <argos2/common/utility/logging/argos_log.h>
#include <argos2/common/utility/argos_random.h>
#include <argos2/common/utility/datatypes/color.h>

#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_wheels_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_leds_actuator.h>
#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_beacon_actuator.h>

#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_sensor.h>

#include <argos2/common/utility/argos_random.h>

#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define __USE_GNU
#include <pthread.h>
#undef __USE_GNU
#include <semaphore.h>
#include <termios.h>
#include "includes/lcm/lcmthread.h"
#include <math.h>

using namespace argos;
using namespace std;

#include <argos2/common/utility/datatypes/datatypes.h>

//#include "xbee_test_types.h"

//#include "client_server_common.h"

//#include "CpuTime.cc"

class HelloFootbot: public CCI_Controller {
private:

	CARGoSRandom::CRNG* RandomGen;
	UInt32 RandomSeed;
	std::string MyId;
	UInt64 Steps;

	//! Sensors:
	CCI_RangeAndBearingSensor* rangeAndBearingSensor;

	//! Actuators:
	CCI_FootBotWheelsActuator* wheelsActuator;
	CCI_FootBotLedsActuator* ledsActuator;
	CCI_RangeAndBearingActuator* rangeAndBearingActuator;
	CCI_FootBotBeaconActuator* beaconActuator;

	// Internal functionality:
//  RobotAddressType RobotIdToAddress(const string& id);
//  RobotAddressType RobotIdToAddress(const char *id);

	/* LCM handler */
	LCMHandler * lcmHandler;
	/* Node */
	Node * node;
	/*  */
	LCMThread lcmThread;

public:

	/* Class constructor. */
	HelloFootbot();

	/* Class destructor. */
	virtual ~HelloFootbot() {
	}

	virtual void Init(TConfigurationNode& t_tree);
	int SetWheelSpeeds(const CRadians previousAngle, const Real currentLength, CRadians currentAngle, CCI_FootBotWheelsActuator * m_pcRuedas);

	virtual void ControlStep();
	virtual void Destroy();
	virtual bool IsControllerFinished() const;

	static UInt8 ConvertValueToByte(Real value);
	static Real ConvertByteToValue(UInt8 byte);
};

#endif
