/* Include the controller definition */
#include "footbot_un.h"
/* Function definitions for XML parsing */
#include <argos2/common/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos2/common/utility/math/vector2.h>

#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/space.h>
#include <argos2/simulator/space/entities/wifi_equipped_entity.h>
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/space/entities/footbot_entity.h>
#include <argos2/simulator/space/entities/embodied_entity.h>

#include <string>

// static emmbers initialization
UInt32 CFootBotUN::sendPackets = 0;
UInt32 CFootBotUN::receivedPackets = 0;

/****************************************/
/****************************************/

CFootBotUN::CFootBotUN() :
		m_pcWheels(NULL), m_pcLEDs(NULL), m_pcProximity(NULL), m_cAlpha(10.0f), m_fDelta(
				0.5f), m_fWheelVelocity(2.5f), m_numberOfGenerators(0.2f), m_generatePacketInterval(
				10), m_cSpace(CSimulator::GetInstance().GetSpace()), m_cGoStraightAngleRange(
				-ToRadians(m_cAlpha), ToRadians(m_cAlpha)) {
}

/****************************************/
/****************************************/

void CFootBotUN::Init(TConfigurationNode& t_node) {
	/*
	 * Get sensor/actuator handles
	 *
	 * The passed string (ex. "footbot_wheels") corresponds to the XML tag of the device
	 * whose handle we want to have. For a list of allowed values, type at the command
	 * prompt:
	 *
	 * $ launch_argos -q actuators
	 *
	 * to have a list of all the possible actuators, or
	 *
	 * $ launch_argos -q sensors
	 *
	 * to have a list of all the possible sensors.
	 *
	 * NOTE: ARGoS creates and initializes actuators and sensors internally, on the basis of
	 *       the lists provided the XML file at the <controllers><footbot_diffusion><actuators>
	 *       and <controllers><footbot_diffusion><sensors> sections. If you forgot to
	 *       list a device in the XML and then you request it here, an error occurs.
	 */
	m_pcWheels =
			dynamic_cast<CCI_FootBotWheelsActuator*>(GetRobot().GetActuator(
					"footbot_wheels"));
	m_pcLEDs = dynamic_cast<CCI_FootBotLedsActuator*>(GetRobot().GetActuator(
			"footbot_leds"));
	m_pcProximity =
			dynamic_cast<CCI_FootBotProximitySensor*>(GetRobot().GetSensor(
					"footbot_proximity"));

	/*
	 * Parse the XML
	 *
	 * The user defines this part. Here, the algorithm accepts three parameters and it's nice to
	 * put them in the XML so we don't have to recompile if we want to try other settings.
	 */
	GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
	m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
	GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
	GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity,
			m_fWheelVelocity);
	GetNodeAttributeOrDefault(t_node, "generators", m_numberOfGenerators,
			m_numberOfGenerators);
	GetNodeAttributeOrDefault(t_node, "interval", m_generatePacketInterval,
			m_generatePacketInterval);

	//////////////////////////////////////////////////////////////
	// Initialize things required by the communications
	//////////////////////////////////////////////////////////////
	m_pcWifiSensor =
			dynamic_cast<CCI_WiFiSensor*>(GetRobot().GetSensor("wifi"));
	m_pcWifiActuator = dynamic_cast<CCI_WiFiActuator*>(GetRobot().GetActuator(
			"wifi"));
	str_Me = GetRobot().GetRobotId();
	m_iSendOrNotSend = 0;

	// How many robots are there ?
	CSpace::TAnyEntityMap& tEntityMap = m_cSpace.GetEntitiesByType(
			"wifi_equipped_entity");
	numberOfRobots = tEntityMap.size();
	printf("%s: There are %d robots [RMAGAN]\n", str_Me.c_str(),
			numberOfRobots);

	// Am I a generator?
	amIaGenerator = false;
	skipFirstSend = true;
	int numGenerators = m_numberOfGenerators * numberOfRobots;
	int myID = atoi(str_Me.c_str() + 3);
	if (myID < numGenerators)
		amIaGenerator = true;
	if (amIaGenerator)
		printf("%s: There are %d generators, I am on of them\n", str_Me.c_str(),
				numGenerators);
	else
		printf("%s: There are %d generators, but not me :-(\n", str_Me.c_str(),
				numGenerators);

	if (amIaGenerator) {
		UInt32 id = CSimulator::GetInstance().GetRNG()->Uniform(
				CRange<UInt32>(numGenerators, numberOfRobots));
		//UInt32 id = myID + numGenerators;
		std::ostringstream str_tmp(ostringstream::out);
		str_tmp << "fb_" << id;
		str_Dest = str_tmp.str();
		printf(" (dest=%s).\n", str_Dest.c_str());
	}

	//////////////////////////////////////////////////////////////
	// end of communications things here
	//////////////////////////////////////////////////////////////

}

/****************************************/
/****************************************/

void CFootBotUN::ControlStep() {
	/* Get readings from proximity sensor */
	const CCI_FootBotProximitySensor::TReadings& tProxReads =
			m_pcProximity->GetReadings();
	/* Sum them together */
	CVector2 cAccumulator;
	for (size_t i = 0; i < tProxReads.size(); ++i) {
		cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
		//printf("Proximity, distance %f, angle %f \n",tProxReads[i].Value,tProxReads[i].Angle);
	}
	//printf("Suma: %f,%f\n", cAccumulator[0],cAccumulator[1]);
	cAccumulator /= tProxReads.size();
	/* If the angle of the vector is small enough and the closest obstacle is far enough,
	 continue going straight, otherwise curve a little */
	CRadians cAngle = cAccumulator.Angle();
	if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle)
			&& cAccumulator.Length() < m_fDelta) {
		/* Go straight */
		m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
	} else {
		/* Turn, depending on the sign of the angle */
		if (cAngle.GetValue() > 0.0f) {
			m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
		} else {
			m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
		}
	}

	m_pcLEDs->SetAllColors(CColor::GREEN);
	/* Set LED colors */
	//m_pcLEDs->SetAllColors(CColor::BLACK);
	//cAngle.UnsignedNormalize();
	//UInt32 unIndex = static_cast<UInt32>(cAngle * 12.0f / CRadians::TWO_PI);
	//m_pcLEDs->SetSingleColor(unIndex, CColor::RED);
	/* Do the communications */
	std::ostringstream str(ostringstream::out);
	if (amIaGenerator) {
		if (m_iSendOrNotSend % m_generatePacketInterval == 0) {
			if (skipFirstSend) {
				skipFirstSend = false;
			} else {
				sendPackets++;
				str << "Hi I'm " << str_Me << " and I say \"Hello " << str_Dest
						<< "\"";
				str << ",Hi I'm " << str_Me << " and I say \"Hello " << str_Dest
						<< "\"";
				m_pcWifiActuator->SendMessageTo(str_Dest, str.str());
				//std::cout << str_Me << " sending\n";
			}
		}
	}

	m_iSendOrNotSend++;

	//searching for the received msgs
	TMessageList t_incomingMsgs;
	m_pcWifiSensor->GetReceivedMessages(t_incomingMsgs);
	for (TMessageList::iterator it = t_incomingMsgs.begin();
			it != t_incomingMsgs.end(); it++) {
		receivedPackets++;
		std::cout << str_Me << " received: " << it->Payload << " from "
				<< it->Sender << " (total received=)" << receivedPackets
				<< std::endl;
	}
}

void CFootBotUN::Destroy() {
	if (str_Me == "an_0") {
		CSimulator& sim = CSimulator::GetInstance();
		//sim.sendPackets = sendPackets;
		//sim.receivedPackets = receivedPackets;
		//sim.avgDelay = 0.0;
		std::cout << str_Me << " DESTROY: received=" << receivedPackets
				<< " out of sended=" << sendPackets << std::endl;
	}
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotUN, "footbot_un_controller")
