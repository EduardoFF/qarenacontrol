/* Include the controller definition */
#include "footbot_central.h"

#include <argos2/common/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos2/common/utility/math/vector2.h>

#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/space.h>
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/space/entities/footbot_entity.h>
#include <argos2/simulator/space/entities/embodied_entity.h>
#include <argos2/simulator/space/entities/wifi_equipped_entity.h>
#include <argos2/common/utility/math/vector2.h>
#include <argos2/common/utility/math/general.h>
#include <string>
#include <vector>
#include <map>

/****************************************/
/****************************************/

CFootBotCentral::CFootBotCentral() :
		m_pcWheels(NULL), m_pcLEDs(NULL), m_pcProximity(NULL), m_cAlpha(10.0f), m_fDelta(0.5f), m_fWheelVelocity(2.5f), m_cSpace(CSimulator::GetInstance().GetSpace()), m_cGoStraightAngleRange(
				-ToRadians(m_cAlpha), ToRadians(m_cAlpha)) {
}

/****************************************/
/****************************************/

void CFootBotCentral::Init(TConfigurationNode& t_node) {

	m_pcWheels = dynamic_cast<CCI_FootBotWheelsActuator*>(GetRobot().GetActuator("footbot_wheels"));
	m_pcLEDs = dynamic_cast<CCI_FootBotLedsActuator*>(GetRobot().GetActuator("footbot_leds"));
	m_pcProximity = dynamic_cast<CCI_FootBotProximitySensor*>(GetRobot().GetSensor("footbot_proximity"));

	/*
	 * Parse the XML
	 *
	 * The user defines this part. Here, the algorithm accepts three parameters and it's nice to
	 * put them in the XML so we don't have to recompile if we want to try other settings.
	 */
	GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
	m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
	GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
	GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

	/* LCM engine */
	lcmHandler = new LCMHandler("udpm://239.255.76.67:7667?ttl=1", "EXAMPLE");

	/* LCM Thread */
	lcmThread.setLcmHandler(lcmHandler);
	lcmThread.startInternalThread();

	/* Init current angle and current module -- Only for LCM testing purposes */
	CFootBotEntity *robotEntityAN = static_cast<CFootBotEntity*>(&m_cSpace.GetEntity(GetRobot().GetRobotId()));
	cFootBotcurrent = robotEntityAN->GetEmbodiedEntity().GetPosition();
	CQuaternion cFootBotOrientation = robotEntityAN->GetEmbodiedEntity().GetOrientation();

	CVector3 axis;
	CRadians oA;
	cFootBotOrientation.ToAngleAxis(oA, axis);
	if (axis.GetZ() < 0)
		oA = -oA;

	newAngle = oA;
}

/****************************************/
/****************************************/

void CFootBotCentral::ControlStep() {

	CFootBotEntity *robotEntityAN = static_cast<CFootBotEntity*>(&m_cSpace.GetEntity(GetRobot().GetRobotId()));
	CVector3 cFootBotPosition = robotEntityAN->GetEmbodiedEntity().GetPosition();
	CQuaternion cFootBotOrientation = robotEntityAN->GetEmbodiedEntity().GetOrientation();

	CVector3 axis;
	CRadians oA;
	cFootBotOrientation.ToAngleAxis(oA, axis);
	if (axis.GetZ() < 0)
		oA = -oA;

	CRadians x;
	CRadians y;
	CRadians z;
	cFootBotOrientation.ToEulerAngles(z,y,x);
	printf("QUAT (w,x,y,z) - (%f,%f,%f,%f)\n", cFootBotOrientation.GetW(), cFootBotOrientation.GetX(), cFootBotOrientation.GetY(), cFootBotOrientation.GetZ());
	printf("ORIENTATION EULER: (z,y,x) - (%f,%f,%f)\n", ToDegrees(z), ToDegrees(y), ToDegrees(x));

	CVector2 position = CVector2(cFootBotPosition.GetX(), cFootBotPosition.GetY());
	CRadians previousAngle = oA;

	UInt8 robotIdInt;
	robotIdInt = atoi(GetRobot().GetRobotId().substr(7).c_str());

	//int status = lcmHandler->getAvailableMessagesTimeout(CPhysicsEngine::GetInverseSimulationClockTick() / 2);

	//if (status > 0) {

	// If there a message for the robot?
	if (lcmThread.getLcmHandler()->existNode(robotIdInt)) {

		// Get the Node
		node = lcmThread.getLcmHandler()->getNodeById(robotIdInt);

		// Move to the destination

		double xDif = node->getPosition().GetX() / 1000 - position.GetX();
		double yDif = node->getPosition().GetY() / 1000 - position.GetY();

		printf("AN %d - Previous (%f,%f) - New (%f,%f)\n", robotIdInt, position.GetX(), position.GetY(), node->getPosition().GetX(), node->getPosition().GetY());
		printf("AN %d - diff (%f,%f)\n", robotIdInt, xDif, yDif);

		newAngle.SetValue(ATan2(yDif, xDif).GetValue());

		currentModule = std::sqrt(xDif * xDif + yDif * yDif);

		cFootBotcurrent.SetX(node->getPosition().GetX() / 1000);
		cFootBotcurrent.SetY(node->getPosition().GetY() / 1000);

	} else {

		//} else if (status == 0) {

		double xDif = cFootBotcurrent.GetX() - position.GetX();
		double yDif = cFootBotcurrent.GetY() - position.GetY();

		printf("AN %d - Previous (%f,%f) - New (%f,%f)\n", robotIdInt, position.GetX(), position.GetY(), cFootBotcurrent.GetX(), cFootBotcurrent.GetY());
		printf("AN %d - diff (%f,%f)\n", robotIdInt, xDif, yDif);

		newAngle.SetValue(ATan2(yDif, xDif).GetValue());

		currentModule = std::sqrt(xDif * xDif + yDif * yDif);

	}

	int turningOrGo = SetWheelSpeeds(previousAngle, currentModule, newAngle, m_pcWheels);

	turningOrGo == 0 ? printf("Robot %d is turning.\n", robotIdInt) : printf("Robot %d is going ahead.\n", robotIdInt);

}

int CFootBotCentral::SetWheelSpeeds(const CRadians previousAngle, const Real currentLength, CRadians currentAngle, CCI_FootBotWheelsActuator * m_pcRuedas) {

	/* Determines if the robot is turning or is going ahead */
	int turningOrGo;

	/* Clamp the speed so that it's not greater than MaxSpeed */
	Real module = Min<Real>(currentLength * 100/*para pasarlo a cm*/, 20);

	Real leftVelocity;
	Real rightVelocity;

	Real deltaOrientation = 5.0f;
	CDegrees deltaOrientationDegrees;
	deltaOrientationDegrees.SetValue(deltaOrientation);

	CDegrees ceroDegrees;
	ceroDegrees.SetValue(0.0f);

	CDegrees piDegrees;
	piDegrees.SetValue(180.0f);

	//CRadians previousAngle = previous.Angle();
	CRadians current = currentAngle;

	CDegrees currentAngleDegrees = ToDegrees(current);

	printf("Heading angle %f\n", ToDegrees(previousAngle).GetValue());
	printf("Current angle %f\n", ToDegrees(current).GetValue());
	printf("Current module %f\n", module);

	Real maxAngularVelocity = 5.0f;

	if (ToDegrees(Abs(previousAngle - current)) < deltaOrientationDegrees || ToDegrees(Abs(previousAngle - current)) == deltaOrientationDegrees) {
		//seguimos rectos el modulo
		leftVelocity = module;
		rightVelocity = module;
		turningOrGo = 1;
	} else {

		if (ToDegrees((previousAngle - current)) > ceroDegrees) {
			//rotamos derecha
			leftVelocity = maxAngularVelocity;
			rightVelocity = -maxAngularVelocity;
			turningOrGo = 0;
		} else if (ToDegrees((previousAngle - current)) < ceroDegrees) {
			//rotamos izq
			leftVelocity = -maxAngularVelocity;
			rightVelocity = maxAngularVelocity;
			turningOrGo = 0;
		} else {
			//seguimos rectos el modulo
			leftVelocity = module;
			rightVelocity = module;
			turningOrGo = 1;
		}
	}

	/* Get readings from proximity sensor */
	const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
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
	if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) && cAccumulator.Length() < m_fDelta) {
		/* Finally, set the wheel speeds if no obstacles */
		m_pcRuedas->SetLinearVelocity(leftVelocity, rightVelocity);
	} else {
		/* Turn, depending on the sign of the angle */
		if (cAngle.GetValue() > 0.0f) {
			m_pcRuedas->SetLinearVelocity(maxAngularVelocity, 0.0f);
		} else {
			m_pcRuedas->SetLinearVelocity(0.0f, maxAngularVelocity);
		}
	}

	/* Finally, set the wheel speeds */
	//m_pcRuedas->SetLinearVelocity(leftVelocity, rightVelocity);
	return turningOrGo;

}

void CFootBotCentral::Destroy() {

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
REGISTER_CONTROLLER(CFootBotCentral, "footbot_central_controller")
