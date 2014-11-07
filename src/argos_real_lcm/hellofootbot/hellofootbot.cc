#include "hellofootbot.h"

HelloFootbot::HelloFootbot() :
		RandomSeed(12345) {

}

/**************************************/

void HelloFootbot::Init(TConfigurationNode& t_tree) {
	// Get actuators and sensors
	wheelsActuator = dynamic_cast<CCI_FootBotWheelsActuator*>(GetRobot().GetActuator("footbot_wheels"));
	ledsActuator = dynamic_cast<CCI_FootBotLedsActuator*>(GetRobot().GetActuator("footbot_leds"));
	//rangeAndBearingActuator = dynamic_cast<CCI_RangeAndBearingActuator*> (GetRobot().GetActuator("range_and_bearing"));
	//beaconActuator          = dynamic_cast<CCI_FootBotBeaconActuator*>   (GetRobot().GetActuator("footbot_beacon"));
	//rangeAndBearingSensor   = dynamic_cast<CCI_RangeAndBearingSensor*>   (GetRobot().GetSensor  ("range_and_bearing"));

	MyId = GetRobot().GetRobotId();
	cout << "Init " << MyId << endl;

	/// Random
	GetNodeAttributeOrDefault(t_tree, "RandomSeed", RandomSeed, RandomSeed);
	CARGoSRandom::CreateCategory("local_rng", RandomSeed);
	RandomGen = CARGoSRandom::CreateRNG("local_rng");

	/* LCM engine */
	lcmHandler = new LCMHandler("udpm://239.255.76.67:7667?ttl=1", "EXAMPLE");

	//ledsActuator->SetAllColors(CColor::GREEN);

	/* LCM Thread */
	lcmThread.setLcmHandler(lcmHandler);
	lcmThread.startInternalThread();
	cout << "End Init " << MyId << endl;
}

void HelloFootbot::ControlStep() {
//	char buf[100];
//	sprintf(buf, "HELLO 2  MESSAGE FROM %s", MyId.c_str());
//	++Steps;
//	if (Steps % 20 == 0) {
//		cout << "Controller steps " << Steps << endl;
//	}

	UInt8 robotIdInt;
	robotIdInt = atoi(GetRobot().GetRobotId().substr(7).c_str());

	//int status = lcmHandler->getAvailableMessagesTimeout(50);

	//printf("Robot id: %s\n", GetRobot().GetRobotId());

	//if (status > 0) {

	//ledsActuator->SetAllColors(CColor::BLUE);

	/* */
	double currentModule = 999999;
	/* */
	CRadians newAngle = CRadians::ZERO;
	CDegrees ceroDegrees;
	ceroDegrees.SetValue(0.0f);
	CVector2 position = CVector2(3, 4.5);

	if (lcmThread.getLcmHandler()->existNode(robotIdInt)) {

		//ledsActuator->SetAllColors(CColor::RED);

		// Get the Node
		node = lcmThread.getLcmHandler()->getNodeById(robotIdInt);

		printf("[CONTROLLER] ID %d - POS: (%f,%f)\n", robotIdInt, node->getPosition().GetZ(), node->getPosition().GetX());

		printf("[CONTROLLER] ID %d - QUAT (w,x,y,z) - (%f,%f,%f,%f)\n", robotIdInt, node->getOrientation().GetW(), node->getOrientation().GetX(), node->getOrientation().GetY(),
				node->getOrientation().GetZ());

		//(w,x,y,z)
		CQuaternion quat(node->getOrientation().GetW(), node->getOrientation().GetZ(), node->getOrientation().GetX(), node->getOrientation().GetY());
//
//		CRadians x;
//		CRadians y;
//		CRadians z;
//		quat.ToEulerAngles(x, y, z);
//
//		printf("[CONTROLLER] ID %d - ORIENTATION EULER: (z,y,x) - (%f,%f,%f)\n", robotIdInt, ToDegrees(z).GetValue(), ToDegrees(y).GetValue(),ToDegrees(z).GetValue());

//		CVector3 axis;
//		CRadians oA;
//		quat.ToAngleAxis(oA, axis);
////		if (axis.GetZ() < 0)
////			oA = -oA;
//

//		double qx = quat.GetX();
//		double qy = quat.GetY();
//		double qz = quat.GetZ();
//		double qw = quat.GetW();

		double qx = node->getOrientation().GetW();
		double qy = node->getOrientation().GetX();
		double qz = node->getOrientation().GetY();
		double qw = node->getOrientation().GetZ();
		double sqw = qw * qw;
		double sqx = qx * qx;
		double sqy = qy * qy;
		double sqz = qz * qz;

		double m[9];
		//invs (inverse square length) is only required if quaternion is not
		//already normalised
		double invs = 1 / (sqx + sqy + sqz + sqw);

		m[0] = (sqx - sqy - sqz + sqw) * invs; // since sqw + sqx + sqy + sqz=1/invs*invs
		m[4] = (-sqx + sqy - sqz + sqw) * invs;
		m[8] = (-sqx - sqy + sqz + sqw) * invs;

		double tmp1 = qx * qy;
		double tmp2 = qz * qw;
		m[3] = 2.0 * (tmp1 + tmp2) * invs;
		m[1] = 2.0 * (tmp1 - tmp2) * invs;

		tmp1 = qx * qz;
		tmp2 = qy * qw;
		m[6] = 2.0 * (tmp1 - tmp2) * invs;
		m[2] = 2.0 * (tmp1 + tmp2) * invs;
		tmp1 = qy * qz;
		tmp2 = qx * qw;
		m[7] = 2.0 * (tmp1 + tmp2) * invs;
		m[5] = 2.0 * (tmp1 - tmp2) * invs;

		double yaw = atan2(-m[6], m[0]);
		double pitch = asin(m[3]);
		double roll = atan2(-m[5], m[4]);

		yaw = yaw * (-1.0);
		//TODO: to see more about quaternions !!!!

		//printf("[CONTROLLER] ID %d - ORIENTATION: (yaw) - (%f)\n", robotIdInt, ToDegrees(yaw).GetValue());
		CRadians yawRad;
		yawRad.SetValue(yaw);
		printf("[CONTROLLER] ID %d - ORIENTATION: (yaw) - (%f)\n", robotIdInt, yaw);
		printf("[CONTROLLER] ID %d - ORIENTATION: (yaw) - (%fº)\n", robotIdInt, ToDegrees(yawRad).GetValue());

		//printf("I received a message - Node id: %d\n", node->getId());

		double xDif = position.GetX() - (node->getPosition().GetZ() / 1000.0);
		double yDif = position.GetY() - (node->getPosition().GetX() / 1000.0);

		printf("AN %d - Previous (%f,%f) - New (%f,%f)\n", robotIdInt, node->getPosition().GetZ(), node->getPosition().GetX(), position.GetX(), position.GetY());
		printf("AN %d - diff (%f,%f)\n", robotIdInt, xDif, yDif);

		newAngle.SetValue(ATan2(yDif, xDif).GetValue());
		//newAngle.UnsignedNormalize();
		//yawRad.UnsignedNormalize();

		if (newAngle.GetValue() < CRadians::ZERO.GetValue()) {
			newAngle.SetValue(newAngle.GetValue() + CRadians::TWO_PI.GetValue());
		}

		if (yawRad.GetValue() < CRadians::ZERO.GetValue()) {
			yawRad.SetValue(yawRad.GetValue() + CRadians::TWO_PI.GetValue());
		}

		currentModule = std::sqrt(xDif * xDif + yDif * yDif);

		int turningOrGo = SetWheelSpeeds(yawRad, currentModule, newAngle, wheelsActuator);

		turningOrGo == 0 ? printf("Robot %d is turning.\n", robotIdInt) : printf("Robot %d is going ahead.\n", robotIdInt);

	} else {

	}
//	} else if (status == 0) {
//
//	}

}

int HelloFootbot::SetWheelSpeeds(const CRadians previousAngle, const Real currentLength, CRadians currentAngle, CCI_FootBotWheelsActuator * m_pcRuedas) {

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

	Real maxAngularVelocity = 1.0f;

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

//	/* Get readings from proximity sensor */
//	const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
//	/* Sum them together */
//	CVector2 cAccumulator;
//	for (size_t i = 0; i < tProxReads.size(); ++i) {
//		cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
//		//printf("Proximity, distance %f, angle %f \n",tProxReads[i].Value,tProxReads[i].Angle);
//	}
//	//printf("Suma: %f,%f\n", cAccumulator[0],cAccumulator[1]);
//	cAccumulator /= tProxReads.size();
//	/* If the angle of the vector is small enough and the closest obstacle is far enough,
//	 continue going straight, otherwise curve a little */
//	CRadians cAngle = cAccumulator.Angle();
//	if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) && cAccumulator.Length() < m_fDelta) {
//		/* Finally, set the wheel speeds if no obstacles */
//		m_pcRuedas->SetLinearVelocity(leftVelocity, rightVelocity);
//	} else {
//		/* Turn, depending on the sign of the angle */
//		if (cAngle.GetValue() > 0.0f) {
//			m_pcRuedas->SetLinearVelocity(maxAngularVelocity, 0.0f);
//		} else {
//			m_pcRuedas->SetLinearVelocity(0.0f, maxAngularVelocity);
//		}
//	}

	/* Finally, set the wheel speeds */
	m_pcRuedas->SetLinearVelocity(leftVelocity, rightVelocity);
	return turningOrGo;

}

/**************************************/

void HelloFootbot::Destroy() {
	DEBUG_CONTROLLER("HelloFootbot::Destroy (  )\n");
}

/**************************************/

bool HelloFootbot::IsControllerFinished() const {
	return false;
}

REGISTER_CONTROLLER(HelloFootbot, "hellofootbot");

