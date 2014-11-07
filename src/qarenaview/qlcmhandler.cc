#include "qlcmhandler.h"

QLcmHandler::QLcmHandler()
{
    m_lcm = new lcm::LCM();

    if(!m_lcm->good())
    {
     ///
    }

    m_lcm->subscribe("EXAMPLE", &QLcmHandler::handleMessage, this);

}

void
QLcmHandler::run()
{
    while(0 == m_lcm->handle());
}
  void
QLcmHandler::handleMessage(const lcm::ReceiveBuffer* rbuf,
		       const std::string& chan, 
		       const poselcm::pose_list_t* msg)

{
  int i;
  //printf("Received message on channel \"%s\":\n", chan.c_str());
  //printf("  timestamp   = %lld\n", (long long)msg->timestamp);
  //printf("  poses: %d", msg->n);
  for(i = 0; i < msg->n; i++)
  {
    const poselcm::pose_t &pose = msg->poses[i];
    //printf("  robotid = %d\n", pose.robotid);
    //printf("  position    = (%d, %d, %d)\n",
	   //pose.position[0], pose.position[1], pose.position[2]);
    //printf("  orientation = (%d, %d, %d, %d)\n",
	   //pose.orientation[0], pose.orientation[1], 
	   //pose.orientation[2], pose.orientation[3]);
    RobotPose *rpose = new RobotPose;
    rpose->pos = QVector3D(pose.position[2],
			 pose.position[0],
			 pose.position[1]);
    /// (w,x,y,z) -> (x,y,z,w)
    rpose->ori = QVector4D(pose.orientation[1]/10000.0,
			   pose.orientation[2]/10000.0,
			   pose.orientation[3]/10000.0,
			   pose.orientation[0]/10000.0);
    //QVector4D qv = rpose->ori;
    //printf(" quaternion %f %f %f %f\n", 
	   //qv.x(), qv.y(), qv.z(), qv.w());
    rpose->id = pose.robotid;
    emit poseReceived(rpose);



  }
  printf("\n");
}
