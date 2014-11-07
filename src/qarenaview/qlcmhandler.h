#include <QtCore>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/lcmmsgs.hpp"
#include "robotpose.h"
class QLcmHandler: public QThread 
{
  Q_OBJECT
  private:
    void run();
    lcm::LCM *m_lcm;
  public:
    QLcmHandler();
    ~QLcmHandler() {}

    void handleMessage(const lcm::ReceiveBuffer* rbuf,
		       const std::string& chan, 
		       const poselcm::pose_list_t* msg);
signals:
    void poseReceived(RobotPose *);
};

