#include <QVector3D>
#include <QQuaternion>
class RobotPose
{
  public:
    int id;
    QVector3D pos;
    QVector4D ori;
    static void ToEulerAngles(QVector4D, double &, double &, double &);
};


