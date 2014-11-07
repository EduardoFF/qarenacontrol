#include "robotpose.h"
#include <QVector4D>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <math.h>
void 
RobotPose::ToEulerAngles(QVector4D qv,
			 double &yaw, double &pitch, double &roll)
{ 
  double qx = qv.x();
  double qy = qv.y();
  double qz = qv.z();
  double qw = qv.w();
  double sqw = qw*qw; 
  double sqx = qx*qx; 
  double sqy = qy*qy; 
  double sqz = qz*qz; 

  double m[9];
  //invs (inverse square length) is only required if quaternion is not already normalised 
  double invs = 1 / (sqx + sqy + sqz + sqw); 

  m[0] = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs 
  m[4] = (-sqx + sqy - sqz + sqw)*invs ; 
  m[8] = (-sqx - sqy + sqz + sqw)*invs ; 

  double tmp1 = qx*qy; 
  double tmp2 = qz*qw; 
  m[3] = 2.0 * (tmp1 + tmp2)*invs ; 
  m[1] = 2.0 * (tmp1 - tmp2)*invs ; 

  tmp1 = qx*qz; 
  tmp2 = qy*qw; 
  m[6] = 2.0 * (tmp1 - tmp2)*invs ; 
  m[2] = 2.0 * (tmp1 + tmp2)*invs ; 
  tmp1 = qy*qz; 
  tmp2 = qx*qw; 
  m[7] = 2.0 * (tmp1 + tmp2)*invs ; 
  m[5] = 2.0 * (tmp1 - tmp2)*invs ; 

  yaw = atan2(-m[6],m[0]); 
  pitch = asin(m[3]); 
  roll = atan2(-m[5],m[4]); 
}


