#ifndef RIGIDBODYDATA_H
#define RIGIDBODYDATA_H

#include <cmath>
#include <iostream>



class RigidBodyData
{

public:

    RigidBodyData(){}
    ~RigidBodyData(){}

    RigidBodyData(float x,float y,float z,float qx,float qy,float qz,float qw, int id):x(x),y(y), z(z), qx(qx), qy(qy), qz(qz), qw(qw), id(id){
        rx = atan2( 2*(qy*qw - qx*qz),1-2*(qy*qy - qz*qz) );
        ry = asin( 2*qx*qy + 2*qz*qw );
        rz = atan2( 2*(qx*qw - qy*qz),1-2*(qx*qx - qz*qz) );

    }
    float x;
    float y;
    float z;

    float qx;
    float qy;
    float qz;
    float qw;

    float rx;
    float ry;
    float rz;

    int id;

};

#endif // RIGIDBODYDATA_H
