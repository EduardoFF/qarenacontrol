#ifndef CLIENT_H
#define CLIENT_H

//#include <QObject>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


#include <ctype.h>
#include <netdb.h>
#include <iostream>

#include "RigidBodyData.h"
#include <vector>
#include <map>
//#include <Player.h>

//#include <QDebug>


#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

// *************************************************   NATNET ******************************
#define MAX_NAMELENGTH              256
// NATNET message ids
#define NAT_PING                    0
#define NAT_PINGRESPONSE            1
#define NAT_REQUEST                 2
#define NAT_RESPONSE                3
#define NAT_REQUEST_MODELDEF        4
#define NAT_MODELDEF                5
#define NAT_REQUEST_FRAMEOFDATA     6
#define NAT_FRAMEOFDATA             7
#define NAT_MESSAGESTRING           8
#define NAT_UNRECOGNIZED_REQUEST    100
#define UNDEFINED                   999999.9999
#define MAX_PACKETSIZE				100000	// max size of packet (actual packet size is dynamic)
// sender
typedef struct
{
    char szName[MAX_NAMELENGTH];            // sending app's name
    unsigned char Version[4];               // sending app's version [major.minor.build.revision]
    unsigned char NatNetVersion[4];         // sending app's NatNet version [major.minor.build.revision]

} sSender;

typedef struct
{
    unsigned short iMessage;                // message ID (e.g. NAT_FRAMEOFDATA)
    unsigned short nDataBytes;              // Num bytes in payload
    union
    {
        unsigned char  cData[MAX_PACKETSIZE];
        char           szData[MAX_PACKETSIZE];
        unsigned long  lData[MAX_PACKETSIZE/4];
        float          fData[MAX_PACKETSIZE/4];
        sSender        Sender;
    } Data;                                 // Payload

} sPacket;
#define MULTICAST_ADDRESS		"239.255.42.99"     // IANA, local network
#define PORT_COMMAND            1510
#define PORT_DATA  			    1511                // Default multicast group
// ************************************************* END  NATNET ******************************



class Client 
{
    //Q_OBJECT
public:
    // *************************************************   NATNET ******************************
    static int NatNetVersion[4];
    static int ServerVersion[4];
    // ************************************************* END  NATNET ******************************



    Client();
    ~Client(){}
    void connect(char *bc_addr);
    void displayError(const char *on_what);

    bool connected;
    bool isConnected();

    void listen();
    void listenTest();

    void Unpack(char* pData);
    bool DecodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int* hour, int* minute, int* second, int* frame, int* subframe);
    bool TimecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe, char *Buffer, int BufferSize);
    //bool getPosYaw( int id, float &x, float &y, float &z, float &yaw);

    void setThresholdToDestination( float _threshold_to_destination ) { threshold_to_destination = _threshold_to_destination; }

    static int z;
    static int x;
    static struct sockaddr_in adr;  /* AF_INET */
    static int len_inet;            /* length */
    static int s;                   /* Socket */
    //static char dgram[512];         /* Recv buffer */
    static char dgram[256*1024];         /* Recv buffer */
    static int so_reuseaddr;


    int mkaddr(void *addr,
               int *addrlen,
               char *str_addr,
               char *protocol);

    void getRigidBodiesData (std::vector<RigidBodyData> & copy){
        std::cerr<<" get RigidBodiesData client of size " << rigidBodiesData.size() <<std::endl;
        copy = std::vector<RigidBodyData> (rigidBodiesData);
        std::cerr<<" get RigidBodiesData client out "<<std::endl;
    }

    void setTargetRigidBodiesData( std::map<int,RigidBodyData> _rigidBodiesData);

    void setTargetRigidBodiesDataTest( std::map<int,RigidBodyData> _rigidBodiesData);

    bool arrivedCloseEnough();

private:
    std::vector<RigidBodyData> rigidBodiesData;
    std::map<int, RigidBodyData> targetRigidBodiesData;

    std::map<int, bool> atDestination;

    float threshold_to_destination;

};

#endif // CLIENT_H
