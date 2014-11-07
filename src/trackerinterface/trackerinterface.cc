#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameListener.h>

#include <boost/program_options.hpp>
#include <time.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/lcmmsgs.hpp"


#define ALL(x) x.begin(), x.end()
#define PV(v,type) copy(ALL(v),ostream_iterator<type>(cout," "))
#define IT(c) __typeof((c).begin())
#define FOREACH(i,c) for(__typeof((c).begin()) i=(c).begin();i!=(c).end();++i)
#define CEIL(VARIABLE) ( (VARIABLE - (int)VARIABLE)==0 ? (int)VARIABLE : (int)VARIABLE+1 )
#define EPSILON 1e-4

#define FATALERROR(m, ...) {fprintf(stderr, "FATAL ERROR at %s %d: ",__FILE__, __LINE__);fprintf(stderr,m, ## __VA_ARGS__);fflush(stderr);exit(EXIT_FAILURE);}


#define PI 3.14159265
#define RADIANS(X) ((X)*PI/180)
#define DEGREES(X) ((X)*180.0)/PI
//#ifndef NDEBUG
#define DEBUGMSG(m, ...) fprintf(stderr, m ,## __VA_ARGS__ )
//#else
//#define DEBUGMSG(m, ...) 
//#endif



uint32_t g_localAddress = 0;
uint32_t g_serverAddress = 0;
bool g_run = false;

struct timeval g_totaTimeStart;

uint32_t g_windowSize = 1000;
/// end of the current time window in milliseconds since the start of the process
uint32_t g_nextFlush = g_windowSize;

/// Temporary buckets where data of the current time window are  stored per each rigid body id
struct  RobotPose
{
  double x,y,z,qx,qy,qz,qw;
};
std::map< int, std::vector< RobotPose > > g_poseBucket;

lcm::LCM *g_lcm;


uint32_t timeElapsed() ;

/// End the program gracefully.
void 
terminate(int)
{
   /// Tell the main() thread to close.
   g_run = false;
}

/// Set the global addresses from the command line.
void 
readOpts( int argc, char* argv[] )
{
   namespace po = boost::program_options;
   
   po::options_description desc("simple-example: demonstrates using NatNetLinux\nOptions");
   desc.add_options()
      ("help", "Display help message")
      ("local-addr,l", po::value<std::string>(), "Local IPv4 address")
      ("server-addr,s", po::value<std::string>(), "Server IPv4 address")
   ;
   
   po::variables_map vm;
   po::store(po::parse_command_line(argc,argv,desc), vm);
   
   if(
      argc < 5 || vm.count("help") ||
      !vm.count("local-addr") ||
      !vm.count("server-addr")
   )
   {
      std::cout << desc << std::endl;
      exit(1);
   }
   
   g_localAddress = inet_addr( vm["local-addr"].as<std::string>().c_str() );
   g_serverAddress = inet_addr( vm["server-addr"].as<std::string>().c_str() );
}

void 
processFrame(MocapFrame &frame)
{
  //printf("processing frame\n");
  //DEBUGMSG("processFrame\n");
  std::vector<RigidBody> const& rBodies = frame.rigidBodies();
  FOREACH(it, rBodies)
  {
    const RigidBody &rB = *it;
    Point3f pos = rB.location();
    Quaternion4f ori = rB.orientation();
    RobotPose pose;
    pose.qx = ori.qx;
    pose.qy = ori.qy;
    pose.qz = ori.qz;
    pose.qw = ori.qw;
    pose.x  = pos.x;
    pose.y  = pos.y;
    pose.z  = pos.z;
    g_poseBucket[rB.id()].push_back( pose );
  }
}

void 
testQuaternion(double qx, double qy, double qz, double qw,
		    double &yaw, double &pitch, double &roll)
{ 
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

void
flushBucket()
{
  
  DEBUGMSG("Flush\n");
  poselcm::pose_list_t mymsg;
  mymsg.n = g_poseBucket.size();
  mymsg.poses.resize(mymsg.n);

  int ix=0;
  FOREACH(it, g_poseBucket)
  {
    DEBUGMSG("Robot %d has %d entries\n",
	     it->first, (it->second).size());
    std::vector< RobotPose > &poses = it->second;
    /// do aggregation, filtering, smoothing, etc

    RobotPose &pose = poses.back();
    poselcm::pose_t my_data;
    my_data.robotid = it->first;;
    my_data.position[0] = CEIL(1000*pose.x);
    my_data.position[1] = CEIL(1000*pose.y);
    my_data.position[2] = CEIL(1000*pose.z);

    printf("ori %f %f %f %f\n",
	   pose.qx, pose.qy, pose.qz, pose.qw);
    my_data.orientation[1] = CEIL(10000.0*pose.qx);
    my_data.orientation[2] = CEIL(10000.0*pose.qy);
    my_data.orientation[3] = CEIL(10000.0*pose.qz);
    my_data.orientation[0] = CEIL(10000.0*pose.qw);

    double yaw, pitch, roll;
    testQuaternion(pose.qx, pose.qy, pose.qz, pose.qw,
		   yaw, pitch, roll);
    //double yaw = atan( 2*(pose.qx*pose.qy + pose.qz*pose.qw) / (pose.qx*pose.qx - pose.qy*pose.qy - pose.qz*pose.qz + pose.qw*pose.qw));

    //double yaw2 = atan2( 2*(pose.qx*pose.qy + pose.qz*pose.qw), (pose.qx*pose.qx - pose.qy*pose.qy - pose.qz*pose.qz + pose.qw*pose.qw));

    DEBUGMSG("Sending ROBOT %d (%d, %d %d) - (%d %d %d %d)\n",
	     my_data.robotid,
	     my_data.position[0],
	     my_data.position[1],
	     my_data.position[2],
	     my_data.orientation[0],
	     my_data.orientation[1],
	     my_data.orientation[2],
	     my_data.orientation[3]);
    printf("Yaw Robot %d = (%.2f %.2f %.2f) \n", my_data.robotid, 
	   DEGREES(yaw), yaw, DEGREES(pitch));
    mymsg.poses[ix++] = my_data;
  }
  mymsg.timestamp = time( NULL );
  g_lcm->publish("EXAMPLE", &mymsg);
  DEBUGMSG("Flush bucket\n");
  g_poseBucket.clear();

}

/// This thread loop processes the frames as they arrive
void 
processFrames(FrameListener& frameListener)
{
  printf("process framesss\n");
   bool valid;
   MocapFrame frame;
   g_run = true;
   while(g_run)
   {
      while( true )
      {
	//printf("trying to get frame\n");
         /// Try to get a new frame from the listener.
         MocapFrame frame(frameListener.pop(&valid).first);
         /// Quit if the listener has no more frames.
         if( !valid )
	 {
	   //printf("Not valid!\n");
	   break;
	 }
	 //printf("valid\n");
	 processFrame(frame);
      }
      
      if( timeElapsed() > g_nextFlush )
      {
	printf("Flush time\n");
	flushBucket();
	g_nextFlush += g_windowSize;
      }
      /// Sleep for 1ms to simulate work :)
      usleep(1000);
   }
}

/// number of milliseconds elapsed since the start of the process
uint32_t timeElapsed() 
{
  struct timeval current_time;
  gettimeofday(&current_time, NULL);
  //printf("current_time tv_sec %d tv_usec %d\n",
	 //current_time.tv_sec, current_time.tv_usec);
  

  uint32_t total_time_ms = 
    (current_time.tv_sec - g_totaTimeStart.tv_sec)*1000 
    + (int)( (current_time.tv_usec - g_totaTimeStart.tv_usec) / 1000.0);
  //printf("elapsed ms %d", total_time_ms);
  
  return total_time_ms;
}


void
initialize()
{
  DEBUGMSG("initialize\n");
  g_lcm = new lcm::LCM("udpm://239.255.76.67:7667?ttl=1");
  if(!g_lcm->good())
  {
    FATALERROR("LCM Failed!\n");
  } else
  {
    DEBUGMSG("LCM Ok\n");
  }

  gettimeofday(&g_totaTimeStart, NULL); 
}

  int 
main(int argc, char* argv[])
{
  /// Version number of the NatNet protocol, as reported by the server.
  unsigned char natNetMajor;
  unsigned char natNetMinor;

  /// Sockets
  int sdCommand;
  int sdData;

  /// Catch ctrl-c and terminate gracefully.
  signal(SIGINT, terminate);

  /// Set addresses
  readOpts( argc, argv );
  /// Use this socket address to send commands to the server.
  struct sockaddr_in serverCommands = NatNet::createAddress(g_serverAddress, NatNet::commandPort);

  /// Create sockets
  sdCommand = NatNet::createCommandSocket( g_localAddress );
  sdData = NatNet::createDataSocket( g_localAddress );

  /// Start the CommandListener in a new thread.
  CommandListener commandListener(sdCommand);
  commandListener.start();

  /// Send a ping packet to the server so that it sends us the NatNet version
  /// in its response to commandListener.
  NatNetPacket ping = NatNetPacket::pingPacket();
  ping.send(sdCommand, serverCommands);

  /// Wait here for ping response to give us the NatNet version.
  commandListener.getNatNetVersion(natNetMajor, natNetMinor);

  /// Initialize
  initialize();  

  /// Start up a FrameListener in a new thread.
  FrameListener frameListener(sdData, natNetMajor, natNetMinor);
  frameListener.start();

  /// This infinite loop simulates a "worker" thread that reads the frame
  /// buffer each time through, and exits when ctrl-c is pressed.
  processFrames(frameListener);
  //timeStats(frameListener);

  /// Wait for threads to finish.
  frameListener.stop();
  commandListener.stop();
  frameListener.join();
  commandListener.join();

  /// Epilogue
  close(sdData);
  close(sdCommand);
  return 0;
}
