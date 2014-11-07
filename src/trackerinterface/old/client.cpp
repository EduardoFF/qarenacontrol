#include "client.h"
#include <algorithm>

using namespace std;

int Client::NatNetVersion[4] = {3,3,0,0};
int Client::ServerVersion[4] = {3,3,0,0};


int Client::z;
int Client::x;
struct sockaddr_in Client::adr;  /* AF_INET */
int Client::len_inet;            /* length */
int Client::s;                   /* Socket */
//char Client::dgram[512];         /* Recv buffer */
char Client::dgram[256*1024];         /* Recv buffer */

int Client::so_reuseaddr = TRUE;

//void Client::displayError(const char *on_what) {
//     fputs(strerror(errno),stderr);
//     fputs(": ",stderr);
//     fputs(on_what,stderr);
//     fputc('\n',stderr);
//     exit(1);
//}

void Client::displayError(const char *on_what) {
    fputs(strerror(errno),stderr);
    fputs(": ",stderr);
    fputs(on_what,stderr);
    fputc('\n',stderr);
    //exit(1);
    connected=false;
}

Client::Client()
{
    threshold_to_destination= 0.5f;
}


bool Client::isConnected(){
    return connected;
}

void Client::connect(char *bc_addr){


    std::cerr<<"connecting to "<<bc_addr<<std::endl;


    /*
         * Create a UDP socket to use:
         */
    s = socket(AF_INET,SOCK_DGRAM,0);
    if ( s == -1 )
        displayError("socket()");

    /*
         * Form the broadcast address:
         */
    len_inet = sizeof adr;

    z = mkaddr(&adr,
               &len_inet,
               bc_addr,
               "udp");

    if ( z == -1 ){
        displayError("Bad broadcast address");
        return;
    }



    /*
         * Allow multiple listeners on the
         * broadcast address:
         */
    z = setsockopt(s,
                   SOL_SOCKET,
                   SO_REUSEADDR,
                   &so_reuseaddr,
                   sizeof so_reuseaddr);

    if ( z == -1 ){
        displayError("setsockopt(SO_REUSEADDR)");
        return;
    }

    /*
       * Bind our socket to the broadcast address:
    */
    z = bind(s,(struct sockaddr *)&adr,len_inet);


    if ( z == -1 ){
        displayError("bind(2)");
        return;
    }
    else{
        connected=true;
    }

}

void Client::listen(){


    std::cerr<<" Client::listen()::Updating robot positions with rigid Bodies "<< rigidBodiesData.size()<<std::endl;

    // Wait for a broadcast message:
    z = recvfrom(s,      /* Socket */
                 dgram,  /* Receiving buffer */
                 sizeof dgram,/* Max rcv buf size */
                 0,      /* Flags: no options */
                 (struct sockaddr *)&adr, /* Addr */
                 (socklen_t*)&x);    /* Addr len, in & out */

    if ( z < 0 ){
        displayError("recvfrom(2)"); /* else err */
    }
    std::cerr<<"unpack "<<std::endl;

    Unpack(dgram);
    std::cerr<<"unpacked "<<std::endl;
    std::cerr<<" Client::listen():: AFTER Updating robot positions nbRigid"<<rigidBodiesData.size()<<std::endl;

}

void Client::listenTest(){

    std::cerr<<" Client::listen()::test"<<std::endl;
    rigidBodiesData.clear();

    /*
    rigidBodiesData.push_back(RigidBodyData(1.5*1.2,0,1.5*1.2,1,1,1,1,1));
    rigidBodiesData.push_back(RigidBodyData(1.5*1.2,0,4.5*1.2,1,1,1,1,2));
    rigidBodiesData.push_back(RigidBodyData(4.5*1.2,0,2.5*1.2,1,1,1,1,3));
*/
    rigidBodiesData.push_back(RigidBodyData(8.5*1.2,0,3.5*1.2,1,1,1,1,3));
    rigidBodiesData.push_back(RigidBodyData(3.5*1.2,0,6.5*1.2,1,1,1,1,2));
    rigidBodiesData.push_back(RigidBodyData(0.5*1.2,0,3.5*1.2,1,1,1,1,1));

}

int Client::mkaddr(void* addr,int *addrlen,char *str_addr,char *protocol) {

    char *inp_addr = strdup(str_addr);
    char *host_part = strtok(inp_addr, ":" );
    char *port_part = strtok(NULL, "\n" );
    struct sockaddr_in *ap =
            (struct sockaddr_in *) addr;
            struct hostent *hp = NULL;
            struct servent *sp = NULL;
            char *cp;
            long lv;

            /*
    * Set input defaults:
    */
            if ( !host_part ) {
                host_part =  "*" ;
            }
            if ( !port_part ) {
                port_part =  "*" ;
            }
            if ( !protocol ) {
                protocol =  "tcp" ;
            }

            /*
    * Initialize the address structure:
    */
            memset(ap,0,*addrlen);
            ap->sin_family = AF_INET;
            ap->sin_port = 0;
            ap->sin_addr.s_addr = INADDR_ANY;

            /*
    * Fill in the host address:
    */
            if ( strcmp(host_part, "*" ) == 0 ) {
                ; /* Leave as INADDR_ANY */
            }
            else if ( isdigit(*host_part) ) {
                /*
      * Numeric IP address:
      */
                ap->sin_addr.s_addr =
                        inet_addr(host_part);
                // if ( ap->sin_addr.s_addr == INADDR_NONE ) {
                if ( !inet_aton(host_part,&ap->sin_addr) ) {
                    return -1;
                }
            }
            else {
                /*
    * Assume a hostname:
    */
                hp = gethostbyname(host_part);
                if ( !hp ) {
                    return -1;
                }
                if ( hp->h_addrtype != AF_INET ) {
                    return -1;
                }
                ap->sin_addr = * (struct in_addr *)
                        hp->h_addr_list[0];
            }

            /*
    * Process an optional port #:
    */
            if ( !strcmp(port_part, "*" ) ) {
                /* Leave as wild (zero) */
            }
            else if ( isdigit(*port_part) ) {
                /*
    * Process numeric port #:
    */
                lv = strtol(port_part,&cp,10);
                if ( cp != NULL && *cp ) {
                    return -2;
                }
                if ( lv < 0L || lv >= 32768 ) {
                    return -2;
                }
                ap->sin_port = htons( (short)lv);
            }
            else {
                /*
    * Lookup the service:
    */
                sp = getservbyname( port_part, protocol);
                if ( !sp ) {
                    return -2;
                }
                ap->sin_port = (short) sp->s_port;
            }

            /*
    * Return address length
    */
            *addrlen = sizeof *ap;

            free(inp_addr);
            return 0;
}



bool Client::DecodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int* hour, int* minute, int* second, int* frame, int* subframe)
{
    bool bValid = true;
    *hour = (inTimecode>>24)&255;
    *minute = (inTimecode>>16)&255;
    *second = (inTimecode>>8)&255;
    *frame = inTimecode&255;
    *subframe = inTimecodeSubframe;
    return bValid;
}

bool Client::TimecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe, char *Buffer, int BufferSize)
{
    bool bValid;
    int hour, minute, second, frame, subframe;
    bValid = DecodeTimecode(inTimecode, inTimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
    //sprintf_s(Buffer,BufferSize,"%2d:%2d:%2d:%2d.%d",hour, minute, second, frame, subframe);
    snprintf(Buffer,BufferSize,"%2d:%2d:%2d:%2d.%d",hour, minute, second, frame, subframe);
    for(unsigned int i=0; i<strlen(Buffer); i++)
        if(Buffer[i]==' ')
            Buffer[i]='0';

    return bValid;
}


void Client::Unpack(char* pData)
{
    std::cerr <<"Begin Packet\n-------" << std::endl ;
    int major = NatNetVersion[0];
    int minor = NatNetVersion[1];
    char *ptr = pData;

    // message ID
    short int MessageID = 0;
    memcpy(&MessageID, ptr, 2); ptr += 2;
    //std::cerr <<"Message ID : "<< MessageID <<std::endl;

    // size
    short int nBytes = 0;
    memcpy(&nBytes, ptr, 2); ptr += 2;
    //std::cerr <<"Byte count : "<< nBytes << std::endl;

    if(MessageID == 7)      // FRAME OF MOCAP DATA packet
    {
        // frame number
        int frameNumber = 0; memcpy(&frameNumber, ptr, 4); ptr += 4;
        std::cerr <<"Frame # : " << frameNumber << std::endl;

        // number of data sets (markersets, rigidbodies, etc)
        int nMarkerSets = 0; memcpy(&nMarkerSets, ptr, 4); ptr += 4;
        //std::cerr <<"Marker Set Count : " << nMarkerSets << std::endl;

        for (int i=0; i < nMarkerSets; i++)
        {
            // Markerset name
            char szName[256];
            //strcpy_s(szName, ptr);
            strcpy(szName, ptr);
            int nDataBytes = (int) strlen(szName) + 1;
            ptr += nDataBytes;
            std::cerr <<"Model Name: " << szName << std::endl;

            // marker data
            int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
            //std::cerr <<"Marker Count : " << nMarkers << std::endl;;
            //std::cerr<<" pointer "<<ptr<<std::endl;
            fprintf(stderr, "%p",ptr);
            for(int j=0; j < nMarkers; j++)
            {
                float x = 0; memcpy(&x, ptr, 4); ptr += 4;
                float y = 0; memcpy(&y, ptr, 4); ptr += 4;
                float z = 0; memcpy(&z, ptr, 4); ptr += 4;
                std::cerr <<"\tMarker " << j << " : [x=" << x << ",y=" << y << ",z=" << z << "]" << std::endl;
            }
        }

        // unidentified markers
        int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
        std::cerr <<"Unidentified Marker Count : " << nOtherMarkers<< std::endl;
        for(int j=0; j < nOtherMarkers; j++)
        {
            float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
            float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
            float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
            std::cerr <<"\tMarker " << j << " : [x=" << x << ",y=" << y << ",z=" << z << "]" << std::endl;
        }

        // rigid bodies

        int nRigidBodies = 0;
        memcpy(&nRigidBodies, ptr, 4); ptr += 4;
        std::cerr <<"Rigid Body Count : "<< nRigidBodies<< std::endl;
        rigidBodiesData.clear();
        //rigidBodiesData.resize(nRigidBodies);

        for (int j=0; j < nRigidBodies; j++)
        {
            // rigid body pos/ori
            int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
            float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
            float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
            float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
            float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
            float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
            float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
            float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
            std::cerr <<"ID : " << ID << std::endl;
            std::cerr <<"pos: ["<<x<<"," << y << "," << z <<"]" << std::endl;
            std::cerr <<"ori: ["<<qx<< "," <<qy<< "," <<qz<< "," <<qw<<"]" << std::endl;

            //rigid body data
            rigidBodiesData.push_back(RigidBodyData(x,y,z,qx,qy,qz,qw,ID));
            fprintf(stderr, "Quat: rx %f - ry %f - rz %f\n",
                    rigidBodiesData[rigidBodiesData.size()-1].rx,
                    (rigidBodiesData.at(rigidBodiesData.size()-1)).ry,
                    (rigidBodiesData.at(rigidBodiesData.size()-1)).rz);

            // associated marker positions
            int nRigidMarkers = 0;  memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
            std::cerr <<"Marker Count from rigid bodies : " << nRigidMarkers<< std::endl;
            int nBytes = nRigidMarkers*3*sizeof(float);
            std::cerr<<"nbytes "<<nBytes<<std::endl;
            float* markerData = (float*)malloc(nBytes);
            memcpy(markerData, ptr, nBytes);
            ptr += nBytes;

            if(major >= 2 || true)
            {
                // associated marker IDs
                nBytes = nRigidMarkers*sizeof(int);
                int* markerIDs = (int*)malloc(nBytes);
                memcpy(markerIDs, ptr, nBytes);
                ptr += nBytes;

                // associated marker sizes
                nBytes = nRigidMarkers*sizeof(float);
                float* markerSizes = (float*)malloc(nBytes);
                memcpy(markerSizes, ptr, nBytes);
                ptr += nBytes;

                // 2.6 and later
                if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) || true)
                {
                    //std::cerr<<" if major "<<major<<" "<<minor<<std::endl;
                    // params
                    short params = 0; memcpy(&params, ptr, 2); ptr += 2;
                    bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
                    //std::cerr<<"params " <<params<<" "<<bTrackingValid<<std::endl;

                }
                //                for(int k=0; k < nRigidMarkers; k++)
                //                {
                //                    std::cerr <<"\tMarker " << k <<": " << "id=" <<markerIDs[k] << " size="<< markerSizes[k]<<
                //                                " pos=["<< markerData[k*3]<<","<< markerData[k*3+1] <<","<<markerData[k*3+2]<< "]"<< std::endl;
                //                }
                if(markerIDs)
                    free(markerIDs);
                if(markerSizes)
                    free(markerSizes);
            }
            else
            {
                //                for(int k=0; k < nRigidMarkers; k++)
                //                {
                //                    std::cerr <<"\tMarker " << k << " : pos = [" << markerData[k*3] << "," << markerData[k*3+1] << "," << markerData[k*3+2] << "]" << std::endl;

                //                }
            }
            if(markerData)
                free(markerData);
            if(major >= 2)
            {
                // Mean marker error
                float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
                //std::cerr <<"Mean marker error: " << fError << std::endl;
            }
        } // next rigid body
        // skeletons (version 2.1 and later)
        if( ((major == 2)&&(minor>0)) || (major>2))
        {
            int nSkeletons = 0;
            memcpy(&nSkeletons, ptr, 4); ptr += 4;
            std::cerr <<"Skeleton Count : "<< nSkeletons<< std::endl;
            for (int j=0; j < nSkeletons; j++)
            {
                // skeleton id
                int skeletonID = 0;
                memcpy(&skeletonID, ptr, 4); ptr += 4;
                // # of rigid bodies (bones) in skeleton
                int nRigidBodies = 0;
                memcpy(&nRigidBodies, ptr, 4); ptr += 4;
                std::cerr <<"Rigid Body Count : "<< nRigidBodies<< std::endl;

                rigidBodiesData.clear();
                rigidBodiesData.resize(nRigidBodies);

                for (int j=0; j < nRigidBodies; j++)
                {
                    // rigid body pos/ori
                    int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
                    float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
                    float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
                    float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
                    float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
                    float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
                    float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
                    float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
                    std::cerr <<"ID : "<< ID<< std::endl;
                    std::cerr <<"pos: [" << x << "," << y << "," << z <<"]"<< std::endl;
                    std::cerr <<"ori: [" << qx << "," << qy << "," << qz << "," << qw <<"]"<< std::endl;

                    rigidBodiesData.push_back(RigidBodyData(x,y,z,qx,qy,qz,qw,ID));

                    // associated marker positions
                    int nRigidMarkers = 0;  memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
                    //      std::cerr <<"Marker Count: "<< nRigidMarkers<< std::endl;
                    int nBytes = nRigidMarkers*3*sizeof(float);
                    float* markerData = (float*)malloc(nBytes);
                    memcpy(markerData, ptr, nBytes);
                    ptr += nBytes;

                    // associated marker IDs
                    nBytes = nRigidMarkers*sizeof(int);
                    int* markerIDs = (int*)malloc(nBytes);
                    memcpy(markerIDs, ptr, nBytes);
                    ptr += nBytes;

                    // associated marker sizes
                    nBytes = nRigidMarkers*sizeof(float);
                    float* markerSizes = (float*)malloc(nBytes);
                    memcpy(markerSizes, ptr, nBytes); ptr += nBytes;

                    /*
                    for(int k=0; k < nRigidMarkers; k++)
                    {
                        std::cerr <<"\tMarker <<" <<k <<": id=" << markerIDs[k] << " size= " << markerSizes[k] <<
                                    "pos=" << "["<<  markerData[k*3] << ","<< markerData[k*3+1] << ","<<markerData[k*3+2] <<"]" << std::endl;
                    }
            */

                    // Mean marker error
                    float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
                    // std::cerr <<"Mean marker error: "<< fError <<std::endl;;

                    // release resources
                    if(markerIDs)
                        free(markerIDs);
                    if(markerSizes)
                        free(markerSizes);
                    if(markerData)
                        free(markerData);

                } // next rigid body

            } // next skeleton
        }

        // labeled markers (version 2.3 and later)
        if( ((major == 2)&&(minor>=3)) || (major>2))
        {
            int nLabeledMarkers = 0;
            memcpy(&nLabeledMarkers, ptr, 4); ptr += 4;
            //std::cerr <<"Labeled Marker Count : "<< nLabeledMarkers<< std::endl;
            for (int j=0; j < nLabeledMarkers; j++)
            {
                // id
                int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
                // x
                float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
                // y
                float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
                // z
                float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
                // size
                float size = 0.0f; memcpy(&size, ptr, 4); ptr += 4;

                // 2.6 and later
                if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) )
                {
                    // marker params
                    short params = 0; memcpy(&params, ptr, 2); ptr += 2;
                    bool bOccluded = params & 0x01;     // marker was not visible (occluded) in this frame
                    bool bPCSolved = params & 0x02;     // position provided by point cloud solve
                    bool bModelSolved = params & 0x04;  // position provided by model solve
                }

                /*
                std::cerr <<"ID  : "<< ID<< std::endl;
                std::cerr <<"pos : [" << x << ","<< y << ","<< z << "]"<<std::endl;
                std::cerr <<"size: [" << size << "]"<<std::endl;
        */
            }
        }

        // latency
        float latency = 0.0f; memcpy(&latency, ptr, 4);	ptr += 4;
        //std::cerr <<"latency : "<< latency<<std::endl;

        // timecode
        unsigned int timecode = 0; 	memcpy(&timecode, ptr, 4);	ptr += 4;
        unsigned int timecodeSub = 0; memcpy(&timecodeSub, ptr, 4); ptr += 4;
        char szTimecode[128] = "";
        TimecodeStringify(timecode, timecodeSub, szTimecode, 128);

        // timestamp
        float timestamp = 0.0f;  memcpy(&timestamp, ptr, 4); ptr += 4;

        // frame params
        short params = 0;  memcpy(&params, ptr, 2); ptr += 2;
        bool bIsRecording = params & 0x01;                  // 0x01 Motive is recording
        bool bTrackedModelsChanged = params & 0x02;         // 0x02 Actively tracked model list has changed


        // end of data tag
        int eod = 0; memcpy(&eod, ptr, 4); ptr += 4;
        std::cerr <<"End Packet\n-------------" << std::endl;

    }
    else if(MessageID == 5) // Data Descriptions
    {
        // number of datasets
        int nDatasets = 0; memcpy(&nDatasets, ptr, 4); ptr += 4;
        std::cerr <<"Dataset Count : "<< nDatasets<< std::endl;

        for(int i=0; i < nDatasets; i++)
        {
            //std::cerr <<"Dataset "<< i<< std::endl;

            int type = 0; memcpy(&type, ptr, 4); ptr += 4;
            //std::cerr <<"Type : "<< i <<","<< type<< std::endl;

            if(type == 0)   // markerset
            {
                // name
                char szName[256];
                //strcpy_s(szName, ptr);
                strcpy(szName, ptr);
                int nDataBytes = (int) strlen(szName) + 1;
                ptr += nDataBytes;
                //std::cerr <<"Markerset Name: "<< szName<< std::endl;

                // marker data
                int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
                //std::cerr <<"Marker Count : "<< nMarkers<< std::endl;

                for(int j=0; j < nMarkers; j++)
                {
                    char szName[256];
                    //strcpy_s(szName, ptr);
                    strcpy(szName, ptr);
                    int nDataBytes = (int) strlen(szName) + 1;
                    ptr += nDataBytes;
                    //  std::cerr <<"Marker Name: "<< szName<< std::endl;
                }
            }
            else if(type ==1)   // rigid body
            {
                if(major >= 2)
                {
                    // name
                    char szName[MAX_NAMELENGTH];
                    strcpy(szName, ptr);
                    ptr += strlen(ptr) + 1;
                    std::cerr <<"Name: "<< szName<< std::endl;
                }

                int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
                std::cerr <<"ID : "<< ID<< std::endl;

                int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
                //std::cerr <<"Parent ID : "<< parentID<< std::endl;

                float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
                //std::cerr <<"X Offset : "<< xoffset<< std::endl;

                float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
                //std::cerr <<"Y Offset : "<< yoffset<< std::endl;

                float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
                //std::cerr <<"Z Offset : "<< zoffset<< std::endl;

            }
            else if(type ==2)   // skeleton
            {
                char szName[MAX_NAMELENGTH];
                strcpy(szName, ptr);
                ptr += strlen(ptr) + 1;
                //std::cerr <<"Name: " << szName<< std::endl;

                int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
                //std::cerr <<"ID : "<< ID << std::endl;

                int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr +=4;
                //std::cerr <<"RigidBody (Bone) Count : "<< nRigidBodies << std::endl;

                for(int i=0; i< nRigidBodies; i++)
                {
                    if(major >= 2)
                    {
                        // RB name
                        char szName[MAX_NAMELENGTH];
                        strcpy(szName, ptr);
                        ptr += strlen(ptr) + 1;
                        std::cerr <<"Rigid Body Name: "<< szName<< std::endl;
                    }

                    int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
                    std::cerr <<"RigidBody ID : "<< ID<< std::endl;

                    int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
                    std::cerr <<"Parent ID : "<< parentID<< std::endl;

                    float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
                    std::cerr <<"X Offset : "<< xoffset<< std::endl;

                    float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
                    std::cerr <<"Y Offset : "<< yoffset<< std::endl;

                    float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
                    std::cerr <<"Z Offset : "<< zoffset << std::endl;
                }
            }

        }   // next dataset

        std::cerr <<"End Packet\n-------------"<< std::endl;

    }
    else
    {
        std::cerr <<"Unrecognized Packet Type."<< std::endl;
        exit -1;
    }


}

bool Client::arrivedCloseEnough()
{
/*
    bool arrived = true;

    for( unsigned int i = 0 ; i < rigidBodiesData.size() ; i++ ){
        int ID = rigidBodiesData[i].id;

        if( ID != HUMAN_ID && targetRigidBodiesData.size() > 0 ){
            if(targetRigidBodiesData.find( ID ) == targetRigidBodiesData.end() ){
                std::cerr << "Client::arrivedCloseEnough()::WARNING::ID of rigib body not in target!! ERROR consider arrived"<<std::endl;
                atDestination[ID] = true;
            } else {

                float x = rigidBodiesData[ i ].x;
                float z = rigidBodiesData[ i ].z;

                float xdest = targetRigidBodiesData[ ID ].x;
                float zdest = targetRigidBodiesData[ ID ].z;

                if( !atDestination[ID] ){
                    if( fabs( xdest - x ) < threshold_to_destination && fabs( zdest - z ) < threshold_to_destination ){
                        std::cerr << "Client::arrivedCloseEnough()::Robot " << ID << " arrived at destination " <<std::endl;
                        atDestination[ID] = true;
                    } else {
                        arrived = false;
                    }
                }
            }
        }
    }

    if (arrived) {
        targetRigidBodiesData.clear();
        atDestination.clear();

        std::cerr << "Client::arrivedCloseEnough()::All Robots arrived at destination" <<std::endl;
    }

    return arrived;
    */
}

void Client::setTargetRigidBodiesDataTest( std::map<int, RigidBodyData> _rigidBodiesData) {
    targetRigidBodiesData.clear();
    atDestination.clear();

    for( unsigned int i = 0 ; i < rigidBodiesData.size() ; i++ ){
        int ID = rigidBodiesData[i].id;
        rigidBodiesData[ i  ] = _rigidBodiesData[ID];
        targetRigidBodiesData[ ID ] = _rigidBodiesData[ID];
        atDestination[ ID ] = true;
    }
}

void Client::setTargetRigidBodiesData( std::map<int, RigidBodyData> _rigidBodiesData) {
    targetRigidBodiesData.clear();
    atDestination.clear();

    for( unsigned int i = 0 ; i < rigidBodiesData.size() ; i++ ){
        int ID = rigidBodiesData[i].id;
        targetRigidBodiesData[ ID ] = _rigidBodiesData[ID];
        atDestination[ ID ] = false;
    }
}
