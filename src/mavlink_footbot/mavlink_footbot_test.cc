#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <mavlink_footbot.h>


void myflush ( FILE *in )
{
  int ch;

  do
    ch = fgetc ( in ); 
  while ( ch != EOF && ch != '\n' ); 

  clearerr ( in );
}

void mypause ( void ) 
{ 
  printf ( "Press [Enter] to continue . . ." );
  fflush ( stdout );
  getchar();
}
 
int main(int argc, char* argv[])
{

  MAVLinkFootbot mv;
  mv.start();
  mypause();
 
  mv.stop(); 
  return 0;
}
