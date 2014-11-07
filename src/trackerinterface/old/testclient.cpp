#include <client.h>


int main()
{
  Client cl;
  cl.connect("192.168.1.241:1511");
  cl.listen();
  return 0;

}
