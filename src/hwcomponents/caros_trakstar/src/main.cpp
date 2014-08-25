#include "ros/ros.h"
#include "trakstar_raw.h"


int main(int argc, char **argv)
{
   ros::init(argc, argv, "trakstar_raw");

   TrakstarRaw tstar;
   tstar.onInit();

   tstar.run();

   return 0;
}
