#include "planar_man_services.h"

int main (int argc, char **argv){

    ros::init(argc,argv,"planar_man_services");
    
    Planar_man_services services;

    services.runServices();

    return 0;

}
