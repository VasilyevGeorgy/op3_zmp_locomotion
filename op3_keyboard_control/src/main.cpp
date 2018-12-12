#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include <string>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <curses.h>

int main (int argc, char **argv){

  ros::init(argc,argv,"op3_keyboard_control");
  ros::NodeHandle node;

  ros::Publisher get_key_pub = node.advertise<std_msgs::String>("op3_keyboard_control", 1000);

  ROS_INFO("Use keyboard to control robot locomotion process");
  ROS_INFO("***Press   R   to start***");
  ROS_INFO("***Press SPACE to stop***");
  ROS_INFO("***Press   Q   to quit***");

  ros::Rate rate(1000); //Hz

  bool quit = false;
  bool is_running = false;

  while(ros::ok()){

    std_msgs::String control_msg;
    std::stringstream sstream;

    // Curses library code
    WINDOW *w;
    w=initscr();     // init new screen
    cbreak();        // use cbreack call to make terminal send all keystrokes directly
    nodelay(w,true); // non-blocking call for getch

    if(is_running){
      sstream.str("");
      sstream<<"run";
    }
    else{
      sstream.str("");
      sstream<<"stop";
    }
    if(quit){
      sstream.str("");
      sstream<<"quit";
    }

    control_msg.data = sstream.str();
    get_key_pub.publish(control_msg);

    //ROS_INFO("%s",control_msg.data.c_str());

    refresh();       // push from buffer to the real terminal
    //std::cout<<sstream.str();
    endwin();


    if(sstream.str() == "quit"){
      break;
    }

    char input_key=getch();
    if((input_key=='q')||(input_key=='Q')){ // ESC == 27
      quit = true;
    }
    if((input_key == 'r')||(input_key == 'R')){ // ENTER == 13
      is_running = true;
    }
    if(input_key == 32){
      is_running = false;
    }

    ros::spinOnce();
    rate.sleep();
  }

  //system("clear");

  return 0;
}
