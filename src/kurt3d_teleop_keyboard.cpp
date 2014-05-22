/*
 * based on teleop_pr2_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Kevin Watts
// Author: Henning Deeken

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "kurt3d/Scan.h"
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_Y 0x79
#define KEYCODE_X 0x78
#define KEYCODE_N 0x6E

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

#define KEYCODE_SPACE 0x20
#define KEYCODE_R 0x72

#define KEYCODE_0 0x30 
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34 
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_8 0x38
#define KEYCODE_9 0x39

class Kurt3DTeleopKeyboard
{
  private:
  double walk_vel, run_vel, yaw_rate, yaw_rate_run;
  geometry_msgs::Twist velocity_cmd;

  ros::NodeHandle n_;
  ros::Publisher velocity_cmd_pub;
  ros::ServiceClient client;
  public:
  void init()
  { 
    velocity_cmd.linear.x = velocity_cmd.linear.y = velocity_cmd.angular.z = 0;
    velocity_cmd_pub = n_.advertise<geometry_msgs::Twist>("velocity_cmd", 1);

    ros::NodeHandle n_private("~");
    n_private.param("walk_vel", walk_vel, 0.5);
    n_private.param("run_vel", run_vel, 1.0);
    n_private.param("yaw_rate", yaw_rate, 1.0);
    n_private.param("yaw_run_rate", yaw_rate_run, 1.5);
    
    ROS_INFO("Waiting for [laserscanner_node] to be advertised");
    ros::service::waitForService("assemble_scans2");
    ROS_INFO("Found laserscanner_node! Starting the teleop node");
    client = n_.serviceClient<kurt3d::Scan>("laserscanner_node");

    
  }
  
  ~Kurt3DTeleopKeyboard()   { }
  void keyboardLoop();

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kurt3d_teleop_key");

  Kurt3DTeleopKeyboard tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
}

void Kurt3DTeleopKeyboard::keyboardLoop()
{
  char c;
  bool velocity_cmd_set=false;
  bool action_cmd_set=false;
  bool audio_cmd_set=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WASD' to translate");
  puts("Use 'QE' to yaw");
  puts("Press 'Shift' to run");
  puts("Press 'y' start scanning");

  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    velocity_cmd.linear.x = velocity_cmd.linear.y = velocity_cmd.angular.z = 0;

    switch(c)
    {
    
    //slow velocity commands
    
    case KEYCODE_W:
      velocity_cmd.linear.x = walk_vel;
      velocity_cmd_set = true;
      break;
    case KEYCODE_S:
      velocity_cmd.linear.x = - walk_vel;
      velocity_cmd_set = true;
      break;
    case KEYCODE_A:
      velocity_cmd.linear.y = walk_vel;
      velocity_cmd_set = true;
      break;
    case KEYCODE_D:
      velocity_cmd.linear.y = - walk_vel;
      velocity_cmd_set = true;
      break;
    case KEYCODE_Q:
      velocity_cmd.angular.z = yaw_rate;
      velocity_cmd_set = true;
      break;
    case KEYCODE_E:
      velocity_cmd.angular.z = - yaw_rate;
      velocity_cmd_set = true;
      break;
      
    //fast velocity commands
    case KEYCODE_W_CAP:
      velocity_cmd.linear.x = run_vel;
      velocity_cmd_set = true;
      break;
    case KEYCODE_S_CAP:
      velocity_cmd.linear.x = - run_vel;
      velocity_cmd_set = true;
      break;
    case KEYCODE_A_CAP:
      velocity_cmd.linear.y = run_vel;
      velocity_cmd_set = true;
      break;
    case KEYCODE_D_CAP:
      velocity_cmd.linear.y = - run_vel;
      velocity_cmd_set = true;
      break;
    case KEYCODE_Q_CAP:
      velocity_cmd.angular.z = yaw_rate_run;
      velocity_cmd_set = true;
      break;
    case KEYCODE_E_CAP:
      velocity_cmd.angular.z = - yaw_rate_run;
      velocity_cmd_set = true;
      break;

    //action commands

    case KEYCODE_Y:
      kurt3d::Scan srv;

	  if (client.call(srv))
      {
        ROS_INFO("Scan finished");
      }
      else
      {
        ROS_ERROR("Failed to call laserscanner_node service");
      }
     break;
      
    }
    
    if (velocity_cmd_set == true)
    {
      velocity_cmd_pub.publish(velocity_cmd);
      velocity_cmd_set = false;
    }
  
  }
}
