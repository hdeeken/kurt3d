/*
 * Copyright (c) 2014, Osnabrueck University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the Osnabrueck University nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include "kurt3d/ServoCommand.h"
#include "kurt3d/Scan.h"
#include <sensor_msgs/LaserScan.h>
#include <laser_assembler/AssembleScans2.h>
#include <cstdlib>
#define _USE_MATH_DEFINES

#define RANGE 1000

static ros::ServiceClient client;
static ros::ServiceClient pointCloudClient;

static ros::Publisher state_pub;
static ros::Publisher laser_pub;

static sensor_msgs::LaserScan lastPublishedScan;

static sensor_msgs::LaserScan currentScan;
static sensor_msgs::LaserScan lastScan;

double min_angle, max_angle, standby_angle;
double min_pos, max_pos, standby_pos;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  lastScan = currentScan;
  currentScan = *msg;
}

bool scan(kurt3d::Scan::Request  &req,
         kurt3d::Scan::Response &res)
{
    std::cout << "Min Pos" << std::endl;
    kurt3d::ServoCommand srv;

    srv.request.channel = 0;
    srv.request.angle = min_pos;
    srv.request.speed = 15;

    if (client.call(srv))
    {
      ROS_INFO("Movement finished");
    }
    else
    {
      ROS_ERROR("Failed to call service servo_command");
      return 1;
    }

    laser_assembler::AssembleScans2 assemble_srv;
    assemble_srv.request.begin = ros::Time::now();
    ros::Rate loop_rate(150);

    for(int i = 0; i < RANGE && ros::ok(); i++)
    {
        kurt3d::ServoCommand srv;
        float angle;

        angle = min_pos+ (float) i / (float) RANGE * (max_pos-min_pos);

        ROS_INFO("angle: [%f]",angle);

        std::cout << "Angle: " << angle << std::endl;

        srv.request.channel = 0;
        srv.request.angle = (angle);
        srv.request.speed = 5;

        if (client.call(srv))
        {
          ROS_INFO("Movement finished");

          while(ros::ok() && (lastPublishedScan.header.seq == currentScan.header.seq || lastPublishedScan.header.seq == lastScan.header.seq))
          {
              ros::spinOnce();
              loop_rate.sleep();
          }

          laser_pub.publish(lastScan);
          std::cout << "" << lastScan.header.seq << std::endl;
          laser_pub.publish(currentScan);
          std::cout << currentScan.header.seq << std::endl;
          lastPublishedScan = currentScan;
        }
        else
        {
          ROS_ERROR("Failed to call service servo nodding");
          return 1;
        }

        ros::spinOnce();
    };



    assemble_srv.request.end = ros::Time::now();

    if (pointCloudClient.call(assemble_srv))
    {
        ROS_INFO("Scan assembled");

        state_pub.publish(assemble_srv.response.cloud);
    }
    else
    {
      ROS_ERROR("Failed to call service assemble_scans");
      return 1;
    }


    srv.request.channel = 0;
    srv.request.angle = standby_pos;
    srv.request.speed = 10;

    if (client.call(srv))
    {
      ROS_INFO("Movement finished");
    }
    else
    {
      ROS_ERROR("Failed to call service servo nodding");
      return 1;
    }




    res.finished = true;
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserscanner_server");


  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  n_private.param("max_angle", max_angle, -50.0);
  n_private.param("standard_angle", standby_angle, 20.0);
  n_private.param("min_angle", min_angle, 60.0);

  min_pos = ((min_angle * (float)M_PI) / 180.0);
  standby_pos = (( standby_angle * (float)M_PI) / 180.0);
  max_pos = ((max_angle * (float)M_PI) / 180.0);

  client = n.serviceClient<kurt3d::ServoCommand>("servo_node");
  pointCloudClient = n.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");

  // inits the publisher
  state_pub = n.advertise<sensor_msgs::PointCloud2>("uos_3dscans", 1);

  // inits the publisher
  laser_pub = n.advertise<sensor_msgs::LaserScan>("cleaned_scan", 100);

  // inits the subscriber
  ros::Subscriber sub = n.subscribe("scan", 100, scanCallback);

  // inits the service
  ros::ServiceServer service = n.advertiseService("laserscanner_node", scan);


  ROS_INFO("laser scanner server ready to laser scan");
  ros::spin();
  return 0;
}
