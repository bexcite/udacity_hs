/*
 * Copyright (c) 2010, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <cmath>

#define PICKUP_X -0.2
#define PICKUP_Y -4.0

#define DROPOFF_X -5.6
#define DROPOFF_Y 0.6

#define TOLERANCE 0.5

bool picking_up = true;
bool dropping_off = false;

ros::Publisher marker_pub;
visualization_msgs::Marker marker;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

  ROS_INFO("Odom received! Processing ...");

  float odom_x = msg->pose.pose.position.x;
  float odom_y = msg->pose.pose.position.y;

  float dx;
  float dy;
  float dist;

  if (picking_up) {
    ROS_INFO("Checking drop off ...");

    dx = odom_x - PICKUP_X;
    dy = odom_y - PICKUP_Y;
    dist = std::sqrt(dx*dx+dy*dy);
    if (dist < TOLERANCE && picking_up) {
      // we are at the pickup location
      ROS_INFO("Picking UP object");

      // Issue pickup-delete marker
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);

      // Wait 5 seconds before trying to show other markers
      ros::Duration(5.0).sleep();

      picking_up = false;
      dropping_off = true;
      return;
    }

  } else if (dropping_off) {
    ROS_INFO("Checking drop off ...");

    dx = odom_x - DROPOFF_X;
    dy = odom_y - DROPOFF_Y;
    dist = std::sqrt(dx*dx+dy*dy);

    if (dist < TOLERANCE) {
      // we are at the pickup location
      ROS_INFO("Dropping OFF object");

      marker.pose.position.x = DROPOFF_X;
      marker.pose.position.y = DROPOFF_Y;

      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);

      ros::Duration(5.0).sleep();
      picking_up = false;
      dropping_off = false;
      return;
    }
  }

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);

  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);


  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = PICKUP_X;
  marker.pose.position.y = PICKUP_Y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  marker_pub.publish(marker);




  ros::Duration(5.0).sleep();

  // Delete marker

  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);

  ros::Duration(5.0).sleep();

  // Show at drop off zone

  marker.pose.position.x = DROPOFF_X;
  marker.pose.position.y = DROPOFF_Y;
  marker.pose.position.z = 0;

  marker.action = visualization_msgs::Marker::ADD;

  marker_pub.publish(marker);




  // ros::Subscriber odom_sub = n.subscribe("/odom", 10, odomCallback);

  ros::spin();


}
