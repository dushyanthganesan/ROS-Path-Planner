/* A-star path planning node to plan waypoints toward a goal-point 
  navigating through obstacles */
#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>


using namespace std;

class Node {
  private:
  float x = 0;
  float y = 0;
  float th = 0;
};


class Astar {
  private:
  ros::NodeHandle nh;
  ros::Subscriber target_sub;
  ros::Publisher pose_pub;
  ros::Publisher vel_pub;
  ros::Publisher robotMarker_pub;
  geometry_msgs::Pose2D pose_msg;
  visualization_msgs::Marker marker_msg;
  float x = 0;
  float y = 0;
  float th = 0;
  float xd, yd, thd;
  float t = 0;

  public:
  Astar() {
    nh = ros::NodeHandle();
    pose_pub = nh.advertise<geometry_msgs::Pose2D>("pose", 10);
    robotMarker_pub = nh.advertise<visualization_msgs::Marker>("visualization/marker", 10);
    
  }

  void planPath(geometry_msgs::PoseStamped msg) {
    xd = msg.pose.position.x;
    yd = msg.pose.position.x;
    thd = msg.pose.orientation.z;

  }

  void publishPose() {
    pose_pub.publish(pose_msg);
    robotMarker_pub.publish(marker_msg);
  }

  void updatePos() {
    t += 0.001;
    ROS_INFO("t = %f", t);
    x = 5*sin(t);
    y = 5 * cos(t);
    th = tan(t);
    
    pose_msg.x = x;
    pose_msg.y = y;
    pose_msg.theta = th;

    // visualization marker
    tf2::Quaternion quat;
    quat.setRPY(0,0,th);
    marker_msg.header.frame_id = "odom";
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.pose.position.x = x;
    marker_msg.pose.position.y = y;
    marker_msg.pose.orientation.z = quat.getZ();
    marker_msg.pose.orientation.w = quat.getW();
    marker_msg.SPHERE;
    marker_msg.color.g = 255;
    marker_msg.color.a = 255;
    marker_msg.scale.x = 1;
    marker_msg.scale.y = 1;
    marker_msg.scale.z = 1;
  }

  void run() {
    // keep running node
    ros::Rate rate(10);
    while (ros::ok()) {
      publishPose();
      updatePos();
      ros::spinOnce();
      rate.sleep();
    }

  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_planner_node");
  ROS_INFO("initiating node");
  Astar planner;
  planner.run();

  return 0;
}