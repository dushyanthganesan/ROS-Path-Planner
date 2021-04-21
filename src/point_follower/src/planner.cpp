/* A-star path planning node to plan waypoints toward a goal-point 
  navigating through obstacles */
#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include <tf/transform_listener.h>


using namespace std;



class Planner {
  private:
  ros::NodeHandle nh;

  // subscribers
  ros::Subscriber target_sub;
  ros::Subscriber map_sub;
  ros::Subscriber path_sub;

  // publishers
  ros::Publisher map_pub;
  ros::Publisher pose_pub;
  ros::Publisher vel_pub;
  ros::Publisher goal_pub;
  ros::Publisher robotMarker_pub;

  // messages
  geometry_msgs::Pose2D pose_msg;
  nav_msgs::OccupancyGrid map_msg;
  visualization_msgs::Marker marker_msg;

  // robot pose params
  float x = 11.107;
  float y = 8.891;
  float th = 0;
  float xd, yd, thd;
  float t = 0;

  public:
  Planner() {
    nh = ros::NodeHandle();

    map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, &Planner::getMap, this);
    target_sub = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 10, &Planner::goal, this);
    path_sub = nh.subscribe<nav_msgs::Path>("global_planner/planner/plan", 1, &Planner::followPath, this);

    pose_pub = nh.advertise<geometry_msgs::Pose2D>("pose", 10);
    robotMarker_pub = nh.advertise<visualization_msgs::Marker>("visualization/marker", 10);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("global_planner/goal", 10);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("global_planner/costmap/costmap", 10);
  }

 

  void planPath(geometry_msgs::PoseStamped msg) {
    xd = msg.pose.position.x;
    yd = msg.pose.position.x;
    thd = msg.pose.orientation.z;

  }

  void getMap(nav_msgs::OccupancyGrid msg) {
    map_msg = msg;
    map_msg.info.height = 608;
    map_msg.info.width = 566;
    map_pub.publish(map_msg);
  }

  void goal(geometry_msgs::PoseStamped msg) {
    goal_pub.publish(msg);
  }

  void publishPose() {
    pose_pub.publish(pose_msg);
    robotMarker_pub.publish(marker_msg); 
  }

  void updatePos() {
    
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
    marker_msg.type = visualization_msgs::Marker::CUBE;
    marker_msg.color.b = 1;
    marker_msg.color.a = 1;
    marker_msg.scale.x = 0.2;
    marker_msg.scale.y = 0.2;
    marker_msg.scale.z = 0.2;
    publishPose();
  }

  void followPath(nav_msgs::Path path) {
    ros::Rate rate(100);
    for (size_t i = 0; i < path.poses.size(); ++i) {
      x = path.poses[i].pose.position.x;
      y = path.poses[i].pose.position.y;
      th = path.poses[i].pose.orientation.z;

      updatePos();
      rate.sleep();
    }
  }

  void run() {
    // keep running node
    ros::Rate rate(10);
    while (ros::ok()) {
      updatePos();
      ros::spinOnce();
      rate.sleep();
    }

  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_planner_node");
  ROS_INFO("initiating planner node");
  Planner planner;
  planner.run();

  return 0;
}