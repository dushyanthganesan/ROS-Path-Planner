/* A-star path planning node to plan waypoints toward a goal-point 
  navigating through obstacles */
  #include <iostream>
  #include <ros/ros.h>
  #include <tf2_ros/transform_broadcaster.h>
  #include <geometry_msgs/TransformStamped.h>
  #include <geometry_msgs/Pose2D.h>
  #include <tf2/LinearMath/Quaternion.h>
  #include <nav_msgs/Odometry.h>
  #include <cstdio>
  #include <visualization_msgs/Marker.h>

  using namespace std;

  class planningNode {
    private:
    ros::NodeHandle nh;

    ros::Subscriber pose_sub;
    ros::Subscriber vel_sub;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped transformStamped;
 
    float vel, angular_vel;


    
    // Callback for each time the pose subscriber recieves a message
    
    void poseCallback(const geometry_msgs::Pose2D msg) {
      double x = msg.x;
      double y = msg.y;
      double th = msg.theta;
      ros::Time current_time = ros::Time::now();

      geometry_msgs::Quaternion odom_quat;
      tf2::Quaternion tf_quat;
      tf_quat.setRPY(0, 0, th);

      odom_quat.x = tf_quat.getX();
      odom_quat.y = tf_quat.getY();
      odom_quat.z = tf_quat.getZ();
      odom_quat.w = tf_quat.getW();


      geometry_msgs::TransformStamped odom_tf;
      odom_tf.header.stamp = ros::Time::now();
      odom_tf.header.frame_id = "odom";
      odom_tf.child_frame_id = "base_link";

      odom_tf.transform.translation.x = x;
      odom_tf.transform.translation.y = y;
      odom_tf.transform.translation.z = 0.0;
      odom_tf.transform.rotation = odom_quat;

      // send transform
      tf_broadcaster.sendTransform(odom_tf);

      // odom message
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      // position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      // velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = vel;
      odom.twist.twist.linear.y = 0; 
      odom.twist.twist.angular.z = angular_vel;

      // publish odom message
      odom_pub.publish(odom);
    }

    // callback for velocity twist msgs
    void velCallback(const geometry_msgs::Twist msg) {
      vel = msg.linear.x;
      angular_vel = msg.angular.z;
    }


    public:
    planningNode() {
      nh = ros::NodeHandle();

      odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
      pose_sub = nh.subscribe("pose", 10, &planningNode::poseCallback, this);
      vel_sub = nh.subscribe("velocity", 10, &planningNode::velCallback, this);

    }

    void run() {
      // keep running node
      ros::Rate rate(10);
      while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
      }

    }

  };




  int main(int argc, char** argv) {
    // initialize ros
    ros::init(argc, argv, "path_planner_node");

    // initialize node object and run
    planningNode node;
    node.run();

    return 0;
  }