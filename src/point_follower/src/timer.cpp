#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <iostream>

using namespace std;

class Timer {
  private:
  ros::NodeHandle nh;
  ros::Time t_start, t_finish;
  double total;

  ros::Subscriber goal_sub;
  ros::Subscriber plan_sub;
  public:
  Timer() {
    nh = ros::NodeHandle();

    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, &Timer::start, this);
    plan_sub = nh.subscribe<nav_msgs::Path>("global_planner/planner/plan", 1, &Timer::finish, this);
  }

  void start(geometry_msgs::PoseStamped msg) {
    t_start = ros::Time::now();
    cout << t_start << endl;
  }

  void finish(nav_msgs::Path msg) {
    t_finish = ros::Time::now();
    cout << t_finish << endl;
    total = (t_finish - t_start).toSec();
    ROS_INFO("Total time to plan path: %.10f", total);
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
  ros::init(argc, argv, "timer");
  ROS_INFO("initiating timer node");
  Timer timer;
  timer.run();

  return 0;
}