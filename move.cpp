#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <cstdlib>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

float actual_x, actual_y, actual_theta;

void moveCallBack(const nav_msgs::Odometry::ConstPtr & odometry_msg) {
  actual_x = odometry_msg -> pose.pose.position.x;
  actual_y = odometry_msg -> pose.pose.position.y;
  actual_theta = tf::getYaw(odometry_msg -> pose.pose.orientation);
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "move_robot");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("/odom", 100, & moveCallBack);
  ros::Publisher cmd_vel_pub = nh.advertise < geometry_msgs::Twist > ("/cmd_vel", 100);
  ros::Rate frequence(100);
  geometry_msgs::Twist cmd;

  float p_eukl; // euclidean distance
  float kp = 0.12; // weight for linear velocity
  double ka = 0.25; // weight for angular velocity
  geometry_msgs::Point goal;
  float goal_matrix[4][2] = {{1,2},{0,3},{-1,2},{0,0}}; // define destination points
  bool succeed; // check if arrived at destination
  int no_goals = sizeof goal_matrix / sizeof goal_matrix[0];
  
  while (ros::ok()) 
  {
    for (int i = 0; i < no_goals; i++)
    {
      succeed = false;
      while(succeed == false)
      {
          double x = goal_matrix[i][0] - actual_x;
          double y = goal_matrix[i][1] - actual_y;
          p_eukl = sqrt(pow(y, 2) + pow(x, 2)); // calculate p_eukl 
          double angleToGoal = atan2(y, x); // claculate angle to goal
          double angle_dif = angleToGoal - actual_theta; // calculate angle difference

          if (p_eukl < 0.01) // when reached goal
          {
            cmd.linear.x = 0.0;
            succeed = true;

          }
          else 
          {
            if (abs(angle_dif) > 0.1) // turn turtlebot to goal
            {
              cmd.angular.z = abs(angle_dif) * ka;
            }
            // start to move linear
            else 
            {
              cmd.angular.z = 0.0;
              cmd.linear.x = p_eukl * kp; // linear speed           
            }
          }

          cmd_vel_pub.publish(cmd);
          frequence.sleep();
        ros::spinOnce();
      }  
    }
  }
  return 0;
}
