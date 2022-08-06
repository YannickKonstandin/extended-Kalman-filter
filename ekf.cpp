#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

using namespace std;


class EKF_class
{   
    public:

    ros::Subscriber odometry_sub;
    ros::Subscriber laser_sub;
    ros::Publisher correction_pub;
    Eigen::Matrix3f I, R, Q, G, sigma, sigma_D, H, K; 
    Eigen::Vector3f mu, mu_D, z, z_D, map_point;
    Eigen::Vector2f Delta, Distance;
    float q;
    float x_position, y_position;
    double theta;
    string path_file;
    char bar = '|';
    ros::Time Time_1;
    ros::Duration Time_delta;
    double odom_dt;

    void Matrizen()
    {
        I << 1, 0, 0,        0, 1, 0,       0, 0, 1;
        R << 0.00021, 0, 0,      0, 0.00023, 0,      0, 0, 0.00019;
        Q << 0.1, 0, 0,      0, 0.2, 0,     0, 0, 0.3;
        map_point << 0, 2, 0;
        mu << 0, 0, 0;
        sigma << 0, 0, 0,   0, 0, 0,    0, 0, 0;
        
    }

    /* can be used for more more landmarks

    void read_map()
    {
        ifstream myFile;
        string line;
        string MAP;
        myFile.open("/home/yannick/konstandin_ws/src/konstandin/map/my_map.txt"); 
        if(myFile.is_open())
        {
            ROS_INFO("File opened\n");
    
            for(int i = 0; i<5; i++)
            {
                while (myFile >> map_points[i][0] >> bar >> map_points[i][1])
                {
                ROS_INFO("MapPoints: x: %f y: %f",map_points[i][0], map_points[i][1]);
                }
    
        }
        else
        {
            ROS_INFO("Couldn't open file\n");
        }
        myFile.close();
    }
    */
    

    EKF_class(ros::NodeHandle *nh) //constructor
    {
        ROS_INFO("Aufruf EKF-class");
        odometry_sub    = nh -> subscribe("odom", 10, &EKF_class::odomCallback, this);
        laser_sub       = nh -> subscribe("scan", 10, &EKF_class::scanCallback, this);
       
        correction_pub  = nh -> advertise<geometry_msgs::PoseStamped>("correction", 100);
        
        //read_map();
        Matrizen();

    }


    // Prediction
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msgs)
    {   
        //calculate dt
        Time_delta = odom_msgs->header.stamp - Time_1;
        odom_dt = double(Time_delta.sec) + double(Time_delta.nsec)/1000000000;
        Time_1 = odom_msgs->header.stamp;

        //ROS_INFO_THROTTLE(1, "Delta_t: %f", odom_dt);

        // get Data from odometry
        float linear_vel = odom_msgs->twist.twist.linear.x;
        float angular_vel = odom_msgs->twist.twist.angular.z;
        x_position = odom_msgs -> pose.pose.position.x;
        y_position = odom_msgs -> pose.pose.position.y;

        // converting quaternions to get theta
        tf::Quaternion o(
            odom_msgs->pose.pose.orientation.x,
            odom_msgs->pose.pose.orientation.y,
            odom_msgs->pose.pose.orientation.z,
            odom_msgs->pose.pose.orientation.w);
        tf::Matrix3x3 t(o);
        double roll, pitch, yaw;
        t.getRPY(roll, pitch, yaw);
        theta = yaw;

        mu(0) = x_position;
        mu(1) = y_position;
        mu(2) = theta;
        
        // Print Theta in degree
        //ROS_INFO_THROTTLE(1,"Theta: %f", theta * 180/M_PI);
        

        // definition of shift vector
        Eigen::Vector3f shift;
        shift(0) = -(linear_vel/angular_vel) * sin(mu(2)) + (linear_vel/angular_vel) * sin(mu(2) + (angular_vel * odom_dt));
        shift(1) = (linear_vel/angular_vel) * cos(mu(2)) - (linear_vel/angular_vel) * cos(mu(2) + (angular_vel * odom_dt));
        shift(2) = (angular_vel * odom_dt);

        /*
        ROS_INFO_THROTTLE(1,"shift-x: %f", shift(0));
        ROS_INFO_THROTTLE(1,"shift-y: %f", shift(1));
        ROS_INFO_THROTTLE(1,"shift-t: %f", shift(2));
        */

        // calculate mu_D
        mu_D = mu + shift;

        ROS_INFO_THROTTLE(1,"Prediction X: %f; Y: %f; t: %f", mu_D(0), mu_D(1), mu_D(2));

        // definition of G-Matrix
        G(0,0) = 1;
        G(0,1) = 0;
        G(0,2) = ((linear_vel/angular_vel) * cos(mu(2))) - ((linear_vel/angular_vel) * cos(mu(2) + (angular_vel * odom_dt)));
        G(1,0) = 0;
        G(1,1) = 1;
        G(1,2) = ((linear_vel/angular_vel) * sin(mu(2))) - ((linear_vel/angular_vel) * sin(mu(2) + (angular_vel * odom_dt)));
        G(2,0) = 0;
        G(2,1) = 0;
        G(2,2) = 1;

        // calculate sigma_D
        sigma_D = G * sigma * G.transpose() + R;

    }


    // Correction
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msgs)
    {
        float increment = scan_msgs->angle_increment; //angle between scans
        float no_scans = (2 * M_PI) / increment; // get number of scans
        float r = 0.05; // radius of cylinder
        float minimum, min;
        double angle;
        geometry_msgs::PoseStamped correction_pose;

        // quaternions to get theta
        tf::Quaternion quat;
     

        //ROS_INFO("Number of scans: %f\n Distance bewteen angles: %f", no_scans, increment);

        // calculate distance to landmark (x,y)
            for(int i = 0; i < no_scans; i++)
            {
                min = scan_msgs->ranges.at(0);
            
                for (int i = 1; i < no_scans; i++)
                {
                    if(scan_msgs->ranges.at(i) < min)
                    {
                        min = scan_msgs->ranges.at(i);
                        angle = (i * 2* M_PI)/ 360 ; // calculate degree to rad
                        minimum = min + r;
                        ROS_INFO_THROTTLE(1,"Min_Distance: %f", minimum);
                        //ROS_INFO("Minimum: %f", minimum);
                        //ROS_INFO("Theta: %f", theta*180/M_PI);
                        //ROS_INFO("Angle: %f", angle);
                        z(0) = minimum;
                        z(1) = angle;
                        z(2) = map_point(2);

                        
                        
                        if (theta >= 0 && theta < M_PI/2)
                        {   
                            Distance(0) = minimum * cos(M_PI - (angle + theta)); // x distance to landmark laserscan
                            Distance(1) = minimum * sin(M_PI - (angle + theta)); // y distance to landmark  laserscan
                            //ROS_INFO_THROTTLE(1,"X-Distance: %f", Distance(0));
                            //ROS_INFO_THROTTLE(1,"Y-Distance: %f", Distance(1));
                        }               
                        else if (theta >= M_PI/2)
                        {
                            Distance(0) = minimum * cos(M_PI - (angle + theta));
                            Distance(1) = minimum * abs(sin(2 * M_PI - (angle + theta)));
                            //ROS_INFO_THROTTLE(1,"X-Distance: %f", Distance(0));
                            //ROS_INFO_THROTTLE(1,"Y-Distance: %f", Distance(1));
                        }
                        else
                        {
                            Distance(0) = minimum * cos(M_PI - (angle + theta));
                            Distance(1) = minimum * abs(sin(M_PI - (angle + theta)));
                       
                        } 
                    }
                }
            }

            // Print Distance laserscan
            ROS_INFO_THROTTLE(1,"X-Distance: %f", Distance(0));
            ROS_INFO_THROTTLE(1,"Y-Distance: %f", Distance(1));

            // Distance to landmark
            Delta(0) = map_point(0) - mu_D(0);
            Delta(1) = map_point(1) - mu_D(1);
            
            // q Matrix bestimmen (Pseudocode Zeile 9)
            q = Delta.transpose() * Delta;

            // z_Dach bestimmen (Pseudocode Zeile 10)
            z_D(0) = sqrt(q);
            z_D(1) = atan2(Delta(1),Delta(0)) - mu_D(2);
            z_D(2) = 0;

            // Jacobimatrix berechnen (Pseudocode Zeile 11)
            H(0,0) = 1/q * sqrt(q) * Delta(0);
            H(0,1) = -1/q * sqrt(q) * Delta(1);
            H(0,2) = 0;
            H(1,0) = 1/q * Delta(1);
            H(1,1) = 1/q * Delta(0);
            H(1,2) = -1/q;
            H(2,0) = 0;
            H(2,1) = 0;
            H(2,2) = 0;
            
            // Kalman-Gain bestimmen (Pseudocode Zeile 12)       
            K = sigma_D * (H.transpose()) * (H * sigma_D * H.transpose() + Q).inverse();

            // calculate corrected mu and sigma
            mu = mu_D + sigma * K * (z - z_D);  
            sigma = (I - sigma * K * H) * sigma_D;

            ROS_INFO_THROTTLE(1,"Correction X: %f; Y: %f; t: %f", mu(0), mu(1), mu(2));

            correction_pose.header.frame_id = "odom";
            quat.setRPY( 0, 0, mu(2));
            correction_pose.pose.position.x = mu(0);
            correction_pose.pose.position.y = mu(1);
            correction_pose.pose.position.z = mu(2);
            correction_pose.pose.orientation.z = quat.getZ();
            correction_pose.pose.orientation.x = quat.getX();
            correction_pose.pose.orientation.y = quat.getY();
            correction_pose.pose.orientation.w = quat.getW();

            // Correction publishen
            correction_pub.publish(correction_pose);
            

    }
    
};
    

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "EKF");
    ros::NodeHandle nh;
    EKF_class EKF_Filter(&nh);
    ros::param::get("/pathfile",EKF_Filter.path_file);
    ros::spin();
    return 0;
}
