#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <sensor_msgs/Imu.h>
#include <iostream>
#include "Eigen/Dense"

double control_period;
double control_time;

Eigen::Quaterniond quaternion_;
Eigen::Vector3d orientation_, angular_velocity_, linear_acceleration_;
Eigen::Matrix3d rotation_matrix_;

ros::Publisher p_imu_roll;
ros::Publisher p_imu_pitch;
ros::Publisher p_imu_yaw;

ros::Publisher p_imu_ang_vel_x;
ros::Publisher p_imu_ang_vel_y;
ros::Publisher p_imu_ang_vel_z;

ros::Publisher p_imu_lin_acc_x;
ros::Publisher p_imu_lin_acc_y;
ros::Publisher p_imu_lin_acc_z;

std_msgs::Float64 m_imu_roll;
std_msgs::Float64 m_imu_pitch;
std_msgs::Float64 m_imu_yaw;

std_msgs::Float64 m_imu_ang_vel_x;
std_msgs::Float64 m_imu_ang_vel_y;
std_msgs::Float64 m_imu_ang_vel_z;

std_msgs::Float64 m_imu_lin_acc_x;
std_msgs::Float64 m_imu_lin_acc_y;
std_msgs::Float64 m_imu_lin_acc_z;


enum Pos
{
    X,
    Y,
    Z,
    RP,
    ROLL = 3,
    PITCH = 4,
    YAW = 5,
    Pos = 6,
    XYZ = 3
};

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    Eigen::Quaterniond quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::Matrix3d direction;
    direction << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    quaternion_ = quat;

    // Angular Velocity [rad/s]
    angular_velocity_(X) = msg->angular_velocity.x;
    angular_velocity_(Y) = - msg->angular_velocity.y;
    angular_velocity_(Z) = - msg->angular_velocity.z;

    // Linear Accceleration [m/s^2]
    linear_acceleration_(X) = msg->linear_acceleration.x;
    linear_acceleration_(Y) = - msg->linear_acceleration.y;
    linear_acceleration_(Z) = - msg->linear_acceleration.z;

    // Rotation Matrix
    rotation_matrix_ = direction * quat.toRotationMatrix();

    // Euler Angle [rad]
    orientation_(Y) = std::asin(-rotation_matrix_(2, 0));
    if (std::cos(orientation_(1)) != 0) { // Check for edge cases
        orientation_(X) = std::atan2(rotation_matrix_(2, 1), rotation_matrix_(2, 2));
        orientation_(Z) = std::atan2(rotation_matrix_(1, 0), rotation_matrix_(0, 0));
        if(orientation_(X) > 0) orientation_(X) = orientation_(X) - 3.141592;
        else orientation_(X) = orientation_(X) + 3.141592;
    } 
    else { // Gimbal lock case (cos(pitch) == 0),  In this case, we can set roll to 0 and calculate yaw differently
        orientation_(X) = 0.0;
        orientation_(Z) = std::atan2(-rotation_matrix_(0, 1), rotation_matrix_(1, 1));
    }

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"microstrain_inertial_listener");
    ros::NodeHandle node_obj;
    ros::Subscriber number_subscriber = node_obj.subscribe("/imu/data", 10, imu_callback, ros::TransportHints().reliable().tcpNoDelay());

    const ros::Duration control_period_(control_period);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time start_time = ros::Time::now();
    ros::Time last_control_time = start_time;

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        ros::Duration elapsed_time = current_time - last_control_time;

        if (elapsed_time >= control_period_)
        {
            // Update the last control time
            last_control_time = current_time;
            control_time += control_period;

            std::cout<<"Euler Angle x :"<<orientation_(X)<<"[rad]"<<std::endl;
            std::cout<<"Euler Angle y :"<<orientation_(Y)<<"[rad]"<<std::endl;
            std::cout<<"Euler Angle z :"<<orientation_(Z)<<"[rad]"<<std::endl;
            std::cout<<std::endl;

            p_imu_roll = node_obj.advertise<std_msgs::Float64>("/imu_roll/",1);
            p_imu_pitch = node_obj.advertise<std_msgs::Float64>("/imu_pitch/",1);
            p_imu_yaw = node_obj.advertise<std_msgs::Float64>("/imu_yaw/",1);
            
            p_imu_ang_vel_x = node_obj.advertise<std_msgs::Float64>("/imu_ang_vel_x/",1);
            p_imu_ang_vel_y = node_obj.advertise<std_msgs::Float64>("/imu_ang_vel_y/",1);
            p_imu_ang_vel_z = node_obj.advertise<std_msgs::Float64>("/imu_ang_vel_z/",1);
            
            p_imu_lin_acc_x = node_obj.advertise<std_msgs::Float64>("/imu_lin_acc_x/",1);
            p_imu_lin_acc_y = node_obj.advertise<std_msgs::Float64>("/imu_lin_acc_y/",1);
            p_imu_lin_acc_z = node_obj.advertise<std_msgs::Float64>("/imu_lin_acc_z/",1);
            
            m_imu_roll.data = orientation_(X); m_imu_pitch.data = orientation_(Y); m_imu_yaw.data = orientation_(Z);
            p_imu_roll.publish(m_imu_roll); p_imu_pitch.publish(m_imu_pitch); p_imu_yaw.publish(m_imu_yaw);
            
            m_imu_ang_vel_x.data = angular_velocity_(X); m_imu_ang_vel_y.data = angular_velocity_(Y); m_imu_ang_vel_z.data = angular_velocity_(Z);
            p_imu_ang_vel_x.publish(m_imu_ang_vel_x); p_imu_ang_vel_y.publish(m_imu_ang_vel_y); p_imu_ang_vel_z.publish(m_imu_ang_vel_z);
            
            m_imu_lin_acc_x.data = linear_acceleration_(X);  m_imu_lin_acc_y.data = linear_acceleration_(Y);  m_imu_lin_acc_z.data = linear_acceleration_(Z);
            p_imu_lin_acc_x.publish(m_imu_lin_acc_x); p_imu_lin_acc_y.publish(m_imu_lin_acc_y); p_imu_lin_acc_z.publish(m_imu_lin_acc_z);

            ros::spinOnce();
            
            // Sleep to enforce the desired control loop frequency
            ros::Duration sleep_time = control_period_ - elapsed_time;
            if (sleep_time > ros::Duration(0))
            {
                sleep_time.sleep();
            }
        }
    }

    return 0;
}