//
// Created by root on 18-4-17.
//

#ifndef _ROS_DRAW_H_
#define _ROS_DRAW_H_
//sgbot
#include <sensor/lidar2d.h>
#include <type/pose2d.h>
#include <type/point2d.h>
#include <linear-algebra/matrix.h>
//ros
#include <ros/node_handle.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>

class RosDraw {
public:
  RosDraw(ros::NodeHandle nh = ros::NodeHandle("~")) : nh_(nh)
  {
    scan_publisher = nh_.advertise<sensor_msgs::LaserScan>("LaserScan",1);
    //rviz can only subscribe PoseStamped
    pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>("Pose",1);
    //the same
    pose_covariance_publisher = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("PoseWithCovariance",1);
    point_publisher = nh_.advertise<geometry_msgs::PointStamped>("Point",1);
  }

  virtual ~RosDraw()
  {
    ROS_INFO("reset");
    reset();
  }

  void displayScan(const sgbot::sensor::Lidar2D& scan)
  {
    int num_readings = scan.getCount();
    int laser_frequency = 40;

    laserScan.angle_min = -1.57;
    laserScan.angle_max = 1.57;
    laserScan.angle_increment = 3.14 / num_readings;
    laserScan.time_increment = (1 / laser_frequency) / (num_readings);
    laserScan.range_min = 0.0;
    laserScan.range_max = 100.0;

    laserScan.ranges.resize(scan.getCount());
    for (int i = 0; i < num_readings; ++i)
    {
      //does Lidar2d need a distance method?
      laserScan.ranges[i] = sgbot::distance(scan.getOrigin(), scan.getPoint(i));
    }
    laserScan.header.frame_id = "map";
  }

  void displayPose(const sgbot::Pose2D& pose)
  {
    geo_pose.pose.position.x = pose.x();
    geo_pose.pose.position.y = pose.y();
    geo_pose.pose.position.z = 0.0;
    geo_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta());

    geo_pose.header.frame_id = "map";
  }

  void displayPoseWithCovariance(const sgbot::Pose2D& pose,const sgbot::la::Matrix<float,3,3>& covariance)
  {
    //initial pose
    poseWithCovarianceStamped.pose.pose.position.x = pose.x();
    poseWithCovarianceStamped.pose.pose.position.y = pose.y();
    poseWithCovarianceStamped.pose.pose.position.z = 0.0;
    poseWithCovarianceStamped.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta());
    //initial covariance
    poseWithCovarianceStamped.pose.covariance[0] = covariance(0,0);//x and x
    poseWithCovarianceStamped.pose.covariance[1] = covariance(0,1);//x and y
    poseWithCovarianceStamped.pose.covariance[6] = covariance(1,0);//y and x
    poseWithCovarianceStamped.pose.covariance[6 + 1] = covariance(1,1);//y and y

    //R,P,Y
    poseWithCovarianceStamped.pose.covariance[5] = covariance(0,2);//x and theta
    poseWithCovarianceStamped.pose.covariance[11] = covariance(1,2);//y and theta
    poseWithCovarianceStamped.pose.covariance[30] = covariance(2,0);//theta and x
    poseWithCovarianceStamped.pose.covariance[31] = covariance(2,1);//theta and y
    poseWithCovarianceStamped.pose.covariance[35] = covariance(2,2);//theta and theta

    poseWithCovarianceStamped.header.frame_id = "map";

  }

  void displayPoint(const sgbot::Point2D& point)
  {
    pointStamped.point.x = point.x();
    pointStamped.point.y = point.y();

    pointStamped.header.frame_id = "map";
  }

  void update()
  {
    scan_publisher.publish(laserScan);
    pose_publisher.publish(geo_pose);
    pose_covariance_publisher.publish(poseWithCovarianceStamped);
    point_publisher.publish(pointStamped);
  }

  void reset()
  {
    scan_publisher.publish(sensor_msgs::LaserScan());
    pose_publisher.publish(geometry_msgs::PoseStamped());
    pose_covariance_publisher.publish(geometry_msgs::PoseWithCovarianceStamped());
    point_publisher.publish(geometry_msgs::PointStamped());
  }
private:
  ros::Publisher scan_publisher;
  ros::Publisher pose_publisher;
  ros::Publisher pose_covariance_publisher;
  ros::Publisher point_publisher;
  //const std::string scan_topic_name,pose_topic_name,pose_covariance_topic_name,point_topic_name;
  ros::NodeHandle nh_;

  sensor_msgs::LaserScan laserScan;
  geometry_msgs::PoseStamped geo_pose;
  geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceStamped;
  geometry_msgs::PointStamped pointStamped;

};


#endif _ROS_DRAW_H_
