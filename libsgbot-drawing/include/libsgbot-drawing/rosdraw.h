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
#include <common/draw.h>
//ros
#include <ros/node_handle.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>

class RosDraw : public sgbot::DrawCallbacks {
public:
  RosDraw(ros::NodeHandle nh = ros::NodeHandle("~"))
    : nh_(nh)
  {
    scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("LaserScan",1);
    //rviz can only subscribe PoseStamped
    pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("Pose",1);
    //the same
    pose_covariance_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("PoseWithCovariance",1);
    point_publisher_ = nh_.advertise<geometry_msgs::PointStamped>("Point",1);
  }

  virtual ~RosDraw()
  {
    ROS_INFO("reset draw callbacks");
    reset();
  }

  virtual void drawScan(const sgbot::sensor::Lidar2D& scan)
  {
    int num_readings = scan.getCount();
    int laser_frequency = 40;

    laser_scan_.angle_min = -1.57;
    laser_scan_.angle_max = 1.57;
    laser_scan_.angle_increment = 3.14 / num_readings;
    laser_scan_.time_increment = (1 / laser_frequency) / (num_readings);
    laser_scan_.range_min = 0.0;
    laser_scan_.range_max = 100.0;

    laser_scan_.ranges.resize(scan.getCount());
    for (int i = 0; i < num_readings; ++i)
    {
      //does Lidar2d need a distance method?
      laser_scan_.ranges[i] = sgbot::distance(scan.getOrigin(), scan.getPoint(i));
    }
    laser_scan_.header.frame_id = "map";
  }

  virtual void drawPose(const sgbot::Pose2D& pose)
  {
    geo_pose_.pose.position.x = pose.x();
    geo_pose_.pose.position.y = pose.y();
    geo_pose_.pose.position.z = 0.0;
    geo_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta());

    geo_pose_.header.frame_id = "map";
  }

  virtual void drawPoseWithCovariance(const sgbot::Pose2D& pose,const sgbot::la::Matrix<float,3,3>& covariance)
  {
    //initial pose
    pose_with_covariance_.pose.pose.position.x = pose.x();
    pose_with_covariance_.pose.pose.position.y = pose.y();
    pose_with_covariance_.pose.pose.position.z = 0.0;
    pose_with_covariance_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta());
    //initial covariance
    pose_with_covariance_.pose.covariance[0] = covariance(0, 0);//x and x
    pose_with_covariance_.pose.covariance[1] = covariance(0, 1);//x and y
    pose_with_covariance_.pose.covariance[6] = covariance(1, 0);//y and x
    pose_with_covariance_.pose.covariance[6 + 1] = covariance(1, 1);//y and y

    //R,P,Y
    pose_with_covariance_.pose.covariance[5] = covariance(0, 2);//x and theta
    pose_with_covariance_.pose.covariance[11] = covariance(1, 2);//y and theta
    pose_with_covariance_.pose.covariance[30] = covariance(2, 0);//theta and x
    pose_with_covariance_.pose.covariance[31] = covariance(2, 1);//theta and y
    pose_with_covariance_.pose.covariance[35] = covariance(2, 2);//theta and theta

    pose_with_covariance_.header.frame_id = "map";
  }

  virtual void drawPoint(const sgbot::Point2D& point)
  {
    point_stamp_.point.x = point.x();
    point_stamp_.point.y = point.y();

    point_stamp_.header.frame_id = "map";
  }

  virtual void update()
  {
    scan_publisher_.publish(laser_scan_);
    pose_publisher_.publish(geo_pose_);
    pose_covariance_publisher_.publish(pose_with_covariance_);
    point_publisher_.publish(point_stamp_);
  }

  virtual void reset()
  {
    scan_publisher_.publish(sensor_msgs::LaserScan());
    pose_publisher_.publish(geometry_msgs::PoseStamped());
    pose_covariance_publisher_.publish(geometry_msgs::PoseWithCovarianceStamped());
    point_publisher_.publish(geometry_msgs::PointStamped());
  }
private:
  ros::Publisher scan_publisher_;
  ros::Publisher pose_publisher_;
  ros::Publisher pose_covariance_publisher_;
  ros::Publisher point_publisher_;
  //const std::string scan_topic_name,pose_topic_name,pose_covariance_topic_name,point_topic_name;
  ros::NodeHandle nh_;

  sensor_msgs::LaserScan laser_scan_;
  geometry_msgs::PoseStamped geo_pose_;
  geometry_msgs::PoseWithCovarianceStamped pose_with_covariance_;
  geometry_msgs::PointStamped point_stamp_;
};


#endif  // _ROS_DRAW_H_
