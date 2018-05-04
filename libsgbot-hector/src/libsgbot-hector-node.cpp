#include <common/draw.h>
#include <sensor/lidar2d.h>
#include <slam/hector/mapping.h>

#include <ros/init.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/GetMap.h>

#include <libsgbot-drawing/rosdraw.h>

#include <boost/thread.hpp>

sgbot::slam::hector::HectorMapping* mapping = NULL;

const int publish_map_level = 0;

ros::Publisher pose_publisher;
ros::Publisher pose_update_publisher;
ros::Publisher map_publisher;
ros::Publisher map_meta_publisher;

geometry_msgs::PoseWithCovarianceStamped getPoseWithCovariance(const sgbot::Pose2D& pose, const sgbot::la::Matrix<float, 3, 3>& cov, const ros::Time& stamp, const std::string frame_id)
{
  geometry_msgs::PoseWithCovarianceStamped result;

  result.header.stamp = stamp;
  result.header.frame_id = frame_id;

  result.pose.pose.position.x = pose.x();
  result.pose.pose.position.y = pose.y();
  result.pose.pose.orientation.w = cos(pose.theta() * 0.5f);
  result.pose.pose.orientation.z = sin(pose.theta() * 0.5f);

  result.pose.covariance[0] = cov(0, 0);
  result.pose.covariance[7] = cov(1, 1);
  result.pose.covariance[35] = cov(2, 2);
  result.pose.covariance[1] = cov(0, 1);
  result.pose.covariance[6] = cov(0, 1);
  result.pose.covariance[5] = cov(0, 2);
  result.pose.covariance[30] = cov(0, 2);
  result.pose.covariance[11] = cov(1, 2);
  result.pose.covariance[31] = cov(1, 2);

  return result;
}

geometry_msgs::PoseStamped getPose(const sgbot::Pose2D& pose, const ros::Time& stamp, const std::string frame_id)
{
  geometry_msgs::PoseStamped result;

  result.header.stamp = stamp;
  result.header.frame_id = frame_id;

  result.pose.position.x = pose.x();
  result.pose.position.y = pose.y();
  result.pose.orientation.w = cos(pose.theta() * 0.5f);
  result.pose.orientation.z = sin(pose.theta() * 0.5f);

  //printf("%f, %f, %f\n", pose.x(), pose.y(), pose.theta());

  return result;
}


nav_msgs::GetMap::Response getMap(sgbot::Map2D& map2d, const ros::Time& stamp, const std::string frame_id)
{
  nav_msgs::GetMap::Response map;
  // TODO: set origin

  map.map.info.resolution = map2d.getResolution();
  map.map.info.width = map2d.getWidth();
  map.map.info.height = map2d.getHeight();
  map.map.header.frame_id = frame_id;
  map.map.header.stamp = stamp;

  map.map.data.resize(map.map.info.width * map.map.info.height);

  for(int x = 0; x < map.map.info.width; ++x)
  {
    for(int y = 0; y < map.map.info.height; ++y)
    {
      int val = -1;
      if(map2d.isKnown(x, y))
      {
        val = 0;
      }
      else if(map2d.isEdge(x, y))
      {
        val = 100;
      }
    }
  }

  return map;
}

void publishLoop()
{
  ros::Rate r(5.0);
  int publish_times = 0;
  while(ros::ok())
  {
    if(mapping->hasUpdatedMap(publish_map_level))
    {
      sgbot::Map2D map2d = mapping->getMap(publish_map_level);
      nav_msgs::GetMap::Response map_rep = getMap(map2d, ros::Time::now(), "hector_map");
      
      if(publish_times == 0)
      {
        map_meta_publisher.publish(map_rep.map.info);
      }

      map_publisher.publish(map_rep.map);

      publish_times++;
    }

    pose_publisher.publish(getPose(mapping->getPose(), ros::Time::now(), "hector_pose"));
    pose_update_publisher.publish(getPoseWithCovariance(mapping->getPose(), mapping->getPoseCovariance(), ros::Time::now(), "hector_update_pose"));

    r.sleep();
  }
}

void scanCallback(const sensor_msgs::LaserScan& scan)
{
  ros::WallTime start_time = ros::WallTime::now();

  sgbot::sensor::Lidar2D laser;

  laser.clear();

  sgbot::Point2D origin;
  origin.x() = 0.0f;
  origin.y() = 0.0f;

  laser.setOrigin(origin);

  float angle = scan.angle_min;

  for(int i = 0; i < scan.ranges.size(); ++i)
  {
    float dist = scan.ranges[i];
    if((dist > scan.range_min) && (dist < (scan.range_max - 0.1f)))
    {
      laser.addBeam(angle, dist);
    }
    angle += scan.angle_increment;
  }

  mapping->updateByScan(laser);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "libsgbot_hector_node");

  ros::NodeHandle node;

  sgbot::DrawCallbacks* rosdraw = new RosDraw(node);
  rosdraw->reset();
  sgbot::draw.setCallbacks(rosdraw);

  mapping = new sgbot::slam::hector::HectorMapping();

  // init ros node
  ros::Subscriber scan_sub = node.subscribe("scan", 5, &scanCallback);
  pose_publisher = node.advertise<geometry_msgs::PoseStamped>("hector_pose", 1, false);
  pose_update_publisher = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("hector_update_pose", 1, false);
  map_publisher = node.advertise<nav_msgs::OccupancyGrid>("hector_map", 1, true);;
  map_meta_publisher = node.advertise<nav_msgs::MapMetaData>("hector_map_meta", 1, true);

  boost::thread publish_thread = boost::thread(boost::bind(&publishLoop));

  ros::spin();

  delete rosdraw;

  return 0;
}
