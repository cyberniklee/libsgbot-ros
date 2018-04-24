#include <common/draw.h>
#include <sensor/lidar2d.h>
#include <slam/hector/mapping.h>

#include <ros/init.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/GetMap.h>

#include <libsgbot-drawing/rosdraw.h>

sgbot::DrawCallbacks* rosdraw = NULL;
sgbot::slam::hector::HectorMapping* mapping = NULL;

int last_map_update = 0;

void initialDrawing()
{
  sgbot::draw = new sgbot::Draw();
  rosdraw = new RosDraw();

  rosdraw->reset();

  sgbot::draw->setCallbacks(rosdraw);
}

void finishDrawing()
{
  delete sgbot::draw;
  delete rosdraw;
}

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
  initialDrawing();

  mapping = new sgbot::slam::hector::HectorMapping();

  // init ros node
  ros::init(argc, argv, "libsgbot_hector_node");
  ros::NodeHandle node;
  ros::Subscriber scan_sub = node.subscribe("scan", 5, &scanCallback);

  ros::spin();

  finishDrawing();

  return 0;
}
