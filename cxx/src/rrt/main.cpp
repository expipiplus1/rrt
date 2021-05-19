#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

// I is an input_iterator
template <typename I>
std::vector<std::array<float, 2>> rrt(int width, int height, I dataBegin);

void processMap(ros::Publisher &pub,
                const nav_msgs::OccupancyGrid::ConstPtr map) {
  ROS_INFO("Begining RRT");

  nav_msgs::Path msg;
  msg.poses.resize(2);
  const auto res = map->info.resolution;

  const auto path = rrt(map->info.width, map->info.height, map->data.begin());
  msg.poses.reserve(path.size());
  // for( const auto& a : path){
  //   msg.poses.push_back(p);
  // }
  std::transform(path.begin(), path.end(), std::back_inserter(msg.poses),
                 [&](const auto a) {
                   geometry_msgs::PoseStamped p;
                   p.pose.position.x = a[0] * res;
                   p.pose.position.y = a[1] * res;
                   p.pose.position.z = 0;
                   return p;
                 });
  msg.header.frame_id = "map";
  pub.publish(msg);

  ROS_INFO("Finished RRT");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<nav_msgs::Path>("path", 1);

  ros::Subscriber sub = n.subscribe<nav_msgs::OccupancyGrid>(
      "map", 1, [&](const nav_msgs::OccupancyGrid::ConstPtr msg) {
        processMap(pub, msg);
      });
  ros::spin();

  return 0;
}
