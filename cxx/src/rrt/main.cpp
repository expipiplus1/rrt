#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

// I is an input_iterator, can't express that (neatly) in C++17
template <typename I>
std::vector<std::array<float, 2>>
rrt(int width, int height, I dataBegin, I dataEnd,
    std::array<float, 2> initialConfig, std::array<float, 2> goalConfig);

void processMap(ros::Publisher &pub, const geometry_msgs::Point initialConfig,
                const geometry_msgs::Point goalConfig,
                const nav_msgs::OccupancyGrid::ConstPtr map) {
  ROS_INFO("Begining RRT");

  nav_msgs::Path msg;
  msg.poses.resize(2);
  const auto res = map->info.resolution;

  const auto path =
      rrt(map->info.width, map->info.height, map->data.begin(), map->data.end(),
          {static_cast<float>(initialConfig.x),
           static_cast<float>(initialConfig.y)},
          {static_cast<float>(goalConfig.x), static_cast<float>(goalConfig.y)});
  msg.poses.reserve(path.size());
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

// Why doens't ROS generate a constructor for this...
geometry_msgs::Point mkPoint(const float x, const float y, const float z) {
  geometry_msgs::Point ret;
  ret.x = x;
  ret.y = y;
  ret.z = z;
  return ret;
}

//
// Listens on
//
// - /map           : OccupancyGrid
// - /initialConfig : Point ∈ [0..1]^2
// - /goalConfig    : Point ∈ [0..1]^2
//
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<nav_msgs::Path>("path", 1);

  // TODO: learn about ROS's threading model (can multiple callbacks run
  // concurrently), better to be on the safe side for now.
  std::atomic<geometry_msgs::Point> initialConfig;
  initialConfig.store(mkPoint(0, 0, 0));
  std::atomic<geometry_msgs::Point> goalConfig;
  initialConfig.store(mkPoint(1, 1, 0));

  auto initSub = n.subscribe<geometry_msgs::Point>(
      "initialConfig", 1,
      [&](const geometry_msgs::Point::ConstPtr msg) { initialConfig = *msg; });
  auto goalSub = n.subscribe<geometry_msgs::Point>(
      "goalConfig", 1,
      [&](const geometry_msgs::Point::ConstPtr msg) { goalConfig = *msg; });
  auto sub = n.subscribe<nav_msgs::OccupancyGrid>(
      "map", 1, [&](const nav_msgs::OccupancyGrid::ConstPtr msg) {
        processMap(pub, initialConfig.load(), goalConfig.load(), msg);
      });

  ros::spin();

  return 0;
}
