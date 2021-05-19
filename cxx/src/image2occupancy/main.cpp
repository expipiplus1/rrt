#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "image2occupancy");
  ros::NodeHandle n;

  std::string imageName;
  ros::NodeHandle("~").getParam("image", imageName);
  if (imageName.empty()) {
    std::cerr << "No parameter 'image' provided" << std::endl;
    return 1;
  }

  std::cerr << "Loading image: " << imageName << std::endl;
  const int desiredComponents = 1;
  int width;
  int height;
  std::unique_ptr<unsigned char, decltype(*stbi_image_free)> imageData(
      stbi_load(imageName.c_str(), &width, &height, nullptr, desiredComponents),
      stbi_image_free);
  if (!imageData) {
    std::cerr << "Unable to load " << imageName << std::endl;
    return 1;
  }

  ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1);
  ros::Rate loop_rate(1);

  while (ros::ok()) {
    nav_msgs::OccupancyGrid msg;
    msg.info.height = height;
    msg.info.width = width;
    msg.info.resolution = 0.01; // meters per pixel
    msg.data.resize(width * height);
    std::copy(imageData.get(), imageData.get() + width * height,
              msg.data.begin());

    ROS_INFO("publishing message");

    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
}
