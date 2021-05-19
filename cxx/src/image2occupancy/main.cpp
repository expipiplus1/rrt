#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

//
// Call with:
//
// `rosrun image2occupancy image2occupancy _image:=images/tunnel.png`
//
int main(int argc, char **argv) {
  ros::init(argc, argv, "image2occupancy");
  ros::NodeHandle n;

  // TODO: find out why "~" is necessary here..
  std::string imageName;
  ros::NodeHandle("~").getParam("image", imageName);
  if (imageName.empty()) {
    std::cerr << "No parameter 'image' provided" << std::endl;
    return 1;
  }

  ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1);
  ros::Rate loop_rate(0.1);

  while (ros::ok()) {
    ROS_INFO("Loading image \"%s\"", imageName.c_str());
    const int desiredComponents = 1;
    int width;
    int height;
    std::unique_ptr<unsigned char, decltype(*stbi_image_free)> imageData(
        stbi_load(imageName.c_str(), &width, &height, nullptr,
                  desiredComponents),
        stbi_image_free);
    if (!imageData) {
      ROS_INFO("Unable to load \"%s\"", imageName.c_str());
    } else {
      ROS_INFO("Loaded \"%s\"", imageName.c_str());
      nav_msgs::OccupancyGrid msg;
      msg.info.height = height;
      msg.info.width = width;
      msg.info.resolution = 0.01; // meters per pixel
      msg.data.resize(width * height);
      std::copy(imageData.get(),
                imageData.get() + width * height * desiredComponents,
                msg.data.begin());

      pub.publish(msg);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }
}
