cmake_minimum_required(VERSION 3.0.2)
project(image2occupancy)

# No c++20 support :(
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs)
catkin_package()

###########
## Build ##
###########

add_executable(image2occupancy main.cpp)
target_link_libraries(image2occupancy ${catkin_LIBRARIES})
