cmake_minimum_required(VERSION 3.0.2)
project(rrt)

add_compile_options(-std=c++17)
find_package(catkin REQUIRED roscpp nav_msgs)
catkin_package()

###########
## Build ##
###########

# TODO: Move these somewhere more sensible
include_directories(
  ../..
)

add_executable(rrt main.cpp rrt.cpp)
set_source_files_properties(rrt.cpp PROPERTIES COMPILE_FLAGS -std=c++2a)
target_link_libraries(rrt ${catkin_LIBRARIES})
