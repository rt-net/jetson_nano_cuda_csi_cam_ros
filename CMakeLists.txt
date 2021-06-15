cmake_minimum_required(VERSION 3.0.2)
project(jetson_nano_cuda_csi_cam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

find_package(CUDA REQUIRED)
find_package(jetson-utils REQUIRED)
message("-- CUDA version: ${CUDA_VERSION}")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

include_directories(${PROJECT_INCLUDE_DIR} ${PROJECT_INCLUDE_DIR}/jetson-utils)
include_directories(/usr/include/gstreamer-1.0 /usr/include/glib-2.0 /usr/include/libxml2 /usr/lib/aarch64-linux-gnu/gstreamer-1.0/include /usr/lib/x86_64-linux-gnu/gstreamer-1.0/include/ /usr/lib/aarch64-linux-gnu/glib-2.0/include/ /usr/lib/x86_64-linux-gnu/glib-2.0/include/)

add_executable(video_stream src/video_stream_node.cpp src/image_converter.cpp)
target_link_libraries(video_stream ${catkin_LIBRARIES} jetson-utils)


#############
## Install ##
#############

foreach(dir launch config)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)

#############
## Testing ##
#############

