# canny_edge_detection
Simple ros package that subscribes to an image topic and applies a canny edge detection filter from opencv

## dependencies
### ros
roscpp
sensor_msgs
std_msgs
image_transport
cv_bridge
### opencv
Opencv
## build
use catkin_make or catkin build by copying the package into a ros workspace

### params
upper_canny_threshold
lower_canny_threshold

### Platform 
Tested on linux and osx using ros-melodic (you might need to recompile cv_camera from source using the stdc++-11 standard)

### Launch and test
build 
source devel/setup.bash
roslaunch canny_edge_my_face canny_edge_detection.launch

visualize the output_image topic using rqt_image_view or (jsk_image_overlay in rviz)




