ROS node to split side-by-side stereo
=====================================

Subscribes to an image stream of side-by-side stereo where each image
message consists of a left and right image concatenated to form a single
double-wide image.  This node splits the incoming image down the middle
and republishes each half as stereo/left and stereo/right images.

The node also provides a CameraInfoManager for the left and right outputs,
so that it appears to the stereo imaging pipeline and camera
calibration code as two cameras.

This is a modified version of public domain code posted by user PeteBlackerThe3rd
in response to my question on ROS Answers:
https://answers.ros.org/question/315298/splitting-side-by-side-video-into-stereoleft-stereoright/

Migrated to ROS 2. Converts BGR8 to mono8

Parameters
----------

- input\_image\_topic -- ROS topic of concantenated stereo images.
- left\_output\_image\_topic -- defaults to /left/image_raw
- right\_output\_image\_topic -- defaults to /right/image_raw
- left\_camera\_info\_topic -- defaults to /left/camera_info
- right\_camera\_info\_topic -- defaults to /right/camera_info

- left_cam_calibration_file -- URL to left camera calibration yaml file
- right_cam_calibration_file -- URL to right camera calibration yaml file



Disparity Map shown using standard ROS 2 image pipline->stereo_image_proc on intel i7 1165g7 ~15-20fps
![Kazam_screenshot_00018](https://github.com/dirksavage88/side_x_side_stereo/assets/35986980/0f810961-342a-4868-8815-b439af440d22)
Disparity Map shown using isaac ros image pipeline->isaac_ros_stereo_image_proc pipeline ~30fps:
![disparityjetsonnano](https://github.com/dirksavage88/side_x_side_stereo/assets/35986980/02ddedb6-e7d0-441e-9e65-0347a0402e22)

**Example command for stereo splitter:**
----------

- ros2 run side_x_side_stereo side_x_side_stereo_node --ros-args -p output_width:=640 -p output_height:=240 -p input_image_topic:=/image_mono -p left_output_image_topic:=/left/image_raw -p right_output_image_topic:=/right/image_raw -p left_camera_info_topic:=/left/camera_info -p right_camera_info_topic:=/right/camera_info

**Example command to set up a disparity node:**
----------
- ros2 run stereo_image_proc disparity_node --ros-remap --remap /left/image_rect:=/left/image_raw --remap /right/image_rect:=/right/image_raw

License


-------

MIT
