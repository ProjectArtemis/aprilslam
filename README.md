# AprilSLAM - Mapping and localization from AprilTags

AprilSLAM is a package designed for fast camera pose estimation from a single or multiple AprilTags(link) in an unstructured environment. AprilSLAM does not require to know the position of tags beforehand to estimate the pose of the camera. The system can map multiple tags in the camera's view as long as there is atleast another tag in view to estimate relative tag pose the first time. The system has been run with a downward looking machine vision cameras onboard quadrotors with an ARM-based computing solutions for precise estimation of the vehicle pose. The system is implemented under ROS (Robot Operating System) for ease of integration, but should be easy to run without it as well.

We use the awsome AprilTag implementation of M. Kaess for tag extraction [1,2]. GTSAM [3] is used for the mapping system.

The default AprilTag family used is 36h11 with a black border of 1. A PDF of the tag family is available here : http://www.dotproduct3d.com/assets/pdf/apriltags.pdf

Package originally developed by Chao Qu and Gareth Cross from Kumar Robotics (www.kumarrobotics.org). Currently developed and maintained by M.H.Kabir.

_Note : The black border parameter and tag family is currently hardcoded to 1 and 36h11 respectively. If you want to use other tags, you will need to modify the apriltag-mit library slightly and change the border and other parameters._ : https://github.com/ProjectArtemis/aprilslam/blob/master/apriltag_mit/src/TagFamily.cc#L28

## Installation instructions -

GTSAM is used for the mapping system, and is a required CMake dependency:
```
mkdir ~/deps
cd deps
wget https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam-3.2.1.tgz
tar -xvf gtsam-3.2.1.tgz
cd gtsam-3.2.1
mkdir build
cd build
cmake ..
make install
cd ~/
rm -rf deps
```

The package can be built using the catkin build system, which is standard for ROS. To install:
```
# clone package source
cd catkin_ws/src
git clone https://github.com/ProjectArtemis/aprilslam
cd ..
# get dependencies
rosdep install --from-paths src --ignore-src
# compile pakcage
catkin build
```
To run the full mapping and tag extraction system, first run your camera node (assumed to be /camera). You might need to adjust the launchfile to remap to your *rectified* image topic (`image_rect`) and `camera_info` topic. Then :
```
roslaunch aprilslam slam.launch
```
RViz should show the detections overlayed on the image and pose of the camera in the world frame (origin at first detected tag)

## Tips/Problems

   * Make sure the tag board is as flat as possible. Best is to glue it to a rigid plate such as aluminum or acrylic glass.
   * Most printers will scale the target during the printing process. Make sure to remeasure the tag sizes and adjust the target configuration accordingly.
   * Keep a white border around the target of min. the size of one grid element (or the detection might be unsuccessful in certain lighting conditions)

## References

Please cite the appropriate papers when using this package or parts of it in an academic publication.

    1. Edwin Olson (2011). AprilTag: A robust and flexible visual fiducial system. Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 3400–3407
    2. Michael Kaess. http://people.csail.mit.edu/kaess/apriltags/, Nov. 2013
    3. GTSAM. https://collab.cc.gatech.edu/borg/gtsam/
