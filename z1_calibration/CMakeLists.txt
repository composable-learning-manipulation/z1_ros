cmake_minimum_required(VERSION 3.0.2)
project(z1_calibration)

find_package(catkin)
catkin_python_setup()
catkin_package()

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS scripts/camera_reconfigure.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS scripts/capture_poses.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
