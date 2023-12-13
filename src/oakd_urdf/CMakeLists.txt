cmake_minimum_required(VERSION 3.10.2)
project(oakd_urdf VERSION 2.8.2 LANGUAGES CXX C)


find_package(catkin REQUIRED
)

catkin_package()

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
FILES_MATCHING PATTERN "*.launch")

install(DIRECTORY
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
