cmake_minimum_required(VERSION 3.16.3...3.20)
project(el5206_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY launch scripts config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS scripts/el5206_example.py  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

