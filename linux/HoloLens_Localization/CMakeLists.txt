cmake_minimum_required(VERSION 2.8.3)
project(hololens_localizer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hololens_localizer
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


include_directories(
  include/anchor_localizer
  include/static_calibration
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

message("----------------------"  ${INCLUDE_DIRS})

add_executable(anchor_localizer
  src/anchor_localizer/main.cpp
  src/anchor_localizer/ICP_module.cpp  
  )

target_link_libraries(anchor_localizer
   ${catkin_LIBRARIES}
)

add_executable(static_calibration
  src/static_calibration/main.cpp
  src/static_calibration/cost_function.cpp
  src/static_calibration/teleop.cpp
  src/static_calibration/calibration.cpp
)

target_link_libraries(static_calibration
   ${catkin_LIBRARIES}
   ceres
)



