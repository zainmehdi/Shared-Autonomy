cmake_minimum_required(VERSION 2.8.3)
project(opencv_test)


## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS

        image_transport
        cv_bridge
        roscpp
        rospy
        std_msgs
        visp_ros
        tf
)
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)

#message(STATUS" Opencv dir:" ${OpenCV_INCLUDE_DIRS})
#message(STATUS" Opencv lib:"${OpenCV_LIBRARIES})
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)




###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${VISP_INCLUDE_DIRS}
        ${visp_ros_INCLUDE_DIRS})




## Declare a C++ library
add_library(local_autonomylib
        src/Feature_Extractor/feature_extractor.cpp
        )


## Specify libraries to link a library or executable target against
target_link_libraries(local_autonomylib
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        )

add_library(shared_autonomylib
        src/Shared_Autonomy.cpp
        )


## Specify libraries to link a library or executable target against
target_link_libraries(shared_autonomylib
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES shared_autonomylib local_autonomylib ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}
        CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs
       # DEPENDS system_lib
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_opencv_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#LINK_DIRECTORIES(/usr/local/lib)


add_executable(test123 src/test.cpp)
target_link_libraries(test123 ${catkin_LIBRARIES} )

add_executable(ROI src/ROI.cpp )
target_link_libraries(ROI ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(segmentation src/segmentation.cpp )
target_link_libraries(segmentation ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(VC src/Video_capture_from_live_camera.cpp )
target_link_libraries(VC ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(MIL src/MIL_Tracker.cpp )
target_link_libraries(MIL ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(SURF src/Surf_matcher.cpp )
target_link_libraries(SURF ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(Surf_detection src/Surf_detection.cpp)
target_link_libraries(Surf_detection ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(Mouse src/Mouse_event_handler.cpp)
target_link_libraries(Mouse ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(Local_Autonomy src/Local_Autonomy.cpp)
target_link_libraries(Local_Autonomy
        local_autonomylib
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}

        )

add_executable(visp_test src/visp_test.cpp )
target_link_libraries(visp_test ${catkin_LIBRARIES} ${VISP_LIBRARIES})

add_executable(ST src/simple_tracker.cpp)
target_link_libraries(ST ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(STD src/SparseToDenseOF.cpp)
target_link_libraries(STD ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(Pioneer_IBVS src/Pioneer_IBVS.cpp)
target_link_libraries(Pioneer_IBVS ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(Pioneer_Simple src/pioneer_simple_approach.cpp)
target_link_libraries(Pioneer_Simple ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(Obstacle_Avoidance src/Obstacle_avoidance_node.cpp)
target_link_libraries(Obstacle_Avoidance ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(LK_demo src/Lk_demo.cpp)
target_link_libraries(LK_demo ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(IPM src/Birds_Eye_View.cpp)
target_link_libraries(IPM ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(Birds_Eye src/Birds_Eye_View2.cpp)
target_link_libraries(Birds_Eye ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(Camera_Calibration src/Camera_Calibration.cpp)
target_link_libraries(Camera_Calibration ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(Pioneer_Final_IPM src/Pioneer_Final_IPM.cpp)
target_link_libraries(Pioneer_Final_IPM ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(Pioneer_Simple_Original src/Pioneer_Simple_approach_original.cpp)
target_link_libraries(Pioneer_Simple_Original ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(Pioneer_Simple_Transformation src/Pioneer_Simple_Transformation.cpp)
target_link_libraries(Pioneer_Simple_Transformation ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(Shared_Autonomy src/Shared_Autonomy_main.cpp)
target_link_libraries(Shared_Autonomy
        shared_autonomylib
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}

        )

add_executable( gms_match_demo ./src/demo.cpp )
target_link_libraries( gms_match_demo ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable( tvl_opticalflow_demo ./src/tvl_opticalflow.cpp )
target_link_libraries( tvl_opticalflow_demo ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable( kcf_try ./src/kcf.cpp )
target_link_libraries( kcf_try ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )