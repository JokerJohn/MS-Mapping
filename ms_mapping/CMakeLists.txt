cmake_minimum_required(VERSION 3.0.2)
project(ms_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Darwin")
    set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
    set(CMAKE_CXX_STANDARD 14)
else ()
    set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
    set(CMAKE_CXX_STANDARD 14) # WE CAN REMOVE THIS
endif ()

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    include(ProcessorCount)
    ProcessorCount(N)
    message("Processer number:  ${N}")
    if (N GREATER 4)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=3)
        message("core for MP: 3")
    elseif (N GREATER 3)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=2)
        message("core for MP: 2")
    else ()
        add_definitions(-DMP_PROC_NUM=1)
    endif ()
else ()
    add_definitions(-DMP_PROC_NUM=1)
endif ()
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# for fastlio
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        rosbag
        std_msgs
        image_transport
        cv_bridge
        tf
        message_generation
        )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Open3D REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)  # for some error
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED QUIET)
# find_package(Gflags REQUIRED)
# find_package(Glog REQUIRED)

###########################################################################
# Find Boost
find_package(Boost REQUIRED COMPONENTS filesystem regex timer date_time program_options)
if (NOT TARGET Boost::boost)
    add_library(Boost::boost INTERFACE IMPORTED include/factors/gravity_factor.h include/factors/gravity_estimator.h include/factors/gravity_kalman_filter.h include/factors/partial_pose_factor.h include/factors/unitSphere_gravityFactor.h include/factors/gravity_fator2.h)
    set_target_properties(Boost::boost PROPERTIES
            INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
            INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif ()
###########################################################################

catkin_package()

include_directories(
        include
        /usr/include/libxml2
        third_parties
        /usr/local/include/yaml-cpp
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${Y_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        ${PYTHON_INCLUDE_DIRS}
        ${Open3D_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${GFLAGS_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
)
link_libraries(-lxml2 yaml-cpp)

include_directories(include)

add_executable(fastlio include/fast_lio/ikd-Tree/ikd_Tree.cpp src/fastlio/laserMapping.cpp src/fastlio/preprocess.cpp)
target_link_libraries(fastlio ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(fastlio PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(ms_mapping
        include/base_type.cpp
        src/data_saver.cpp
        src/cloud_process.cpp
        include/factors/gravity_kalman_filter.h
        src/ms_mapping/ms_mapping.cpp
        src/ms_mapping/ms_mapping_node.cpp
        )
target_link_libraries(ms_mapping ${catkin_LIBRARIES} ${Open3D_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_CXX_FLAGS} ${OpenCV_LIBS}  gtsam)



