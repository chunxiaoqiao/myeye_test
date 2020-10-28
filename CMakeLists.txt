cmake_minimum_required(VERSION 3.0)

project(myeye_test VERSION 1.0.0 LANGUAGES C CXX)

# flags

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")

# packages

if(MSVC)
    set(SDK_ROOT "$ENV{MYNTEYES_SDK_ROOT}")
    if(SDK_ROOT)
        message(STATUS "MYNTEYES_SDK_ROOT: ${SDK_ROOT}")
        list(APPEND CMAKE_PREFIX_PATH
                "${SDK_ROOT}/lib/cmake"
                "${SDK_ROOT}/3rdparty/opencv/build"
                )
    else()
        message(FATAL_ERROR "MYNTEYES_SDK_ROOT not found, please install SDK firstly")
    endif()
endif()

#ROS
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        cv_bridge
        )
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES pointclould_test
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)
include_directories(
        include ${catkin_INCLUDE_DIRS}
        # include
        # ${catkin_INCLUDE_DIRS}
)

## mynteye
find_package(mynteye REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")

# When SDK build with OpenCV, we can add WITH_OPENCV macro to enable some
# features depending on OpenCV, such as ToMat().
if(mynteye_WITH_OPENCV)
    add_definitions(-DWITH_OPENCV)
endif()

## OpenCV

# Set where to find OpenCV
#set(OpenCV_DIR "/usr/share/OpenCV")

# When SDK build with OpenCV, we must find the same version here.
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

# pcl
find_package( PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

#sophus
find_package(Sophus REQUIRED)
include_directories(
        ${Sophus_INCLUDE_DIRS})
# targets
include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

## mynteye_demo

add_executable(imu_stamp src/main_imu_stamp.cpp src/util_cv.cc)
target_link_libraries(imu_stamp mynteye ${OpenCV_LIBS})

add_executable(get_imu src/main_get_imu.cpp src/util_cv.cc)
target_link_libraries(get_imu mynteye ${OpenCV_LIBS})

add_executable(get_depth_point src/main_get_depth_point.cpp src/util_cv.cc src/util_pcl.cc )
add_dependencies(get_depth_point ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_depth_point
        mynteye
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(pointclould_test src/main_pointclould_test.cpp)
target_link_libraries(pointclould_test mynteye ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(get_depth
        src/main_get_depth.cpp
        src/util_cv.cc
        src/util_pcl.cc
        src/RGBRectifier.cpp src/RGBRectifier.h
        src/MAKEpoint_clould.cpp src/MAKEpoint_clould.h)
add_dependencies(get_depth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_depth
        mynteye
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(feature_test
        src/main_feature_test.cpp
        src/util_cv.cc
        src/util_pcl.cc
        src/RGBRectifier.cpp src/RGBRectifier.h
        src/MAKEpoint_clould.cpp src/MAKEpoint_clould.h
        src/Feature.h src/Feature.cpp  src/Stereo_Calib.cpp src/Stereo_Calib.h)
add_dependencies(feature_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(feature_test
        mynteye
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
add_executable(public_ros
        src/main_public_ros.cpp
        src/util_cv.cc
        src/util_pcl.cc
        src/RGBRectifier.cpp src/RGBRectifier.h
        src/MAKEpoint_clould.cpp src/MAKEpoint_clould.h)
add_dependencies(public_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(public_ros
        mynteye
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
add_executable(public_ros_test
        src/main_public_ros_test.cpp
        src/util_cv.cc
        src/util_pcl.cc
        src/RGBRectifier.cpp src/RGBRectifier.h
        src/MAKEpoint_clould.cpp src/MAKEpoint_clould.h)
add_dependencies(public_ros_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(public_ros_test
        mynteye
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(get_from_callback
        src/main_get_from_calback.cpp
        src/util_cv.cc
        src/util_pcl.cc
        src/RGBRectifier.cpp src/RGBRectifier.h
        src/MAKEpoint_clould.cpp src/MAKEpoint_clould.h)
add_dependencies(get_from_callback ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_from_callback
        mynteye
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )


add_executable(myeye_and_litipipei
        src/main_myeye_and_litipipei.cpp
        src/util_cv.cc
        src/util_pcl.cc
        src/RGBRectifier.cpp src/RGBRectifier.h
        src/MAKEpoint_clould.cpp src/MAKEpoint_clould.h
        src/Stereo_Calib.cpp src/Stereo_Calib.h)
add_dependencies(myeye_and_litipipei ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(myeye_and_litipipei
        mynteye
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(pub_img
        src/main_pub_img.cpp
        src/util_cv.cc)
add_dependencies(pub_img ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_img
        mynteye
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        )
add_executable(sub_img src/main_sub_img.cpp)
add_dependencies(sub_img ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sub_img ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(sub_img_cloud
        src/main_sub_img_cloud.cpp
        src/RGBRectifier.cpp src/RGBRectifier.h
        src/MAKEpoint_clould.cpp src/MAKEpoint_clould.h
        src/Stereo_Calib.cpp src/Stereo_Calib.h)
add_dependencies(sub_img_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sub_img_cloud
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Sophus_LIBRARIES})


# Build
#   mkdir _build
#   cd _build
#
#   # win
#   cmake -G "Visual Studio 15 2017 Win64" ..
#   msbuild.exe ALL_BUILD.vcxproj /property:Configuration=Release
#
#   .\Release\mynteye_demo.exe
#
#   # unix
#   cmake ..
#   make
#
#   ./mynteye_demo
