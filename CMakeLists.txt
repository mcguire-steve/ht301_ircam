# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(ht301_ircam)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED TRUE)
set (CMAKE_CXX_EXTENSIONS FALSE)


# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
		    roscpp
		    camera_info_manager
		    dynamic_reconfigure
		    image_transport
		    nodelet
		    sensor_msgs

		    ht301_msgs
		    cv_bridge)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
generate_dynamic_reconfigure_options(cfg/UVCCamera.cfg)

find_package(libuvc REQUIRED)
message(STATUS "libuvc ${libuvc_VERSION_MAJOR}.${libuvc_VERSION_MINOR}.${libuvc_VERSION_PATCH}")

catkin_package(
  CATKIN_DEPENDS
    roscpp
    camera_info_manager
    dynamic_reconfigure
    image_transport
    nodelet
    sensor_msgs
    ht301_msgs
  LIBRARIES ht301_ircam_nodelet
  )

add_definitions(-Dlibuvc_VERSION_MAJOR=${libuvc_VERSION_MAJOR})
add_definitions(-Dlibuvc_VERSION_MINOR=${libuvc_VERSION_MINOR})
add_definitions(-Dlibuvc_VERSION_PATCH=${libuvc_VERSION_PATCH})
include_directories(include ${libuvc_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LINK_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(camera_node src/main.cpp src/camera_driver.cpp src/XthermDll.cpp)
target_link_libraries(camera_node ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(camera_node ${PROJECT_NAME}_gencfg)

add_library(ht301_ircam_nodelet src/nodelet.cpp src/camera_driver.cpp src/XthermDll.cpp)
add_dependencies(ht301_ircam_nodelet ${ht301_ircam_EXPORTED_TARGETS})
target_link_libraries(ht301_ircam_nodelet ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(ht301_ircam_nodelet ${PROJECT_NAME}_gencfg)

install(TARGETS camera_node ht301_ircam_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES ht301_ircam_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
