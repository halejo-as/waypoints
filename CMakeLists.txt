cmake_minimum_required(VERSION 2.8.3)
project(waypoints)
add_compile_options(-std=c++11 -fPIC)

find_package(catkin REQUIRED COMPONENTS rviz std_msgs geometry_msgs message_generation rosmaster)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
include_directories(${Qt5Widgets_INCLUDE_DIRS})
set(QT_LIBRARIES Qt5::Widgets)
qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})


add_definitions(-DQT_NO_KEYWORDS)

set(waypoints_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

set(HEADERS_FILES
  include/waypoints_tool.h
  include/waypointPanel.h
)

add_message_files(
   FILES
   waypoint_msg.msg
   waypoint_group.msg
   waypointArray.msg
)
add_service_files(
   FILES
   Save_Wp.srv
   Load_Wp.srv
   Run_Wp.srv
   Stop_Wp.srv
   Delete_Wp.srv
   Groups_Wp.srv
   Wp_2_Group.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  waypoints
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
                   rviz
		   message_runtime
)

include_directories(
        include
	${catkin_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

add_definitions(-DQT_NO_KEYWORDS)




QT5_WRAP_CPP(MOC_FILES
  include/waypoints_tool.h
  include/waypointPanel.h 
 
)

qt5_wrap_ui(UIC_FILES
  resource/waypointPanel.ui
)

set(SOURCE_FILES
  src/waypoints_tool.cpp
  src/waypointPanel.cpp 
  ${MOC_FILES}
)



add_executable(waypoints_server src/waypoints_server.cpp)
target_link_libraries(waypoints_server ${catkin_LIBRARIES})
add_dependencies(waypoints_server ${PROJECT_NAME}_gencpp)

add_executable(debug_waypoints src/debug_waypoints.cpp)
target_link_libraries(debug_waypoints ${catkin_LIBRARIES})
add_dependencies(debug_waypoints ${PROJECT_NAME}_gencpp)


include_directories(${waypoints_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES} ${MOC_FILES} ${UIC_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


