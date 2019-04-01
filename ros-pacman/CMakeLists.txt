cmake_minimum_required(VERSION 2.8.3)
project(pacman)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(Qt5Widgets_DIR "/usr/lib/x86_64-linux-gnu/cmake/Qt5Widgets")
set(Qt5Multimedia_DIR "/usr/lib/x86_64-linux-gnu/cmake/Qt5Multimedia")
set(Qt5SerialPort_DIR "/usr/lib/x86_64-linux-gnu/cmake/Qt5SerialPort")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  message_generation
  Qt5Widgets
  Qt5Gui
  Qt5Core
  Qt5Multimedia
  Qt5SerialPort
  geometry_msgs
)
list(APPEND libs Qt5::Multimedia)

add_message_files(
FILES 
actions.msg
pos.msg
pacmanPos.msg
ghostsPos.msg
cookiesPos.msg
bonusPos.msg
game.msg
performance.msg
)

add_service_files(
  FILES
  mapService.srv
)

generate_messages(
DEPENDENCIES 
std_msgs
geometry_msgs
)

#openGL
find_package(OpenGL REQUIRED)
list(APPEND libs ${OPENGL_LIBRARIES})

catkin_package(
   CATKIN_DEPENDS roscpp rospy roslib message_runtime geometry_msgs
   DEPENDS Qt5Widgets Qt5Gui Qt5Core Qt5Multimedia OpenGL Qt5SerialPort
)

include_directories(include
 ${catkin_INCLUDE_DIRS}
 ${OPENGL_INCLUDE_DIRS}
)

set(ros-pacman-simulator_SRCS
  src/main.cpp
  src/glwidget.cpp
  src/window.cpp
  src/listenmsgthread.cpp
  src/maps.cpp
  src/Ghosts.cpp
  src/Pacman.cpp
  src/Utilities.cpp
)

set(ros-pacman-simulator_HDRS
  include/pacman/glwidget.h
  include/pacman/window.h
  include/pacman/maps.h
  include/pacman/listenmsgthread.h
  include/pacman/Ghosts.h
  include/pacman/Pacman.h
  include/pacman/Utilities.h
)

set(pacman_controller_hw_SRCS
  src/pacman_controller_hw.cpp
)

set(pacman_controller_hw_HDRS
  #Add here header C++ files, if any
  #include/pacman_controller/header.h
)

set(pacman_controller_phone_SRCS
  src/pacman_controller_phone.cpp
)

set(pacman_controller_phone_HDRS
  #Add here header C++ files, if any
  #include/pacman_controller/header.h
)


qt5_wrap_cpp(ros-pacman-simulator_MOCS ${ros-pacman-simulator_HDRS})
qt5_wrap_cpp(pacman_controller_hw_MOCS ${pacman_controller_hw_HDRS})
qt5_wrap_cpp(pacman_controller_phone_MOCS ${pacman_controller_phone_HDRS})

qt5_add_resources(RC_SRCS 
    resources/resources.qrc
    ${SHARED_RC}
)

add_executable(pacman_world ${ros-pacman-simulator_SRCS} ${ros-pacman-simulator_MOCS} ${RC_SRCS})
target_link_libraries(pacman_world Qt5::Widgets ${catkin_LIBRARIES} ${OPENGL_LIBRARIES})
add_dependencies(pacman_world pacman_generate_messages_cpp)

add_executable(pacman_controller_hw ${pacman_controller_hw_SRCS} ${pacman_controller_hw_HDRS})
target_link_libraries(pacman_controller_hw ${catkin_LIBRARIES})
add_dependencies(pacman_controller_hw ${catkin_EXPORTED_TARGETS})

add_executable(pacman_controller_phone ${pacman_controller_phone_SRCS} ${pacman_controller_phone_HDRS})
target_link_libraries(pacman_controller_phone ${catkin_LIBRARIES})
add_dependencies(pacman_controller_phone ${catkin_EXPORTED_TARGETS})
