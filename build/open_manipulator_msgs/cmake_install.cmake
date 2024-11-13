# Install script for directory: /home/onebean/catkin_ws/src/open_manipulator_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/onebean/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_msgs/msg" TYPE FILE FILES
    "/home/onebean/catkin_ws/src/open_manipulator_msgs/msg/JointPosition.msg"
    "/home/onebean/catkin_ws/src/open_manipulator_msgs/msg/KinematicsPose.msg"
    "/home/onebean/catkin_ws/src/open_manipulator_msgs/msg/OpenManipulatorState.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_msgs/srv" TYPE FILE FILES
    "/home/onebean/catkin_ws/src/open_manipulator_msgs/srv/GetJointPosition.srv"
    "/home/onebean/catkin_ws/src/open_manipulator_msgs/srv/GetKinematicsPose.srv"
    "/home/onebean/catkin_ws/src/open_manipulator_msgs/srv/SetJointPosition.srv"
    "/home/onebean/catkin_ws/src/open_manipulator_msgs/srv/SetKinematicsPose.srv"
    "/home/onebean/catkin_ws/src/open_manipulator_msgs/srv/SetDrawingTrajectory.srv"
    "/home/onebean/catkin_ws/src/open_manipulator_msgs/srv/SetActuatorState.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_msgs/cmake" TYPE FILE FILES "/home/onebean/catkin_ws/build/open_manipulator_msgs/catkin_generated/installspace/open_manipulator_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/onebean/catkin_ws/devel/include/open_manipulator_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/onebean/catkin_ws/devel/share/roseus/ros/open_manipulator_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/onebean/catkin_ws/devel/share/common-lisp/ros/open_manipulator_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/onebean/catkin_ws/devel/share/gennodejs/ros/open_manipulator_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/onebean/miniconda3/envs/IRC/bin/python3" -m compileall "/home/onebean/catkin_ws/devel/lib/python3/dist-packages/open_manipulator_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/onebean/catkin_ws/devel/lib/python3/dist-packages/open_manipulator_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/onebean/catkin_ws/build/open_manipulator_msgs/catkin_generated/installspace/open_manipulator_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_msgs/cmake" TYPE FILE FILES "/home/onebean/catkin_ws/build/open_manipulator_msgs/catkin_generated/installspace/open_manipulator_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_msgs/cmake" TYPE FILE FILES
    "/home/onebean/catkin_ws/build/open_manipulator_msgs/catkin_generated/installspace/open_manipulator_msgsConfig.cmake"
    "/home/onebean/catkin_ws/build/open_manipulator_msgs/catkin_generated/installspace/open_manipulator_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_msgs" TYPE FILE FILES "/home/onebean/catkin_ws/src/open_manipulator_msgs/package.xml")
endif()

