# Install script for directory: /Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/src/turtlebot3_custom_simulation

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/install/turtlebot3_custom_simulation")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Users/mateolopezv/miniforge3/envs/rosenv/bin/llvm-objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_custom_simulation" TYPE EXECUTABLE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/turtlebot3_custom_simulation")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_custom_simulation/turtlebot3_custom_simulation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_custom_simulation/turtlebot3_custom_simulation")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Users/mateolopezv/miniforge3/envs/rosenv/bin/arm64-apple-darwin20.0.0-strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_custom_simulation/turtlebot3_custom_simulation")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/CMakeFiles/turtlebot3_custom_simulation.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation" TYPE DIRECTORY FILES
    "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/src/turtlebot3_custom_simulation/launch"
    "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/src/turtlebot3_custom_simulation/param"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation/worlds" TYPE DIRECTORY FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/src/turtlebot3_custom_simulation/worlds/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/" TYPE DIRECTORY FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/src/turtlebot3_custom_simulation/include/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/turtlebot3_custom_simulation")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/turtlebot3_custom_simulation")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation/environment" TYPE FILE FILES "/Users/mateolopezv/miniforge3/envs/rosenv/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation/environment" TYPE FILE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation/environment" TYPE FILE FILES "/Users/mateolopezv/miniforge3/envs/rosenv/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation/environment" TYPE FILE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation" TYPE FILE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation" TYPE FILE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation" TYPE FILE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation" TYPE FILE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation" TYPE FILE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_index/share/ament_index/resource_index/packages/turtlebot3_custom_simulation")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation/cmake" TYPE FILE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation/cmake" TYPE FILE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation/cmake" TYPE FILE FILES
    "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_core/turtlebot3_custom_simulationConfig.cmake"
    "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/ament_cmake_core/turtlebot3_custom_simulationConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_custom_simulation" TYPE FILE FILES "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/src/turtlebot3_custom_simulation/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/Users/mateolopezv/Documents/UdeSA/Principios de la Robótica Autónoma/TPs/TP3/ros_ws/build/turtlebot3_custom_simulation/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
