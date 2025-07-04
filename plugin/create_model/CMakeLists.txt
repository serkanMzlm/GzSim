cmake_minimum_required(VERSION 3.10)
project(create_model)

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};$ENV{MAVLINK_PATH}")


find_package(MAVLink REQUIRED)
find_package(gz-cmake3 REQUIRED)

set(GZ_DISTRIBUTION "Harmonic")

gz_find_package(gz-sim8 REQUIRED)
gz_find_package(gz-msgs10 REQUIRED)
gz_find_package(sdformat14 REQUIRED)
gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
gz_find_package(gz-transport13 VERSION 13.1 REQUIRED COMPONENTS log parameters)

set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

include_directories(include)
add_library(CreateModelPlugin SHARED
  src/CreateModelPlugin.cpp
  src/udp.cpp
)

target_link_libraries(CreateModelPlugin
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-transport${GZ_TRANSPORT_VER}::core

  MAVLink::mavlink
)

install(TARGETS CreateModelPlugin DESTINATION lib)