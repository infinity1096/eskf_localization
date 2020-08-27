cmake_minimum_required(VERSION 2.8.3)
project(eskf_localizer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(Eigen3  REQUIRED)

# Add GeographicLib.
add_subdirectory(third_party/GeographicLib)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  third_party/GeographicLib/include
  ##only for testing
  ${catkin_INCLUDE_DIRS}
)

add_library(eskf_localizer_lib
  src/eskf_localizer.cpp
  src/eskf.cpp
  src/imu_processor.cpp
  src/gps_processor.cpp
  src/mag_processor.cpp
  src/initializer.cpp
)

target_link_libraries(
	eskf_localizer_lib
	${catkin_LIBRARIES}
)


target_link_libraries(eskf_localizer_lib
	${EIGEN3_LIBS}
	libGeographiccc
)

