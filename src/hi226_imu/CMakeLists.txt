
# HI226 IMU

cmake_minimum_required(VERSION 2.8.3)
project(hi226_imu)


add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    sensor_msgs 
    tf 
    cmake_modules
)
find_package(Eigen REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


catkin_package(
)

add_executable(hi226_imu
    src/Imu.cpp  
    src/imu_data_decode.c   include/hi226_imu/imu_data_decode.h
    src/packet.c    include/hi226_imu/packet.h
)

target_link_libraries(hi226_imu
    ${catkin_LIBRARIES}
)
