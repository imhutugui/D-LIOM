cmake_minimum_required(VERSION 3.1)
project(south-3d-collect)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch urdf configuration_files
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
