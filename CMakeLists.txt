cmake_minimum_required(VERSION 2.8)

include(cmake/base.cmake)

set(PROJECT_NAME mc_myfirst_controller)
set(PROJECT_DESCRIPTION "Simple controller")
set(PROJECT_URL https://gite.lirmm.fr/me/mc_myfirst_controller)

setup_project()

#Enable C++11
if(NOT WIN32)
        add_definitions(-std=c++0x)
endif()

set(PKG_CONFIG_ADDITIONAL_VARIABLES "${PKG_CONFIG_ADDITIONAL_VARIABLES} libdir")
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
add_required_dependency(mc_rtc)

add_subdirectory(src)
setup_project_finalize()


