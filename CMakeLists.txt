cmake_minimum_required(VERSION 2.8.3)
project(csapex_dataset)

find_package(catkin REQUIRED COMPONENTS
    csapex
    csapex_core_plugins
    csapex_opencv
    csapex_point_cloud
    csapex_evaluation
)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PCL REQUIRED COMPONENTS common io)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    LIBRARIES csapex_dataset
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

file(GLOB sources
    src/people_3D/*.hpp
    src/people_3D/*.cpp
    src/people_visual/*.hpp
    src/people_visual/*.cpp
    src/sand/*.hpp
    src/sand/*.cpp
)

add_library(csapex_dataset_node
    ${sources}
)

target_link_libraries(csapex_dataset_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
)

#############
## Install ##
#############

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_dataset_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

set(BUILD_DATASET_TOOLS 1)

if (BUILD_DATASET_TOOLS)
    add_subdirectory(tools)
endif()
