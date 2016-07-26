cmake_minimum_required(VERSION 2.8.3)
project(csapex_dataset)

find_package(catkin REQUIRED COMPONENTS
    csapex
    csapex_core_plugins
)

find_package(Boost REQUIRED COMPONENTS filesystem)

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
)

file(GLOB sources

)

add_library(csapex_dataset_node
    ${sources}
)

target_link_libraries(csapex_dataset_node
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############
