set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_LIST_DIR}/cmake
)

find_package(catkin REQUIRED COMPONENTS roscpp rosbag cv_bridge tf)

find_package(Boost REQUIRED COMPONENTS system filesystem timer program_options)
find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
find_package(YamlCpp REQUIRED)
find_package(PCL REQUIRED)

###########
## Build ##
###########

include(cmake/enforce-cpp11.cmake)

## Setup QT
set(Qt_INCLUDE_DIRS
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)
set(Qt_LIBRARIES
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
)

set(CMAKE_INCLUDE_CURRENT_DIR ON) # needed to find generated Qt files
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON) # not supported until cmake 3.0.2

## Includes
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Qt_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

## Qt GUI
file(GLOB_RECURSE ui_files src/*.ui)
qt5_wrap_ui(ui_sources ${ui_files})

## Application
file(GLOB_RECURSE sources
    src/*.cpp
    src/*.hpp
)

add_executable(csapex_dataset_labeling_tool
    ${sources}
    ${ui_sources}
)

target_link_libraries(csapex_dataset_labeling_tool
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Qt_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    ${PCL_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS csapex_dataset_labeling_tool
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#############
## Testing ##
#############

