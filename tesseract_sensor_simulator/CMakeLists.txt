cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS tesseract_monitoring)

# libgz-msgs9-dev
# libgz-transport12-dev
# libgz-sensors7-core-dev
# libgz-sensors7-camera-dev
# libgz-sensors7-depth-camera-dev
# libgz-sensors7-rendering-dev
find_package(gz-sensors7 COMPONENTS camera depth_camera rendering REQUIRED)
find_package(gz-transport12 REQUIRED)
find_package(gz-msgs9 REQUIRED)

find_package(tesseract_common REQUIRED)
find_package(tesseract_qt REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
      ${PROJECT_NAME}
    CATKIN_DEPENDS
      tesseract_monitoring
    DEPENDS
      gz-sensors7
      gz-transport12
      gz-msgs9
      tesseract_qt
)

# Sensor Simulator
add_library(${PROJECT_NAME} src/sensor_simulator.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC tesseract::tesseract_common
         tesseract::tesseract_qt_common
         tesseract::tesseract_qt_gazebo_utils
         gz-msgs9::gz-msgs9
         gz-transport12::gz-transport12
         gz-sensors7::gz-sensors7
         gz-sensors7::gz-sensors7-camera
         gz-sensors7::gz-sensors7-depth_camera
         gz-sensors7::gz-sensors7-rendering
         ${catkin_LIBRARIES})
target_include_directories(
  ${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                          "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/sensor_simulator_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_node PRIVATE VERSION 17)
target_include_directories(${PROJECT_NAME}_node PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})


# Mark executables and/or libraries for installation
install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
