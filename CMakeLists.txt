cmake_minimum_required(VERSION 2.8.3)
project(ros_envelope)

##################
## Compile options
add_compile_options(-std=c++11)

###################################
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    std_msgs
)

###################
## Get Source Files
set (${PROJECT_NAME}_SRC
    ${PROJECT_SOURCE_DIR}/src/RosEnvelopeInterface.cpp
    )
set (${PROJECT_NAME}_HDRS
    ${PROJECT_SOURCE_DIR}/include/ros_envelope/ros_envelope.h
    ${PROJECT_SOURCE_DIR}/include/ros_envelope/ros_envelope_iface.h
    ${PROJECT_SOURCE_DIR}/include/ros_envelope/ros_envelope_iface_impl.h
    ${PROJECT_SOURCE_DIR}/include/ros_envelope/ros_envelope_template_impl.h
    )

########
## Build
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRC} ${${PROJECT_NAME}_HDRS})

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        ${catkin_LIBRARIES}
)

################################
## Catkin specific configuration
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/ros_enevlope
    ${catkin_INCLUDE_DIRS}
    )

catkin_package(CATKIN_DEPENDS std_msgs
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME})


install(TARGETS ${PROJECT_NAME}
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
       DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       PATTERN ".svn" EXCLUDE)

#######
## Test

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    set (${PROJECT_NAME}_TEST_SRC
        ${PROJECT_SOURCE_DIR}/test/src/RosEnvelopeTest.cpp
    )
    catkin_add_gtest(RosEnvelopeTest ${${PROJECT_NAME}_TEST_SRC})
    target_link_libraries(RosEnvelopeTest ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
