# Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

cmake_minimum_required(VERSION 3.12)

project(study-air-hockey)

find_package(YARP 3.3.0 REQUIRED COMPONENTS os sig dev math)
find_package(ICUB REQUIRED COMPONENTS ctrlLib)
find_package(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()

add_subdirectory(gazebo)

add_executable(${PROJECT_NAME}-1 ${CMAKE_SOURCE_DIR}/src/study-1.cpp)
target_compile_definitions(${PROJECT_NAME}-1 PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME}-1 ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME}-1 DESTINATION bin)

add_executable(${PROJECT_NAME}-2 ${CMAKE_SOURCE_DIR}/src/study-2.cpp)
target_include_directories(${PROJECT_NAME}-2 PRIVATE /workspace/spline/src)
target_compile_definitions(${PROJECT_NAME}-2 PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME}-2 ${YARP_LIBRARIES} ICUB::ctrlLib)
install(TARGETS ${PROJECT_NAME}-2 DESTINATION bin)

icubcontrib_add_uninstall_target()

file(GLOB conf ${CMAKE_SOURCE_DIR}/app/conf/*.ini ${CMAKE_SOURCE_DIR}/app/conf/*.tsv)
file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
