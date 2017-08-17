cmake_minimum_required(VERSION 3.4)
project(RobotRaconteur_EGM)

include(RobotRaconteurExamplesCommon.cmake)

find_package(ProtoBuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS egm.proto)

add_executable(RobotRaconteur_EGM
	edu__rpi__robotics__abb__egm.h
	edu__rpi__robotics__abb__egm_stubskel.h
	edu__rpi__robotics__abb__egm_stubskel.cpp
	RobotRaconteur_EGM.h
	RobotRaconteur_EGM.cpp
	${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(RobotRaconteur_EGM ${RobotRaconteur_LIBRARY} ${Boost_LIBRARIES} ${RobotRaconteur_EXTRA_LIBRARIES} ${PROTOBUF_LIBRARIES})
