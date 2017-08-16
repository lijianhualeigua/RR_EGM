cmake_minimum_required(VERSION 2.8.12)
project(RR_EGM)

SET (Boost_ADDITIONAL_VERSIONS 1.60.0 1.60)
SET (Boost_USE_STATIC_LIBS ON)
SET (Boost_USE_MULTITHREADED ON)
SET (Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS date_time filesystem system regex chrono atomic thread REQUIRED)
add_definitions(-DBOOST_ALL_NO_LIB)

find_package(RobotRaconteur REQUIRED)
find_package(Protobuf REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX)
	find_package(OpenSSL)	
	set (RobotRaconteur_EXTRA_LIBRARIES ${RobotRaconteur_EXTRA_LIBRARIES} ${OPENSSL_LIBRARIES} pthread rt z)	
endif()

if(CMAKE_GENERATOR STREQUAL Xcode)

	set(OPENSSL_USE_STATIC_LIBS TRUE)
	find_package(OpenSSL)	

	include(CMakeFindFrameworks)
	set(CMAKE_XCODE_ATTRIBUTE_GCC_VERSION "com.apple.compilers.llvm.clang.1_0")
	set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
	CMAKE_FIND_FRAMEWORKS(CoreFoundation)
	CMAKE_FIND_FRAMEWORKS(Security)
	include_directories(${CoreFoundation_FRAMEWORKS}/Headers ${Security_FRAMEWORKS}/Headers)	
endif()

include_directories(include ${Boost_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS} ${RobotRaconteur_INCLUDE_DIRS})

add_executable(RR_EGM EGM.cpp egm.pb.cc EGM_Service.cpp example__RR_EGM_stubskel.cpp RR_EGM.cpp stdafx.cpp)

target_link_libraries(RR_EGM ${RobotRaconteur_LIBRARY} ${Boost_LIBRARIES} ${RobotRaconteur_EXTRA_LIBRARIES} ${PROTOBUF_LIBRARIES})

