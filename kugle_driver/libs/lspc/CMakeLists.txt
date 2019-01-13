# Configure build as release (for maximum optimization) but still add debug information (-g flag)
set(CMAKE_BUILD_TYPE "Release")

message(STATUS "Adding library: lspc [${CMAKE_BUILD_TYPE}]")

set(library_dependencies  ${Boost_LIBRARIES} )

#FILE(GLOB_RECURSE binary_sources *.cpp) # find all cpp files
#FILE(GLOB_RECURSE binary_headers *.hpp) # find all hpp files

set(binary_sources "${CMAKE_CURRENT_SOURCE_DIR}/Socket.cpp")
set(binary_headers "${CMAKE_CURRENT_SOURCE_DIR}/LSPC/lspc/Packet.hpp"
	           "${CMAKE_CURRENT_SOURCE_DIR}/LSPC/lspc/Serializable.hpp"
	           "${CMAKE_CURRENT_SOURCE_DIR}/LSPC/lspc/SocketBase.hpp")


set(GLOBAL_HEADERS ${GLOBAL_HEADERS} ${binary_headers} PARENT_SCOPE)

add_library( lspc SHARED ${binary_sources} ${GLOBAL_HEADERS} )
target_link_libraries( lspc ${library_dependencies} )
target_include_directories( lspc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/LSPC/)

set_target_properties( lspc PROPERTIES COMPILE_FLAGS "-g -Wall -Wextra" ) # enable debugging
