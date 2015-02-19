cmake_minimum_required(VERSION 2.8.3)
project(rose_shared_variables)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	luctor_classes
	rose_conversions
)


find_package(Boost REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
    INCLUDE_DIRS 
    	include
    LIBRARIES 
    	rose_shared_variables
    CATKIN_DEPENDS 
		roscpp 
		luctor_classes
    DEPENDS 
)

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} )

add_library(rose_shared_variables SHARED 
	src/shared_variable.cpp
	src/common.cpp 
)

target_link_libraries(rose_shared_variables ${catkin_LIBRARIES})

add_executable(test_read_client  
	src/test_read_client.cpp
)

target_link_libraries(test_read_client ${catkin_LIBRARIES} rose_shared_variables)

add_executable(test_write_client  
	src/test_write_client.cpp
)

target_link_libraries(test_write_client ${catkin_LIBRARIES} rose_shared_variables)

add_executable(test_server  
	src/test_server.cpp
)

target_link_libraries(test_server ${catkin_LIBRARIES} rose_shared_variables)

add_executable(bool_shared_variable_server  
	src/bool_shared_variable_server.cpp
)

target_link_libraries(bool_shared_variable_server ${catkin_LIBRARIES} rose_shared_variables)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-utest test/test.cpp)
# if(TARGET ${PROJECT_NAME}-utest)
#   target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME})
# endif()
