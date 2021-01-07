cmake_minimum_required(VERSION 3.0.0)
project(MTc++ VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

 
set(CMAKE_INCLUDE_CURRENT_DIR ON) 

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 

 
 

include(CTest)
enable_testing()
  

add_executable(MTC++ main.cpp Cellstorage.cpp)


find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS iostreams system filesystem)
target_include_directories(MTC++ PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${Boost_INCLUDE_DIRS})

target_compile_features(MTC++ PRIVATE cxx_std_17)
target_link_libraries (MTC++ Eigen3::Eigen)
 
target_link_libraries(MTC++ ${Boost_LIBRARIES})


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
