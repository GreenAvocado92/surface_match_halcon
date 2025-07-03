cmake_minimum_required(VERSION 3.10)

set(PROJECT_N surface_match)
project(${PROJECT_N} VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(${PROJECT_SOURCE_DIR}/include)
file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# BOOST
find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

message(STATUS "===========================")

# HALCON
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(HALCON_ROOT "/opt/halcon")  # 安装路径
    set(HALCON_ARCH "x64-linux")    # 
    include_directories(${HALCON_ROOT}/include)
    file(GLOB_RECURSE HALCONLIBS ${HALCON_ROOT}/lib/${HALCON_ARCH}/*.so)
    add_compile_options(-finput-charset=UTF-8 -fexec-charset=UTF-8)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(HALCON_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/halcon_cpp)
    set(HALCON_ARCH "x64-win64")    #
    include_directories(${HALCON_ROOT}/include)
    file(GLOB_RECURSE HALCONLIBS ${HALCON_ROOT}/lib/${HALCON_ARCH}/*.lib)
else()
    message(FATAL_ERROR "Unsupported OS: ${CMAKE_SYSTEM_NAME}")
endif()



# file(GLOB_RECURSE BOOSTLIBS ${CMAKE_CURRENT_SOURCE_DIR}/deps/PCL_1.10.1/PCL_1.10.1/3rdParty/*.lib)
add_executable(${PROJECT_N} ./src/main.cpp)

# target_include_directories(${PROJECT_N} PUBLIC ${LIBS})
target_link_libraries(${PROJECT_N} PUBLIC ${PCL_LIBRARIES} ${HALCONLIBS} Boost::program_options)