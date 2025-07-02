cmake_minimum_required(VERSION 3.10)

set(PROJECT_N surface_match)
project(${PROJECT_N} VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp)

# a macro that gets all of the header containing directories. 
MACRO(header_directories return_list includes_base_folder extention )
    FILE(GLOB_RECURSE new_list ${includes_base_folder}/*.${extention})
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

# using said macro.
header_directories(INCLUDES ${PROJECT_SOURCE_DIR}/include/ hpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/halcon_cpp/include/)
file(GLOB_RECURSE LIBS ${CMAKE_CURRENT_SOURCE_DIR}/halcon_cpp/lib/x64-win64/*.lib)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Eigen/eigen3/)
set(PCL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/PCL_1.10.1/PCL_1.10.1/cmake/")
find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###
# message("src files:")
# foreach(file ${SRCS})
#         message(STATUS ${file})
# endforeach()
# 
# message("include directories:")
# foreach(dir ${INCLUDES})
#         message(STATUS ${dir})
# endforeach()
### 
file(GLOB_RECURSE BOOSTLIBS ${CMAKE_CURRENT_SOURCE_DIR}/deps/PCL_1.10.1/PCL_1.10.1/3rdParty/*.lib)
add_executable(${PROJECT_N} ./src/main.cpp)

# target_include_directories(${PROJECT_N} PUBLIC ${LIBS})
target_link_libraries(${PROJECT_N} PUBLIC ${LIBS} ${PCL_LIBRARIES} ${BOOSTLIBS})