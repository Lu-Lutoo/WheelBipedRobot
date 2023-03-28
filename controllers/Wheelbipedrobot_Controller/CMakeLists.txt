cmake_minimum_required(VERSION 3.10)

project(Wheelbipedrobot_Controller VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_subdirectory(Sim_Robot)
add_subdirectory(Robot_Controller)

aux_source_directory(${PROJECT_SOURCE_DIR} SRCs)

add_executable(${PROJECT_NAME} ${SRCs})

target_link_libraries(${PROJECT_NAME} 
                        PUBLIC Sim_Robot
                        PUBLIC Robot_Controller)

target_include_directories(${PROJECT_NAME} 
                            PRIVATE ${PROJECT_SOURCE_DIR}/Robot_Controller
                            PRIVATE ${PROJECT_SOURCE_DIR}/Sim_Robot)
