add_library(Sim_Robot STATIC Sim_Robot.cpp)
target_link_directories(Sim_Robot PUBLIC $ENV{WEBOTS_HOME}/lib/controller)
set(LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})

target_link_libraries(Sim_Robot PUBLIC ${LIBRARIES})

target_include_directories(Sim_Robot 
                            PUBLIC $ENV{WEBOTS_HOME}/include/controller/cpp
                            PRIVATE ${PROJECT_SOURCE_DIR}/Sim_Robot
                            )

