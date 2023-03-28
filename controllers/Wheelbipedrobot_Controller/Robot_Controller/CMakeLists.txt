set(EIGEN_PATH D:/Softwares_Installed/eigen/eigen_3_4_0)

add_library(Robot_Controller Robot_Controller.cpp PID.cpp)

target_include_directories(Robot_Controller 
                            PUBLIC  ${EIGEN_PATH}
                            PRIVATE ${PROJECT_SOURCE_DIR}/Robot_Controller
                            PRIVATE ${PROJECT_SOURCE_DIR})