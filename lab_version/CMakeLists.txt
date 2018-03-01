cmake_minimum_required(VERSION 2.8.3)
project(lab3)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables

cs_add_executable(path_service3 src/path_service3.cpp)

cs_add_executable(path_client_not_lazy src/path_client_not_lazy.cpp)
cs_install()
cs_export()
    
