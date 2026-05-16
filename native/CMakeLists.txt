cmake_minimum_required(VERSION 3.10)
project(tinympc_jni)

find_package(JNI REQUIRED)

# Point this to the folder that has the "Eigen" folder inside it
# If you followed the previous step, it should be:
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/generated_solver/include/Eigen")

include_directories("${JNI_INCLUDE_DIRS}/generated_solver/")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/generated_solver")

add_library(tinympc_jni SHARED 
    TinyMPCJNI.cpp 
    generated_solver/tinympc/admm.cpp
    generated_solver/tinympc/tiny_api.cpp
    generated_solver/tinympc/rho_benchmark.cpp
)