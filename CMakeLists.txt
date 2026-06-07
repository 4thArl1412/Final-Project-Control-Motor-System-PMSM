cmake_minimum_required(VERSION 3.31)
project(Final_Project C)

set(CMAKE_C_STANDARD 23)

find_package(Matlab REQUIRED COMPONENTS MEX_COMPILER)
find_package(Eigen3 REQUIRED)

# Define the specific output folder for MEX files
set(MEX_OUTPUT_DIR "${CMAKE_SOURCE_DIR}")
file(MAKE_DIRECTORY ${MEX_OUTPUT_DIR})

add_executable(Final_Project Speed_CTRL.cpp)
target_link_libraries(Final_Project Eigen3::Eigen)
target_include_directories(Final_Project PRIVATE "D:/Download/Clion/lib/eigen-5.0.0/Eigen")

# Add a global PRE_BUILD command to kill MATLAB processes (to release MEX file locks)
add_custom_target(kill_matlab_before_build ALL
    COMMAND ${CMAKE_COMMAND} -P "${CMAKE_SOURCE_DIR}/kill_matlab.cmake"
    COMMENT "Closing MATLAB to release MEX file locks..."
)

# Include Directory for s function code
include_directories("C:/Program Files/MATLAB/R2024a/simulink/include")
include_directories("C:/Program Files/MATLAB/R2024a/rtw/c/src")
include_directories("C:/Program Files/MATLAB/R2024a/toolbox/rtw/accel/accelTemplateFolder")
include_directories("C:/Program Files/MATLAB/R2024a/extern/include")

# List your C source files and desired MEX names (parallel lists)
set(MEX_SOURCE_FILES
    "IPMSM.c"
    "SVPWM.c"
    "RFOC.c"
)

set(MEX_NAMES
    "IPMSM"
    "SVPWM"
    "RFOC"
        "MTPA_RNN"
)

# Loop to build multiple MEX files
list(LENGTH MEX_SOURCE_FILES num_files)
math(EXPR num_files "${num_files} - 1")

foreach(idx RANGE 0 ${num_files})
    list(GET MEX_SOURCE_FILES ${idx} SOURCE_FILE)
    list(GET MEX_NAMES ${idx} MEX_NAME)

    # Add the MEX target
    matlab_add_mex(
        NAME ${MEX_NAME}
        SRC ${SOURCE_FILE}
    )

    # Set the specific folder for this output
    set_target_properties(${MEX_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${MEX_OUTPUT_DIR}"
        RUNTIME_OUTPUT_DIRECTORY "${MEX_OUTPUT_DIR}"
    )

    # Add pre-link command to delete old MEX file before rebuilding (with retry)
    add_custom_command(TARGET ${MEX_NAME} PRE_LINK
        COMMAND ${CMAKE_COMMAND} -P "${CMAKE_SOURCE_DIR}/delete_mex_file.cmake" "${MEX_OUTPUT_DIR}/${MEX_NAME}.mexw64"
        COMMENT "Removing old MEX file: ${MEX_NAME}.mexw64"
    )

    # Make sure MATLAB is closed before building
    add_dependencies(${MEX_NAME} kill_matlab_before_build)
endforeach()
