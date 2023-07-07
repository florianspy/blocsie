add_library(${LIBRARY_NAME} FileHandler.cpp  MaterialInterpolator.cpp f_main.cpp)
target_include_directories(${LIBRARY_NAME} PUBLIC 
    "./"
    "${CMAKE_BINARY_DIR}/config_files/include")
