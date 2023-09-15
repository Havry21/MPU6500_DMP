cmake_minimum_required(VERSION 3.0.0)
project(mpu6500 LANGUAGES CXX)
set(CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS -pthread)
# include(CTest)
# enable_testing()

set(LIBS_NAME
    I2Cdev
    sensor
    inv_mpu_lib
    )
add_executable(mpu6500 main.cpp)

foreach(LIB ${LIBS_NAME})
    add_subdirectory(${LIB})
    target_link_libraries(mpu6500 PRIVATE ${LIB})
endforeach(LIB)

