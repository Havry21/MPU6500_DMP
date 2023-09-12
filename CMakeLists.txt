cmake_minimum_required(VERSION 3.0.0)
project(mpu6500 VERSION 0.1.0 LANGUAGES C CXX)

# include(CTest)
# enable_testing()

set(LIBS_NAME
    
    I2Cdev
    )

add_executable(mpu6500 main.cpp 
sensor/sensor.cpp
                        inv_mpu_lib/inv_mpu_dmp_motion_driver.cpp 
                        inv_mpu_lib/inv_mpu.cpp)

foreach(LIB ${LIBS_NAME})
    add_subdirectory(${LIB})
    target_link_libraries(mpu6500 PRIVATE ${LIB})
endforeach(LIB)




set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
