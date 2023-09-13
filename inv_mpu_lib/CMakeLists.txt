add_library(inv_mpu_lib STATIC)
target_sources(inv_mpu_lib PRIVATE inv_mpu.cpp inv_mpu_dmp_motion_driver.cpp)
    target_link_libraries(inv_mpu_lib PRIVATE I2Cdev )

 