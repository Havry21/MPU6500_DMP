add_library(I2Cdev STATIC)
target_sources(I2Cdev PRIVATE I2Cdev.cpp)
target_include_directories(I2Cdev PUBLIC I2Cdev.h)