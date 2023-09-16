add_library(regulator STATIC)
target_sources(regulator PRIVATE regulator.cpp)
target_link_libraries(regulator PRIVATE sensor )
target_link_libraries(regulator PRIVATE I2Cdev )
