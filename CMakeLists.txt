cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(iqs7211a_sensor)

target_sources(app PRIVATE src/main.c)

# Add the driver source files and include directories
add_subdirectory(src/iqs7211a_driver)
