# Add the source files for the IQS7211A driver
target_sources(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/iqs7211a.c
)

# Add include directories for the header files
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
