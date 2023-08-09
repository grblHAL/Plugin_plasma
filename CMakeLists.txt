add_library(plasma INTERFACE)

target_sources(plasma INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/thc.c
)

target_include_directories(plasma INTERFACE ${CMAKE_CURRENT_LIST_DIR})
