add_library(plasma INTERFACE)

target_sources(plasma INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/thc.c
 ${CMAKE_CURRENT_LIST_DIR}/linuxcnc.c
 ${CMAKE_CURRENT_LIST_DIR}/sheetcam.c
)
