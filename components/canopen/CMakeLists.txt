FILE(GLOB_RECURSE app_sources1 ${CMAKE_SOURCE_DIR}/components/canopen/src/*.c)
FILE(GLOB_RECURSE app_sources2 ${CMAKE_SOURCE_DIR}/components/canopen/hardware/*.c)
FILE(GLOB_RECURSE app_sources3 ${CMAKE_SOURCE_DIR}/components/canopen/dictionary/*.c)

idf_component_register(SRCS ${app_sources1} ${app_sources2} ${app_sources3}
                    PRIV_REQUIRES esp_timer
                    REQUIRES driver
                    INCLUDE_DIRS "./inc" "./inc/esp32xxx" "./hardware" "./dictionary")
