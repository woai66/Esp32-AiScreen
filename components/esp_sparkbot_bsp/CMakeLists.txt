#IDF version is less than IDF5.0
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0")
    set(SRC_VER "esp_sparkbot_bsp_idf4.c")
else()
    set(SRC_VER "esp_sparkbot_bsp_idf5.c")
endif()

idf_component_register(
    SRCS "esp_sparkbot_bsp.c" ${SRC_VER}
    INCLUDE_DIRS "include" "include/bsp"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver spiffs touch_element
    PRIV_REQUIRES fatfs esp_lcd 
)
