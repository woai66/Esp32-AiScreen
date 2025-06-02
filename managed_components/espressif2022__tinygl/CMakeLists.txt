idf_component_register(
    SRC_DIRS
        TinyGL/src
    INCLUDE_DIRS
        TinyGL/src
        TinyGL/include
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-maybe-uninitialized)

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})