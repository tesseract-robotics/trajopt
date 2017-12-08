include_directories(include)
set(JSONCPP_SOURCES
    src/lib_json/json_reader.cpp
    src/lib_json/json_value.cpp
    src/lib_json/json_writer.cpp)
add_library(json ${JSONCPP_SOURCES})
set_target_properties(json PROPERTIES COMPILE_FLAGS -fPIC)
