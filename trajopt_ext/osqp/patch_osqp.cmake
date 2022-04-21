file(READ CMakeLists.txt ROOT_FILE)
string(REPLACE "set_target_properties(osqpstatic PROPERTIES OUTPUT_NAME osqp)" "" ROOT_FILE2 ${ROOT_FILE})
file(WRITE CMakeLists.txt ${ROOT_FILE2})

file(READ lin_sys/direct/qdldl/qdldl_sources/CMakeLists.txt QDLDL_FILE)
string(REPLACE "set_target_properties(qdldlstatic PROPERTIES OUTPUT_NAME qdldl)" "" QDLDL_FILE2 ${QDLDL_FILE})
file(WRITE lin_sys/direct/qdldl/qdldl_sources/CMakeLists.txt ${QDLDL_FILE2})
