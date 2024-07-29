file(READ CMakeLists.txt ROOT_FILE)
string(
  REPLACE "MESSAGE(FATAL_ERROR \"Compiling qpOASES as a shared library in Windows is not supported.\")"
          "set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)\nENDIF()\ninclude(GNUInstallDirs)\nIF(0)\n"
          ROOT_FILE2
          ${ROOT_FILE})
string(REGEX REPLACE "SET\\(CMAKE_INSTALL[^\r\n]*\\)" "" ROOT_FILE3 ${ROOT_FILE2})
string(REGEX REPLACE "set\\([^\r\n]*RPATH[^\r\n]*\\)" "" ROOT_FILE4 ${ROOT_FILE3})
string(REGEX REPLACE "SET\\([^\r\n]*OUTPUT_PATH[^\r\n]*\\)" "" ROOT_FILE5 ${ROOT_FILE4})
string(REPLACE "cmake_minimum_required(VERSION 2.6)" "cmake_minimum_required(VERSION 3.10.0)" ROOT_FILE6 ${ROOT_FILE5}) 

file(WRITE CMakeLists.txt ${ROOT_FILE6})
