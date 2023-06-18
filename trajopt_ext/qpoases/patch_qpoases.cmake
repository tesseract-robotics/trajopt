file(READ CMakeLists.txt ROOT_FILE)
string(
  REPLACE "MESSAGE(FATAL_ERROR \"Compiling qpOASES as a shared library in Windows is not supported.\")"
          "set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)\ninclude(GNUInstallDirs)"
          ROOT_FILE2
          ${ROOT_FILE})
string(
  REPLACE "SET(CMAKE_INSTALL_BINDIR \${CMAKE_INSTALL_LIBDIR})"
          ""
          ROOT_FILE3
          ${ROOT_FILE2})
file(WRITE CMakeLists.txt ${ROOT_FILE3})
