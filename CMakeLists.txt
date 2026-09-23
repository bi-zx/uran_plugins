cmake_minimum_required(VERSION 3.16)
project(uran_plugins VERSION 0.1.0 LANGUAGES NONE)

include(GNUInstallDirs)
install(DIRECTORY protocols motion media sensors tasks common simulation docs
  DESTINATION ${CMAKE_INSTALL_DATADIR}/uran_plugins
  PATTERN "__pycache__" EXCLUDE
  PATTERN "tests" EXCLUDE)
install(FILES README.md DESTINATION ${CMAKE_INSTALL_DOCDIR})
