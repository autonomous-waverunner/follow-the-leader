cmake_minimum_required (VERSION 2.6)
project (follow-the-leader)

set(PATH_ALG "../../model/s-functions/path_alg")
set(EIGEN "../../model/s-functions/lib/eigen-eigen-323c052e1731")
set(LPFILTER "../../model/s-functions/lib/lp-filter")
get_filename_component(SBGECOM "../sbgECom" ABSOLUTE)

include_directories(
  inc
  ${PATH_ALG}/inc
  ${EIGEN}
  ${SBGECOM}/src
  ${SBGECOM}/src/protocol
  ${SBGECOM}/src/commands
  ${SBGECOM}/src/binaryLogs
  ${SBGECOM}/common
  ${SBGECOM}/common/platform
  ${SBGECOM}/common/crc
  ${LPFILTER}
)

file(GLOB_RECURSE SRC ${SBGECOM}/src/*.c)
file(GLOB_RECURSE COMMON_SRC ${SBGECOM}/common/*.c)
get_filename_component(win_file "${SBGECOM}/common/interfaces/sbgInterfaceSerialWin.c" ABSOLUTE)
list(REMOVE_ITEM COMMON_SRC ${win_file})
add_library(sbgECom STATIC ${SRC} ${COMMON_SRC})

find_library(libconfig config++)
find_library(libproj proj)

set(CMAKE_CXX_FLAGS "-Wall")

file(GLOB SOURCES
  src/*.cpp
  )

add_executable(ftl
  ${SOURCES}
  "${PATH_ALG}/src/path_alg.cpp"
  "${PATH_ALG}/src/pid.cpp"
  "${PATH_ALG}/src/dist_to_path.cpp"
  "${LPFILTER}/LowPassFilter.cpp"
)
target_link_libraries(ftl
  ${libconfig}
  ${libproj}
  sbgECom
  -pthread
)

