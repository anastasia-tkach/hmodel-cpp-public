find_package(Ceres CONFIG REQUIRED)

include_directories(${CERES_INCLUDE_DIRS}})
include_directories(${MINIGLOG_INCLUDE_DIR})
include_directories(${CERES_EXPORTED_SOURCE_DIR})

LIST(APPEND LIBRARIES ${CERES_LIBRARIES})