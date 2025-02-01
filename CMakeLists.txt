CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
SET(CMAKE_CXX_STANDARD 20)
SET(CXX_STANDARD_REQUIRED ON)
PROJECT(FibrationTrees VERSION 0.1)

EXECUTE_PROCESS(COMMAND whoami OUTPUT_VARIABLE current_user OUTPUT_STRIP_TRAILING_WHITESPACE)
MESSAGE("USER: ${current_user}")

ADD_COMPILE_DEFINITIONS("PROGRAM_FOLDER_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

ENABLE_TESTING()

################################################################################
## DEBUG BUILD
SET(CMAKE_CXX_FLAGS "-g")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror -Wno-error=deprecated-declarations")

################################################################################
INCLUDE(GoogleTest)

FIND_PACKAGE(Boost 1.40 COMPONENTS program_options REQUIRED)
SET(DEPENDENCIES ${DEPENDENCIES} boost)

FIND_PACKAGE(yaml-cpp REQUIRED)
SET(DEPENDENCIES ${DEPENDENCIES} yaml-cpp)

FIND_PACKAGE(ompl REQUIRED)
SET(DEPENDENCIES ${DEPENDENCIES} OMPL)

FIND_PACKAGE(DART 6.12.1 REQUIRED COMPONENTS gui-osg utils utils-urdf collision-bullet)
SET(DEPENDENCIES ${DEPENDENCIES} DART)

SET(DART_LIBRARIES ${DART_LIBRARIES})
LIST(REMOVE_DUPLICATES DART_LIBRARIES)

FOREACH(DEPENDENCY ${DEPENDENCIES})
  MESSAGE("-- ${DEPENDENCY}")
  MESSAGE("VERSION   :  ${${DEPENDENCY}_VERSION}")
  MESSAGE("LIBRARIES :  ${${DEPENDENCY}_LIBRARIES} ${${DEPENDENCY}_LIBRARY_DIR}")
  MESSAGE("LIBS      :  ${${DEPENDENCY}_LIBS}")
  MESSAGE("INCLUDE   :  ${${DEPENDENCY}_INCLUDE_DIRS} ${${DEPENDENCY}_INCLUDE_DIR} ${${DEPENDENCY}_INCLUDE_DIRECTORIES}")
  INCLUDE_DIRECTORIES(SYSTEM ${${DEPENDENCY}_INCLUDE_DIRS})
  INCLUDE_DIRECTORIES(SYSTEM ${${DEPENDENCY}_INCLUDE_DIR})
  ADD_DEFINITIONS(${${DEPENDENCY}_DEFINITIONS})
ENDFOREACH(DEPENDENCY)

INCLUDE_DIRECTORIES(SYSTEM inc)

################################################################################
# Build library and dependencies
################################################################################
FOREACH(SOURCETYPE cpp hpp)
  FILE(GLOB_RECURSE SUBDIR_SRC src/*.${SOURCETYPE})
  SET(SOURCE_FILES ${SOURCE_FILES} ${SUBDIR_SRC})
ENDFOREACH(SOURCETYPE)

ADD_LIBRARY(FibrationTrees STATIC ${SOURCE_FILES})
FOREACH(DEPENDENCY ${DEPENDENCIES})
  TARGET_LINK_LIBRARIES(FibrationTrees ${${DEPENDENCY}_LIBRARIES})
ENDFOREACH(DEPENDENCY)
TARGET_LINK_LIBRARIES(FibrationTrees yaml-cpp)
TARGET_LINK_LIBRARIES(FibrationTrees Boost::program_options)

################################################################################
# Build executables
################################################################################
FILE(GLOB EXECUTABLES_SOURCES executables/*.cpp)
LIST(SORT EXECUTABLES_SOURCES)
FOREACH(EXEC_FILENAME ${EXECUTABLES_SOURCES})
  GET_FILENAME_COMPONENT(EXEC ${EXEC_FILENAME} NAME_WE)
  MESSAGE("-- Add Executable ${EXEC}")
  ADD_EXECUTABLE(${EXEC} ${EXEC_FILENAME})
  TARGET_LINK_LIBRARIES(${EXEC} FibrationTrees)
ENDFOREACH(EXEC_FILENAME)

################################################################################
# Build tests
################################################################################
FILE(GLOB TEST_SOURCES tests/*.cpp)
LIST(SORT TEST_SOURCES)

FOREACH(EXEC_FILENAME ${TEST_SOURCES})
  GET_FILENAME_COMPONENT(EXEC ${EXEC_FILENAME} NAME_WE)
  MESSAGE("-- Add Test ${EXEC}")
  ADD_EXECUTABLE(${EXEC} ${EXEC_FILENAME})
  ADD_TEST(NAME ${EXEC} COMMAND ${EXEC})
  TARGET_LINK_LIBRARIES(${EXEC} gtest gtest_main FibrationTrees)
  TARGET_COMPILE_DEFINITIONS(${EXEC} PRIVATE -DGTEST_BREAK_ON_FAILURE=1)
  SET_TESTS_PROPERTIES(${EXEC} PROPERTIES LABELS "unit")
  SET_TESTS_PROPERTIES(${EXEC} PROPERTIES TIMEOUT 30)
ENDFOREACH(EXEC_FILENAME)

################################################################################
# Build integration tests
################################################################################
FILE(GLOB INTEGRATION_TEST_SOURCES tests/integration/*.cpp)
LIST(SORT INTEGRATION_TEST_SOURCES)

FOREACH(EXEC_FILENAME ${INTEGRATION_TEST_SOURCES})
  GET_FILENAME_COMPONENT(EXEC ${EXEC_FILENAME} NAME_WE)
  MESSAGE("-- Add Integration Test ${EXEC}")
  ADD_EXECUTABLE(${EXEC} ${EXEC_FILENAME})
  ADD_TEST(NAME ${EXEC} COMMAND ${EXEC})
  TARGET_LINK_LIBRARIES(${EXEC} gtest gtest_main FibrationTrees)
  TARGET_COMPILE_DEFINITIONS(${EXEC} PRIVATE -DGTEST_BREAK_ON_FAILURE=1)

  SET_TESTS_PROPERTIES(${EXEC} PROPERTIES LABELS "integration")
  SET_TESTS_PROPERTIES(${EXEC} PROPERTIES TIMEOUT 300)
ENDFOREACH(EXEC_FILENAME)

ADD_CUSTOM_TARGET(integration-tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure -L integration
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

## Run only unit tests on executing "make test"
set(CMAKE_CTEST_ARGUMENTS --output-on-failure -L unit)
