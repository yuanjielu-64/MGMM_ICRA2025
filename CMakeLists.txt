CMAKE_MINIMUM_REQUIRED(VERSION  3.20)

PROJECT(Runner CXX C)
SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

IF(NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE Release)
ENDIF()


 SET(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -fpermissive" CACHE STRING  
      "Flags used by the C++ compiler during debug builds." FORCE)
  SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -fpermissive " 
      CACHE STRING "Flags used by the C++ compiler during release builds." FORCE) 
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -fpermissive " CACHE STRING
      "Flags used by the C++ compiler during release builds with debug information." FORCE) 

  SET(CMAKE_C_FLAGS_DEBUG "-g -O0" CACHE STRING
      "Flags used by the C compiler during debug builds." FORCE)
  SET(CMAKE_C_FLAGS_RELEASE "-O3 -Wno-unused-result -DNDEBUG" CACHE STRING
      "Flags used by the C compiler during release builds." FORCE)
  SET(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 -g" CACHE STRING
      "Flags used by the C compiler during release builds with debug information." FORCE)



ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
 
#############################################################################
#Find OpenGL and GLUT
#
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(GLUT REQUIRED)

INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} )


#############################################################################
#Find Other Libs
#
FIND_LIBRARY(PNG_LIB png /usr/lib/ /usr/local/lib/ ./lib/ ${LIBRARY_OUTPUT_PATH}) 
FIND_LIBRARY(DL_LIB dl /usr/lib/ /usr/local/lib/ ./lib/ ${LIBRARY_OUTPUT_PATH}) 

#############################################################################
INCLUDE_DIRECTORIES(src)
AUX_SOURCE_DIRECTORY(src/Utils      SRC_UTILS)
AUX_SOURCE_DIRECTORY(src/External   SRC_EXTERNAL)
AUX_SOURCE_DIRECTORY(src/External/PQP SRC_PQP)
AUX_SOURCE_DIRECTORY(src/Components SRC_COMPONENTS)
AUX_SOURCE_DIRECTORY(src/Planners   SRC_PLANNERS)
AUX_SOURCE_DIRECTORY(src/Programs   SRC_PROGRAMS)
AUX_SOURCE_DIRECTORY(src/CreateScenes SRC_CREATE_SCENES)

ADD_EXECUTABLE(Runner
  ${SRC_UTILS}
  ${SRC_EXTERNAL}
  ${SRC_PQP}
  ${SRC_COMPONENTS}
  ${SRC_PLANNERS}
  ${SRC_PROGRAMS}
  ${SRC_CREATE_SCENES}
  )



MESSAGE("
*****************************************************************************
* EXTERNAL LIBRARIES
* GL      : ${OPENGL_LIBRARIES}
* GLUT   : ${GLUT_glut_LIBRARY}
* PNG_LIB: ${PNG_LIB}
* DL_LIB: ${DL_LIB}
*****************************************************************************
")

set_property(TARGET Runner PROPERTY ENABLE_EXPORTS 1)

include_directories(/usr/local/include/)
include_directories(${PROJECT_BINARY_DIR}/treelite/include)

TARGET_LINK_LIBRARIES(Runner
 ${DL_LIB}
  ${OPENGL_LIBRARIES}
  ${GLUT_glut_LIBRARY}
  ${PNG_LIB}
  m
  pthread
  /usr/local/lib/lib_lightgbm.so
  ${PROJECT_BINARY_DIR}/treelite/build/libtreelite.so
  ${PROJECT_BINARY_DIR}/treelite/build/libtreelite_runtime.so
  stdc++fs)




