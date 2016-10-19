########
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
###########

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )
find_package(PCL REQUIRED COMPONENTS common io visualization filters)
#FIND_PACKAGE( OpenNI2  )


# 添加头文件和库文件
#INCLUDE_DIRECTORIES( /usr/include/ni  )
#LINK_LIBRARIES( /usr/lib )

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package(OpenNI REQUIRED)

IF(OPENNI_INCLUDE_DIR AND OPENNI_LIBRARY_DIR)
MESSAGE(STATUS "Found OpenNI lib")

INCLUDE_DIRECTORIES( ${OPENNI_INCLUDE_DIR}
                     ${OpenCV_INCLUDE_DIRS}
                     ${PCL_INCLUDE_DIRS}
                      )

MESSAGE(STATUS "include dir is" ${OPENNI_INCLUDE_DIR})
MESSAGE(STATUS "lib dir is" ${OPENNI_LIBRARY_DIR})

ADD_EXECUTABLE( kinect1_drive kinect1_drive.cpp )
TARGET_LINK_LIBRARIES( kinect1_drive 
    ${OpenCV_LIBS} 
    ${OPENNI_LIBRARY_DIR}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    )

ENDIF(OPENNI_INCLUDE_DIR AND OPENNI_LIBRARY_DIR)



