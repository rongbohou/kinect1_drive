CMAKE_MINIMUM_REQUIRED( VERSION 2.8 ) #设定版本
# 工程名称
PROJECT(kinect1_drive)
SET( CMAKE_CXX_COMPILER "g++") #设定编译器
SET( CMAKE_BUILD_TYPE Release  )
#设定头文件目录
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)

#设定存放编译出来的库文件的目录
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) 

#并且把该目录设为连接目录
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)
#增加子文件夹,ADD_SUBDIRECTORY( source_dir binary_dir), source_dir 当前工程的源文件，binary_dir 为中间二进制和目标二进制存放位置 下指令的lib在 /build/lib
ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src lib)




