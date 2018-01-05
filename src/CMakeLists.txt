# Eigen
#include_directories( "/usr/include/eigen3" )
# 增加PCL库的依赖
FIND_PACKAGE(PCL REQUIRED COMPONENTS common io)
# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )
# 添加头文件和库文件
ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS})

set(EXECUTABLE_OUTPUT_PATH ../)  									## ../../  最终实际在主工程文件下的build文件夹下(工程的根目录下的build下)
set(LIBRARY_OUTPUT_PATH ../../lib)											## ../../lib  最终出现在主工程文件下的lib文件夹下
ADD_EXECUTABLE(generate_pointcloud generatePointCloud.cpp)
TARGET_LINK_LIBRARIES(generate_pointcloud ${OpenCV_LIBS} ${PCL_LIBRARIES})
