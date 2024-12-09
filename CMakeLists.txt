cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(urdf2fk)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)

# 添加源代码
add_executable(urdf2fk urdf2fk.cpp tinyxml2.cpp)

# 如果你在系统上安装了 tinyxml2，可以使用 find_package 来链接
find_package(TinyXML2 REQUIRED)

# 如果 tinyxml2 在默认路径中找不到，你可以指定库路径
# target_include_directories(DH2urdf PRIVATE /path/to/tinyxml2)

# 如果是用系统安装的 tinyxml2，则需要链接它
target_link_libraries(urdf2fk PRIVATE tinyxml2)
