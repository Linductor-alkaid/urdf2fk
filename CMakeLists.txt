cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(urdf2fk)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)

# 添加源代码（直接使用项目目录中的tinyxml2）
add_executable(urdf2fk urdf2fk.cpp tinyxml2.cpp)

# 设置包含目录为当前目录，以便找到tinyxml2.h
target_include_directories(urdf2fk PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# 处理编译警告
# 禁用sscanf的安全警告（使用_CRT_SECURE_NO_WARNINGS）
target_compile_definitions(urdf2fk PRIVATE _CRT_SECURE_NO_WARNINGS)

# 在Windows上设置UTF-8编码以避免字符编码警告
if(MSVC)
    target_compile_options(urdf2fk PRIVATE /utf-8)
endif()
