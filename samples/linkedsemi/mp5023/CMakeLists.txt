cmake_minimum_required(VERSION 3.22)

# 添加这行来指定board overlay文件
set(DTC_OVERLAY_FILE boards/mp5023.overlay)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mp5023_app)

# 添加PMBus库目录到包含路径
include_directories($ENV{ZEPHYR_BASE}/drivers/sensor/pmbus)

# 添加应用程序源文件
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# 确保设备树生成正确
zephyr_library_include_directories(${ZEPHYR_BASE}/include)

# 添加依赖关系，确保PMBus库在MP5023之前编译
add_dependencies(app zephyr_interface)