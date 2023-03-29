cmake_minimum_required(VERSION 3.15)

include(proj.conf)

find_package(bouffalo_sdk REQUIRED HINTS $ENV{BL_SDK_BASE})

file(GLOB_RECURSE sources "${CMAKE_CURRENT_SOURCE_DIR}/demos/*.c")
target_sources(app PRIVATE ${sources})

sdk_add_include_directories("${CMAKE_CURRENT_SOURCE_DIR}/exported")
sdk_set_main_file(main.c)

project(bl618_m0pdock)
