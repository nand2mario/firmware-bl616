cmake_minimum_required(VERSION 3.15)

set(CONFIG_FREERTOS 1)
set(CONFIG_CHERRYUSB 1)
set(CONFIG_CHERRYUSB_HOST 1)
set(CONFIG_FATFS 1)
set(CONFIG_FATFS_USBH 1)

find_package(bouffalo_sdk REQUIRED HINTS $ENV{BL_SDK_BASE})

sdk_add_include_directories(.)

target_sources(app PRIVATE programmer.c)
sdk_set_main_file(main.c)

project(tangcore-companion)
