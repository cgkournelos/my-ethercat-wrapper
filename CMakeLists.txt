project(my_ehtercat_wrapper)
cmake_minimum_required(VERSION 2.8)

include_directories(
    include
)

add_library(ethercat_soem SHARED
    include/ethercat_soem/ecat.h
    include/ethercat_soem/osal.h
    include/ethercat_soem/oshw.h
    include/ethercat_soem/nicdrv.h
    include/ethercat_soem/ethercattype.h
    include/ethercat_soem/ethercatsoe.h
    include/ethercat_soem/ethercatprint.h
    include/ethercat_soem/ethercatmain.h
    include/ethercat_soem/ethercatfoe.h
    include/ethercat_soem/ethercatdc.h
    include/ethercat_soem/ethercatconfiglist.h
    include/ethercat_soem/ethercatconfig.h
    include/ethercat_soem/ethercatcoe.h
    include/ethercat_soem/ethercatbase.h
    include/ethercat_soem/osal_defs.h
    src/ethercat_soem/ecat.c
    src/ethercat_soem/osal.c
    src/ethercat_soem/oshw.c
    src/ethercat_soem/nicdrv.c
    src/ethercat_soem/ethercatsoe.c
    src/ethercat_soem/ethercatprint.c
    src/ethercat_soem/ethercatmain.c
    src/ethercat_soem/ethercatfoe.c
    src/ethercat_soem/ethercatdc.c
    src/ethercat_soem/ethercatconfig.c
    src/ethercat_soem/ethercatcoe.c
    src/ethercat_soem/ethercatbase.c
)

add_library(my_ehtercat_wrapper SHARED
    include/my_force_lib.h
    src/my_force_lib.cpp
)

target_link_libraries(ethercat_soem pthread rt)
target_link_libraries(my_ehtercat_wrapper ethercat_soem pthread rt)
