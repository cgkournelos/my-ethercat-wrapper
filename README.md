README
===================


This README document whatever steps are necessary to get  **my_ehtercat_wrapper** linked with your aplication.

----------

### Description  

This library developed to make easier the communication with some sensors that uses Simple Open EtherCAT Master library.
Version 1.0.0

----------

### Set up guidelines 

This is a C++ shared library. With following steps you can included into your C++ project.

* Dependencies :
> Needs to be installed g++ compiler , and cmake.
> +  $ sudo apt-get install binutils cmake

* Create library:
```bash
# Clone my_ehtercat_wrapper into your pc.
$ cd my_ehtercat_wrapper
$ mkdir build 
$ cd build
$ cmake ..
$ make
```
**Note:**   Now you can find libethercat_soem.so libmy_force_lib.so  files inside folder ,this files is shared libs

* Include library into your own cmake project:
 Paste into your CMakeLists.txt file the code below:   
```
 include_directories( ...path_of_the_library/include)
 add_library( force_lib SHARED IMPORTED)
 set_property(TARGET force_lib PROPERTY IMPORTED_LOCATION path_of_the_library/build/libmy_force_lib.so)
```
 After the add_executable command add :
```
target_link_libraries(my_exe_name force_lib )
```

 ----------
