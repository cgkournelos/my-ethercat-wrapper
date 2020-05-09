/*********************************************************************
* my_ehtercat_wrapper.h/
*
* Author: Christos Gkournelos
*********************************************************************/
/** @defgroup WRAPPER_EtherCAT WRAPPER_Library_EtherCAT
 *
 * @brief This package contains all the files for the intergration of EtherCAT communication
 *
 * It is developed to give us a better "manipulation"
 * of Force-Sensor values without knowing all the principles
 * behind Open EtherCAT protocol.
 * @n The main Class for use is My_Force_Sensor
 */

/*! @file my_ehtercat_wrapper.h
 * @ingroup WRAPPER_EtherCAT
 * @brief Headerfile for WRAPPER library of Force Sensor connection via EtherCAT
 *
 *
 * @author Christos Gkournelos
 * @n Contact: cgkournelos@gmail.com
 */

#ifndef my_ehtercat_wrapper_H
#define my_ehtercat_wrapper_H

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <cstdlib>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <vector>
#include <cmath>

#include "ethercat_soem/ethercattype.h"
#include "ethercat_soem/nicdrv.h"
#include "ethercat_soem/ethercatbase.h"
#include "ethercat_soem/ethercatmain.h"
#include "ethercat_soem/ethercatdc.h"
#include "ethercat_soem/ethercatcoe.h"
#include "ethercat_soem/ethercatfoe.h"
#include "ethercat_soem/ethercatconfig.h"
#include "ethercat_soem/ethercatprint.h"

#define GENERAL_OUTPUT 0
#define MY_EC_TIMEOUT 500

/**
 * @class My_Force_Sensor
 * @brief Class for implemention of Force-Sensor communication
 *
 * In this Class are wrapped all the appropriate functions
 * for the connection and comunication of one EtherCAT
 * slave. In our case this slave is the Force-Sensor.
 *
 */

struct sensor_values{
    std::vector<float> channel;
    double ch1, ch2, ch3, ch4, ch5, ch6;
};

class My_Force_Sensor{
public:
    /**
     * @brief Main Constructor
     *
     * Main functions that constructor wraps inside
     * \n-open 2 log files
     * \n-call ec_init(char* ifname) with name as input argument
     * \n-call ec_config_init()
     * \n-count the connected slaves (in our application must be 1)
     * \n-call ec_config_map(void *pIOmap)
     * \n-wait for slave to reach at SAFE_OP state
     * \n-calculate workcounter
     * \n-wait for slave to reach at the OP state
     * \n All this operations are monitored by some flags if
     * something went wrong then the object will not constructed correctly.
     *
     * @param Input parameter is the ethernet name  eg. "eth0"
     */

    My_Force_Sensor(char*);
    /**
     * @brief Deconstructor
     *
     * Deletes all pointers and call the ec_close() function.
     */

    ~My_Force_Sensor();
    /**
     * @brief Set the Force-Sensor in OP state
     *
     * This Function is to set the one slave,
     *  that in our case is the Force-Sensor,
     * in OP state.If the slave reach that state within
     * a timeout duration the funtion returns true.
     *
     * @return True when the Force-Sensor is in OP .\n
     * False in any othercase .
     */

    bool set_sensor_op_state();
    /**
     * @brief
     *
     * @return bool
     */

    bool read_sensor_raw_values();

    bool find_init_values();

    bool read_sensor_values(std::vector<float> &);

    bool translation_racer1(std::vector<float> &);

    FILE * f_gen; /**< TODO */
    FILE * f_sens_val; /**< TODO */

private:

    char* _ifname; /**< TODO */

    bool _ec_init_flag = false; /**< TODO */
    bool _ec_config_init_flag = false; /**< TODO */
    bool _in_operational_state = false; /**< TODO */

    char IO_config_map[4096]; /**< TODO */
    int expectedWKC; /**< TODO */
    volatile int wkc; /**< TODO */
    ec_ODlistt *ODlist; /**< TODO */
    ec_OElistt *OElist; /**< TODO */

    sensor_values *raw_sensor_values;   /**< TODO */
    sensor_values *init_values;

    float linear_thres = 12.0;
    float rot_thres    = 0.8;

};


#endif
