/*********************************************************************
* my_ehtercat_wrapper.cpp/
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, 
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of University of Patras nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Christos Gkournelos
*********************************************************************/

/*! \file my_ehtercat_wrapper.cpp
 * \ingroup WRAPPER_EtherCAT
 * \brief Source File for WRAPPER library of Force Sensor connection via EtherCAT
 *
 * \author Christos Gkournelos
 * \n Contact: cgkournelos@gmail.com
 */

#include "my_ehtercat_wrapper.h"


My_Force_Sensor::My_Force_Sensor(char* name){
    _ifname = name;

    f_gen = fopen("logs/fsensor_general_events.log","w+");
    f_sens_val = fopen("logs/fsensor_values.log","w+");

    raw_sensor_values = new sensor_values;
    raw_sensor_values->channel.resize(7,0);
    init_values = new sensor_values;
    init_values->channel.resize(7,0);


    if(ec_init(_ifname)){
        _ec_init_flag = true;
        #if GENERAL_OUTPUT
        printf("EtherCat initialization on %s succeeded.\n",_ifname);
        #endif
        fprintf(f_gen,"EtherCat initialization on %s succeeded.\n",_ifname);
    }
    else {
        #if GENERAL_OUTPUT
        printf("No socket connection on %s\nExcecute as root\n",_ifname);
        #endif
        fprintf(f_gen,"No socket connection on %s\nExcecute as root\n",_ifname);

    }

    if ( (ec_config_init(FALSE) > 0) && _ec_init_flag ){
        #if GENERAL_OUTPUT
        printf("%d slaves found.\n",ec_slavecount);
        #endif
        fprintf(f_gen,"%d slaves found.\n",ec_slavecount);

        if(ec_slavecount == 1){
            _ec_config_init_flag = true;
        }
    }
    else {
        #if GENERAL_OUTPUT
        printf("No slaves found!\n");
        #endif
        fprintf(f_gen,"No slaves found!\n");

    }

    if(_ec_config_init_flag){
        ec_config_map(&IO_config_map);
        ec_configdc();
        #if GENERAL_OUTPUT
        printf("Slave mapped, state to SAFE_OP.\n");
        #endif
        fprintf(f_gen,"Slave mapped, state to SAFE_OP.\n");

        /* wait for slave to reach SAFE_OP state */
        ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);

        expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
        #if GENERAL_OUTPUT
        printf("Calculated workcounter %d\n", expectedWKC);
        #endif
        fprintf(f_gen,"Calculated workcounter %d\n", expectedWKC);
    }

    if(set_sensor_op_state()){
        if (ec_slave[0].state == EC_STATE_OPERATIONAL ){
            #if GENERAL_OUTPUT
            printf("Slave state to OP.\n");
            #endif
            fprintf(f_gen,"Slave state to OP.\n");

            _in_operational_state = true;

        }
    }
    else{
        #if GENERAL_OUTPUT
        printf("Failed to set the slave in OP state\n");
        #endif
        fprintf(f_gen,"Failed to set the slave in OP state\n");
    }

}

My_Force_Sensor::~My_Force_Sensor(){
    if (_ec_init_flag){
        if(_ec_config_init_flag){
            #if GENERAL_OUTPUT
            printf("\nRequest init state for slave\n");
            #endif
            fprintf(f_gen,"\nRequest init state for slave\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        ec_close();
    }

    fclose(f_gen);
    fclose(f_sens_val);
}

bool My_Force_Sensor::set_sensor_op_state(){
    int false_tries_cnt = 0;
    int false_tries = 50;

    if(_ec_config_init_flag){
        ec_slave[0].state = EC_STATE_OPERATIONAL;
        /* send one valid process data to make outputs in slave happy*/
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        /* request OP state for  slave */
        ec_writestate(0);
        /* wait for slave to reach OP state */
        do{
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
            false_tries_cnt++;
            if(false_tries_cnt == false_tries){
                return false;
            }
        }while((ec_slave[0].state != EC_STATE_OPERATIONAL));

        return true;
    }
}

bool My_Force_Sensor::read_sensor_raw_values(){
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    if(wkc >= expectedWKC){
        ODlist = new ec_ODlistt();
        ODlist->Entries = 0;

        if( ec_readODlist(1, ODlist)){
//            for(int i = 0 ; i < ODlist->Entries ; i++){
                int i = 21;
                ec_readODdescription(i, ODlist);
                while(EcatError) ec_elist2string();
                if( ODlist->Index[i] == 0x6130){
                    OElist = new ec_OElistt();
                    ec_readOE(i, ODlist, OElist);
                    while(EcatError) ec_elist2string();

                    for (int j = 1; j < 7 ; j++){
                        int k=0;
                        if ((OElist->DataType[j] > 0) && (OElist->BitLength[j] > 0)){
                            if ((OElist->ObjAccess[j] & 0x0007)){
                                int tbuff_size;
                                float tbuffer;
                                tbuff_size = sizeof(tbuffer);
                                tbuffer = 0x00;
                                ec_SDOread(1, ODlist->Index[i], j, FALSE,
                                           &tbuff_size, &tbuffer, EC_TIMEOUTRXM);

                                this->raw_sensor_values->channel.at(k) = tbuffer;

                                switch (j) {
                                case 1:
                                    raw_sensor_values->ch1 = tbuffer;

                                    break;
                                case 2:
                                    raw_sensor_values->ch2 = tbuffer;
                                    break;
                                case 3:
                                    raw_sensor_values->ch3 = tbuffer;
                                    break;
                                case 4:
                                    raw_sensor_values->ch4 = tbuffer;
                                    break;
                                case 5:
                                    raw_sensor_values->ch5 = tbuffer;
                                    break;
                                case 6:
                                    raw_sensor_values->ch6 = tbuffer;
                                    break;
                                default:
                                    break;
                                }

                                fprintf(f_sens_val,"%f\t",raw_sensor_values->channel.at(k));
//                                std::cout << "Ch" << j << " : " << raw_sensor_values->channel.at(k) <<"\t";

                                k++;
                            }
                        }
                    }
                    fprintf(f_sens_val,"\n");
//                    std::cout<<"\n";

                    delete OElist;
                    delete ODlist;

                    return true;
                }
//            }
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}

bool My_Force_Sensor::find_init_values(){
    double temp_values1[6] = {0};
    double temp_values2[6] = {0};
    for(int i=0;i< 100;i++ ){
        if(read_sensor_raw_values()){
//            std::cout<< "\n" << this->raw_sensor_values->ch1 << ", "
//                             << this->raw_sensor_values->ch2 << ", "
//                             << this->raw_sensor_values->ch3 << ", "
//                             << this->raw_sensor_values->ch4 << ", "
//                             << this->raw_sensor_values->ch5 << ", "
//                             << this->raw_sensor_values->ch6 << std::endl;;

                temp_values1[0] = temp_values1[0] + raw_sensor_values->ch1;
                temp_values1[1] = temp_values1[1] + raw_sensor_values->ch2;
                temp_values1[2] = temp_values1[2] + raw_sensor_values->ch3;
                temp_values1[3] = temp_values1[3] + raw_sensor_values->ch4;
                temp_values1[4] = temp_values1[4] + raw_sensor_values->ch5;
                temp_values1[5] = temp_values1[5] + raw_sensor_values->ch6;

        }
        else{
            return false;
        }
    }

    init_values->ch1 = temp_values1[0] / 100 ;
    init_values->ch2 = temp_values1[1] / 100 ;
    init_values->ch3 = temp_values1[2] / 100 ;
    init_values->ch4 = temp_values1[3] / 100 ;
    init_values->ch5 = temp_values1[4] / 100 ;
    init_values->ch6 = temp_values1[5] / 100 ;


    std::cout << "Init values : " << init_values->ch1 << ", "
                                  << init_values->ch2 << ", "
                                  << init_values->ch3 << ", "
                                  << init_values->ch4 << ", "
                                  << init_values->ch5 << ", "
                                  << init_values->ch6 << std::endl;
    fprintf(f_gen,"Init values %f\t%f\t%f\t%f\t%f\t%f\n",
            init_values->ch1,
            init_values->ch2,
            init_values->ch3,
            init_values->ch4,
            init_values->ch5,
            init_values->ch6);

    return true;
}

bool My_Force_Sensor::read_sensor_values(std::vector<float> &rvalues){
    bool ret = false;
    float  tvalues[6];

    ret = read_sensor_raw_values();
    if(ret){
        if(raw_sensor_values->ch1 > (init_values->ch1 + linear_thres) )
            rvalues.at(0) = 5.0;
        else if(raw_sensor_values->ch1 < (init_values->ch1 - linear_thres) )
            rvalues.at(0) = -5.0;
        else
            rvalues.at(0) = 0;
        if(raw_sensor_values->ch2 > (init_values->ch2 + linear_thres) )
            rvalues.at(1) = 5.0;
        else if(raw_sensor_values->ch2 < (init_values->ch2 - linear_thres) )
            rvalues.at(1) = -5.0;
        else
            rvalues.at(1) = 0;
        if(raw_sensor_values->ch3 > (init_values->ch3 + linear_thres) )
            rvalues.at(2) = 5.0;
        else if(raw_sensor_values->ch3 < (init_values->ch3 - linear_thres) )
            rvalues.at(2) = -5.0;
        else
            rvalues.at(2) = 0;
        if(raw_sensor_values->ch4 > (init_values->ch4 + rot_thres) )
            rvalues.at(3) = 1.0;
        else if(raw_sensor_values->ch4 < (init_values->ch4 - rot_thres) )
            rvalues.at(3) = -1.0;
        else
            rvalues.at(3) = 0;
        if(raw_sensor_values->ch5 > (init_values->ch5 + rot_thres) )
            rvalues.at(4) = 1.0;
        else if(raw_sensor_values->ch5 < (init_values->ch5 - rot_thres) )
            rvalues.at(4) = -1.0;
        else
            rvalues.at(4) = 0;
        if(raw_sensor_values->ch6 > (init_values->ch6 + rot_thres) )
            rvalues.at(5) = 1.0;
        else if(raw_sensor_values->ch6 < (init_values->ch6 - rot_thres) )
            rvalues.at(5) = -1.0;
        else
            rvalues.at(5) = 0;


        return ret;
    }
    else{
        return ret;
    }

}

bool My_Force_Sensor::translation_racer1(std::vector<float> &rvalues){
    std::vector<float> tvalues;
    tvalues.resize(6);

    tvalues.at(0) = rvalues.at(0);
    tvalues.at(1) = rvalues.at(1);
    tvalues.at(2) = rvalues.at(2);
    tvalues.at(3) = rvalues.at(3);
    tvalues.at(4) = rvalues.at(4);
    tvalues.at(5) = rvalues.at(5);


    rvalues.at(0) =   tvalues.at(2);
    rvalues.at(1) = - tvalues.at(0);
    rvalues.at(2) =   tvalues.at(1);
    rvalues.at(3) =   tvalues.at(4);
    rvalues.at(4) =   tvalues.at(3);
    rvalues.at(5) =   tvalues.at(5);

    return true;
}
