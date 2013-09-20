/*  $Date: 2009/10/23 $
 *  $Revision: 1.2 $
 */

/*
 * BMP085 Pressure Sensor API
 * Copyright (C) 2009 Bosch Sensortec GmbH and (C) 2012 Rober Bosch LLC 
 *
 * \section intro_sec Introduction
 * BMP085 digital Altimeter Programming Interface
 * The BMP085 API enables quick access to Bosch Sensortec's digital altimeter.
 * The only mandatory steps are: 
 *
 * 1. linking the target application's communication functions to the API
 *    (\ref BMP085_WR_FUNC_PTR, \ref BMP085_RD_FUNC_PTR)
 *
 * 2. calling the bmp085_init() routine, which initializes all necessary data
 *    structures for using all functions
 *
 *
 * 
 * \section license_sec License
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License and the following
 * stipulations. The Apache License , Version 2.0 is applicable unless 
 * otherwise stated by the stipulations of the disclaimer below. 
 * 
 * You may obtain a copy of the License at 
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * \section disclaimer_sec Disclaimer
 *
 * Common:
 * This Work is developed for the consumer goods industry. It may only be used 
 * within the parameters of the respective valid product data sheet.  The Work 
 * provided with the express understanding that there is no warranty of fitness
 * for a particular purpose.  It is not fit for use in life-sustaining, safety
 * or security sensitive systems or any system or device that may lead to bodily
 * harm or property damage if the system or device malfunctions. In addition,
 * the Work is not fit for use in products which interact with motor vehicle
 * systems.  The resale and/or use of the Work are at the purchaser's own risk
 * and his own responsibility. The examination of fitness for the intended use
 * is the sole responsibility of the Purchaser. 
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims,
 * including any claims for incidental, or consequential damages, arising from
 * any Work or Derivative Work use not covered by the parameters of the
 * respective valid product data sheet or not approved by Bosch Sensortec and
 * reimburse Bosch Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased Work and Derivative
 * Works, particularly with regard to product safety and inform Bosch Sensortec
 * without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary
 * from the valid technical specifications of the product series. They are
 * therefore not intended or fit for resale to third parties or for use in end
 * products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series.
 * Bosch Sensortec assumes no liability for the use of engineering samples. By
 * accepting the engineering samples, the Purchaser agrees to indemnify Bosch
 * Sensortec from all claims arising from the use of engineering samples.
 *
 * Special:
 * This Work and any related information (hereinafter called "Information") is
 * provided free of charge for the sole purpose to support your application
 * work. The Work and Information is subject to the following terms and
 * conditions: 
 *
 * The Work is specifically designed for the exclusive use for Bosch Sensortec
 * products by personnel who have special experience and training. Do not use
 * this Work or Derivative Works if you do not have the proper experience or
 * training. Do not use this Work or Derivative Works fot other products than
 * Bosch Sensortec products.  
 *
 * The Information provided is believed to be accurate and reliable. Bosch
 * Sensortec assumes no responsibility for the consequences of use of such
 * Information nor for any infringement of patents or other rights of third
 * parties which may result from its use. No license is granted by implication
 * or otherwise under any patent or patent rights of Bosch. Specifications
 * mentioned in the Information are subject to change without notice.
 */


/*! \file bmp085_bst.c
  \brief This file contains all function implementations for the BMP085 API

  Details.
  */

#include "BMP085.h"

static bmp085_t *p_bmp085 = 0; /**< pointer to SMD500 / BMP085 device area */



/*
 * initialize BMP085 / SMD500 
 *
 * This function initializes the BMP085 pressure sensor/ the successor SMD500
 * is also supported. The function automatically detects the sensor type and
 * stores this for all future communication and calculation steps
 *
 * \param *bmp085_t pointer to bmp085 device data structure
 * \return result of communication routines
 */
int bmp085_init(bmp085_t *bmp085) {
    char comres = 0;
    unsigned char data;
    long dummy;

    p_bmp085 = bmp085; /* assign BMP085 ptr */
    p_bmp085->sensortype = E_SENSOR_NOT_DETECTED;
    p_bmp085->dev_addr = BMP085_I2C_ADDR; /* preset BMP085 I2C_addr */
    comres += p_bmp085->BMP085_BUS_READ_FUNC(p_bmp085->dev_addr, BMP085_CHIP_ID__REG, &data, 1);  /* read Chip Id */

    p_bmp085->chip_id = BMP085_GET_BITSLICE(data, BMP085_CHIP_ID);  
    p_bmp085->number_of_samples = 1;  
    p_bmp085->oversampling_setting=0;

    if (p_bmp085->chip_id == BMP085_CHIP_ID) {            /* get bitslice */
        p_bmp085->sensortype = BOSCH_PRESSURE_BMP085;

        comres += p_bmp085->BMP085_BUS_READ_FUNC(p_bmp085->dev_addr, BMP085_VERSION_REG, &data, 1); /* read Version reg */

        p_bmp085->ml_version = BMP085_GET_BITSLICE(data, BMP085_ML_VERSION);        /* get ML Version */
        p_bmp085->al_version = BMP085_GET_BITSLICE(data, BMP085_AL_VERSION);        /* get AL Version */
        bmp085_get_cal_param( ); /* readout bmp085 calibparam structure */
    } else {
        p_bmp085->sensortype = BOSCH_PRESSURE_SMD500;
        p_bmp085->smd500_t_resolution = SMD500_T_RESOLUTION_16BIT;
        p_bmp085->smd500_masterclock = SMD500_MASTERCLOCK_32768HZ;
        smd500_get_cal_param();

        /* calculate B1*/
        dummy =  (p_bmp085->cal_param.ac3 + SMD500_PARAM_M3);
        dummy *= SMD500_PARAM_ME;
        dummy >>= 11;
        dummy += SMD500_PARAM_MF;   
        p_bmp085->cal_param.b1 = dummy;

        /* calculate B2*/
        dummy = (p_bmp085->cal_param.ac2 + SMD500_PARAM_M2);
        dummy *= p_bmp085->cal_param.b1;
        dummy >>= 4;
        dummy /= (p_bmp085->cal_param.ac3 + SMD500_PARAM_M3);   // calculate B2      
        p_bmp085->cal_param.b2 = dummy;

        p_bmp085->sensortype = BOSCH_PRESSURE_SMD500;

        p_bmp085->cal_param.ac1 += SMD500_PARAM_M1;
        p_bmp085->cal_param.ac1 <<= 1;

        p_bmp085->cal_param.ac2 += SMD500_PARAM_M2;
        p_bmp085->cal_param.ac2 <<= 2;

        p_bmp085->cal_param.ac3 += SMD500_PARAM_M3;
        p_bmp085->cal_param.ac3 <<= 3;

        p_bmp085->cal_param.ac4 += SMD500_PARAM_M4;
        p_bmp085->cal_param.ac4 <<= 1;

        p_bmp085->cal_param.ac5 += SMD500_PARAM_M5;
        p_bmp085->cal_param.ac5 <<= 3;

        p_bmp085->cal_param.ac6 += SMD500_PARAM_M6;
        p_bmp085->cal_param.ac6 <<= 3;

        p_bmp085->cal_param.mb = SMD500_PARAM_MB;
        p_bmp085->cal_param.mc = SMD500_PARAM_MC;
        p_bmp085->cal_param.md = SMD500_PARAM_MD;   
    }

    return comres;
}

/*
 * read out parameters cal_param from BMP085 memory
 * \return result of communication routines
 */

//int bmp085_read_cal_param(void)
int bmp085_get_cal_param(void) {
    int comres;
    unsigned char data[22];
    comres = p_bmp085->BMP085_BUS_READ_FUNC( p_bmp085->dev_addr, BMP085_PROM_START__ADDR, data, BMP085_PROM_DATA__LEN );

    /* parameters AC1-AC6 */
    p_bmp085->cal_param.ac1 = (data[0]  << 8) | data[1];
    p_bmp085->cal_param.ac2 = (data[2]  << 8) | data[3];
    p_bmp085->cal_param.ac3 = (data[4]  << 8) | data[5];
    p_bmp085->cal_param.ac4 = (data[6]  << 8) | data[7];
    p_bmp085->cal_param.ac5 = (data[8]  << 8) | data[9];
    p_bmp085->cal_param.ac6 = (data[10] << 8) | data[11];

    /*parameters B1,B2*/
    p_bmp085->cal_param.b1 =  (data[12] << 8) | data[13];
    p_bmp085->cal_param.b2 =  (data[14] << 8) | data[15];

    /*parameters MB,MC,MD*/
    p_bmp085->cal_param.mb =  (data[16] << 8) | data[17];
    p_bmp085->cal_param.mc =  (data[18] << 8) | data[19];
    p_bmp085->cal_param.md =  (data[20] << 8) | data[21];

    return comres;
}

/*
 * read out parameters cal_param from SMD500 memory 
 *
 * This routine generates parameters from bitsliced data
 * \return result of communication routines
 */
//int smd500_read_cal_param(void) 
int smd500_get_cal_param(void) {
    unsigned char data[SMD500_PROM_DATA__LEN];    
    int comres;

    comres = p_bmp085->BMP085_BUS_READ_FUNC( p_bmp085->dev_addr, SMD500_PROM_START__ADDR, data, SMD500_PROM_DATA__LEN );

    p_bmp085->cal_param.ac1 = (unsigned short) ((data[0] << 8) | data[1]) & 0x3FFF;
    p_bmp085->cal_param.ac2 = ((data[0] &  0xC0) >> 6) + ((data[2] & ~0x01) << 1);
    p_bmp085->cal_param.ac3 = ((data[2] &  0x01) << 8) + data[3];
    p_bmp085->cal_param.ac4 = ((data[4] &  0x1f) << 8) + data[5];
    p_bmp085->cal_param.ac5 = ((data[4] & ~0x1f) >> 5) + ((data[6] & 0xf0) >> 1);
    p_bmp085->cal_param.ac6 = ((data[6] &  0x0f) << 8) + data[7];

    return comres;
}


/*
 * calculate temperature from ut
 *
 * ut was read from the device via I2C and fed into the right calc path for either SMD500 or BMP085
 * \param ut parameter ut read from device
 * \return temperature in steps of 0.1 deg celsius
 * \see bmp085_read_ut()
 */
//short bmp085_calc_temperature(unsigned long ut) 
short bmp085_get_temperature(unsigned long ut) {
    short temperature;
    long x1,x2,x3,x4,y2,y3,y4;

    if (p_bmp085->sensortype == BOSCH_PRESSURE_BMP085) {
        x1 = (((long) ut - (long) p_bmp085->cal_param.ac6) * (long) p_bmp085->cal_param.ac5) >> 15;
        x2 = ((long) p_bmp085->cal_param.mc << 11) / (x1 + p_bmp085->cal_param.md);
        p_bmp085->param_b5 = x1 + x2;
    } else { // SMD500
        if (p_bmp085->smd500_t_resolution == SMD500_T_RESOLUTION_16BIT) { // check for SMD500 temp resolution mode
            x1 = ((unsigned long) ((( ut * SMD500_PARAM_MJ) >> 16) - p_bmp085->cal_param.ac6));
        } else { //13BIT
            x1 = ut * 8  - p_bmp085->cal_param.ac6;
        }

        x2 = (x1 * x1) >> 13;
        y2 = (SMD500_PARAM_MB * x2) >> 13;
        x3 = (x2 * x1) >> 16;
        y3 = (SMD500_PARAM_MC * x3) >> 12;
        x4 = (x2 * x2) >> 16;
        y4 = (SMD500_PARAM_MD * x4) >> 14;

        p_bmp085->param_b5 = (((p_bmp085->cal_param.ac5) * ( (2*x1) + y2 + y3 + y4))) >> 13; // temperature in (1/160)�C
    }
    temperature = ((p_bmp085->param_b5 + 8) >> 4);  // temperature in 0.1�C

    return temperature;
}

/*
 * calculate pressure from up
 *
 * up was read from the device via I2C and fed into the right calc path for
 * either SMD500 or BMP085. In case of SMD500 value averaging is done in this
 * function, in case of BMP085 averaging is done through oversampling by the
 * sensor IC
 *
 * \param ut parameter ut read from device
 *  \return temperature in steps of 1.0 Pa
 *  \see bmp085_read_up()
 */
//long bmp085_calc_pressure(unsigned long up)
long bmp085_get_pressure(unsigned long up) {
    long pressure,x1,x2,x3,b3,b6;
    unsigned long b4, b7;

    b6 = p_bmp085->param_b5 - 4000;
    //*****calculate B3************
    x1 = (b6*b6) >> 12;       
    x1 *= p_bmp085->cal_param.b2;
    x1 >>= 11;

    x2 = (p_bmp085->cal_param.ac2*b6);
    x2 >>= 11;

    x3 = x1 +x2;

    b3 = (((((long)p_bmp085->cal_param.ac1 )*4 + x3) <<p_bmp085->oversampling_setting) + 2) >> 2;

    //*****calculate B4************
    x1 = (p_bmp085->cal_param.ac3* b6) >> 13;
    x2 = (p_bmp085->cal_param.b1 * ((b6*b6) >> 12) ) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (p_bmp085->cal_param.ac4 * (unsigned long) (x3 + 32768)) >> 15;

    b7 = ((unsigned long)(up - b3) * (50000>>p_bmp085->oversampling_setting));   
    if (b7 < 0x80000000) {
        pressure = (b7 << 1) / b4;
    } else { 
        pressure = (b7 / b4) << 1;
    }

    x1 = pressure >> 8;
    x1 *= x1;
    x1 = (x1 * SMD500_PARAM_MG) >> 16;
    x2 = (pressure * SMD500_PARAM_MH) >> 16;
    pressure += (x1 + x2 + SMD500_PARAM_MI) >> 4;    // pressure in Pa  

    return pressure;
}


/*
 * Read out ut for temperature conversion
 *
 * \return ut parameter that represents the uncompensated temperature sensors
 * conversion value
 */
//unsigned short bmp085_read_ut ()
unsigned short bmp085_get_ut(void) {
    unsigned short ut;
    unsigned char data[2];    
    unsigned char ctrl_reg_data;
    int wait_time;
    int comres;

    if (p_bmp085->chip_id == BMP085_CHIP_ID) { /* get bitslice */
        ctrl_reg_data = BMP085_T_MEASURE;
        wait_time = BMP085_TEMP_CONVERSION_TIME;
    } else {
        ctrl_reg_data = SMD500_T_MEASURE + p_bmp085->smd500_t_resolution + p_bmp085->smd500_masterclock;
    }

    // wait_time can be 9 ms for 13 bit smd500_t_resolution
    if (p_bmp085->smd500_t_resolution == SMD500_T_RESOLUTION_13BIT) {
        wait_time = SMD500_TEMP_CONVERSION_TIME_13;
    } else {
        wait_time = SMD500_TEMP_CONVERSION_TIME_16;
    }
    comres = p_bmp085->BMP085_BUS_WRITE_FUNC(p_bmp085->dev_addr, BMP085_CTRL_MEAS_REG, &ctrl_reg_data, 1);


    p_bmp085->delay_msec (wait_time);  
    comres += p_bmp085->BMP085_BUS_READ_FUNC(p_bmp085->dev_addr, BMP085_ADC_OUT_MSB_REG, data, 2);
    ut = (data[0] <<8) | data[1];

    return ut;
}


/*
 * read out up for pressure conversion
 *
 * depending on the oversampling ratio setting up can be 16 to 19 bit
 * \return up parameter that represents the uncompensated pressure value
 */
//unsigned long bmp085_read_up ()
unsigned long bmp085_get_up(void) {
    int i;
    unsigned long up = 0;
    unsigned char data[3];    
    unsigned char ctrl_reg_data;
    int comres = 0;

    if (p_bmp085->chip_id == BMP085_CHIP_ID) { 
        ctrl_reg_data = BMP085_P_MEASURE + (p_bmp085->oversampling_setting << 6);
        comres = p_bmp085->BMP085_BUS_WRITE_FUNC( p_bmp085->dev_addr, BMP085_CTRL_MEAS_REG, &ctrl_reg_data, 1 );

        p_bmp085->delay_msec ( 2 + (3 << (p_bmp085->oversampling_setting) ) );
        comres += p_bmp085->BMP085_BUS_READ_FUNC( p_bmp085->dev_addr, BMP085_ADC_OUT_MSB_REG, data, 3 );

        up = (((unsigned long) data[0] << 16) | ((unsigned long) data[1] << 8) | (unsigned long) data[2]) >> (8-p_bmp085->oversampling_setting);
        p_bmp085->number_of_samples = 1;
    } else {
        // SMD500 
        ctrl_reg_data = SMD500_P_MEASURE + p_bmp085->smd500_masterclock;
        p_bmp085->number_of_samples = (1 << (p_bmp085->oversampling_setting));

        for (i = 0; i < p_bmp085->number_of_samples; i++) {
            comres += p_bmp085->BMP085_BUS_WRITE_FUNC( p_bmp085->dev_addr, BMP085_CTRL_MEAS_REG, &ctrl_reg_data, 1 );
            p_bmp085->delay_msec (34);

            comres += p_bmp085->BMP085_BUS_READ_FUNC( p_bmp085->dev_addr, BMP085_ADC_OUT_MSB_REG, data, 2 );
            up += (((unsigned long)data[0] <<8) | (unsigned long)data[1]);
        }    
    }

    return up;
}
