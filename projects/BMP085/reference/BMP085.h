/*  $Date: 2009/10/23 $
 *  $Revision: 1.2 $
 */

/*
 * \mainpage BMP085 Barometric Pressure Sensor API
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
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License and the following
 * stipulations. The Apache License , Version 2.0 is applicable unless otherwise
 * stated by the stipulations of the disclaimer below. 
 * 
 * You may obtain a copy of the License at 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
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
 * training. Do not use this Work or Derivative Works for other products than
 * Bosch Sensortec products.  
 *
 * The Information provided is believed to be accurate and reliable. Bosch
 * Sensortec assumes no responsibility for the consequences of use of such
 * Information nor for any infringement of patents or other rights of third
 * parties which may result from its use. No license is granted by implication
 * or otherwise under any patent or patent rights of Bosch. Specifications
 * mentioned in the Information are subject to change without notice.
 */

/** \file bmp085_bst.h
    \brief Header file for all #define constants and function prototypes
*/

#ifndef __BMP085_H__
#define __BMP085_H__

#define bmp085_calc_temperature(ut) bmp085_get_temperature(ut)
#define bmp085_calc_pressure(up)    bmp085_get_pressure(up)
#define bmp085_read_ut()            bmp085_get_ut()
#define bmp085_read_up()            bmp085_get_up()
#define bmp085_read_reg(address, data, length)  bmp085_get_reg(address, data, length)
#define bmp085_write_reg(address, data, length) bmp085_set_reg(address, data, length)
#define bmp085_read_cal_param() bmp085_get_cal_param()
#define smd500_read_cal_param() smd500_get_cal_param()

/*
 * Define for used read and write macros 
 */



/*
 * Defines the return parameter type of the BMP085_WR_FUNCTION
 */
#define BMP085_BUS_WR_RETURN_TYPE char

/*
 * Defines the calling parameter types of the BMP085_WR_FUNCTION
 */
#define BMP085_BUS_WR_PARAM_TYPES unsigned char, unsigned char, unsigned char*, unsigned char

/*
 * Links the order of parameters defined in BMP085_BUS_WR_PARAM_TYPE to
 * function calls used inside the API
 */
#define BMP085_BUS_WR_PARAM_ORDER device_addr, register_addr, register_data, write_length

/* Never change this line */
#define BMP085_BUS_WRITE_FUNC(device_addr, register_addr, register_data, write_length) bus_write( device_addr, register_addr, register_data, write_length )

/*
 * Defines the return parameter type of the BMP085_RD_FUNCTION
 */
#define BMP085_BUS_RD_RETURN_TYPE char

/*
 * defines the calling parameter types of the BMP085_RD_FUNCTION
 */
#define BMP085_BUS_RD_PARAM_TYPES unsigned char, unsigned char, unsigned char*, unsigned char

/*
 * links the order of parameters defined in BMP085_BUS_RD_PARAM_TYPE to function calls used inside the API
 */
#define BMP085_BUS_RD_PARAM_ORDER device_addr, register_addr, register_data, read_length


/* Never change this line */
#define BMP085_BUS_READ_FUNC(device_addr, register_addr, register_data, read_length) bus_read( device_addr, register_addr, register_data, read_length )


/*
 * CHIP_TYPE CONSTANTS
 */
#define BMP085_CHIP_ID        0x55
#define BOSCH_PRESSURE_SMD500 05
#define BOSCH_PRESSURE_BMP085 85


/*
 * BMP085 I2C Address
 */
#define BMP085_I2C_ADDR (0xEE >> 1)


/*
 * SMB380 API error codes
 */
#define E_BMP_NULL_PTR        ((char) -127)
#define E_BMP_COMM_RES        ((char) -1)
#define E_BMP_OUT_OF_RANGE    ((char) -2)
#define E_SENSOR_NOT_DETECTED ((char)  0)

/* 
 * Register definitions    
 */
#define BMP085_PROM_START__ADDR 0xAA
#define BMP085_PROM_DATA__LEN   22

#define BMP085_CHIP_ID_REG      0xD0
#define BMP085_VERSION_REG      0xD1

#define BMP085_CTRL_MEAS_REG    0xF4
#define BMP085_ADC_OUT_MSB_REG  0xF6
#define BMP085_ADC_OUT_LSB_REG  0xF7

#define BMP085_SOFT_RESET_REG   0xE0

#define BMP085_T_MEASURE        0x2E // temperature measurent 
#define BMP085_P_MEASURE        0x34 // pressure measurement

#define BMP085_TEMP_CONVERSION_TIME 5 // TO be spec'd by GL or SB


/* SMD500 specific constants */
#define SMD500_PROM_START__ADDR 0xF8
#define SMD500_PROM_DATA__LEN   8

#define SMD500_PARAM_M1  -2218 // Calibration parameter
#define SMD500_PARAM_M2   -457 // Calibration parameter
#define SMD500_PARAM_M3  -1984 // Calibration parameter
#define SMD500_PARAM_M4   8808 // Calibration parameter
#define SMD500_PARAM_M5    496 // Calibration parameter
#define SMD500_PARAM_M6   1415 // Calibration parameter

#define SMD500_PARAM_MB  -4955 // Calibration parameter
#define SMD500_PARAM_MC  11611 // Calibration parameter
#define SMD500_PARAM_MD -12166 // Calibration parameter
#define SMD500_PARAM_ME -17268 // Calibration parameter
#define SMD500_PARAM_MF  -8970 // Calibration parameter

#define SMD500_PARAM_MG   3038 // Calibration parameter
#define SMD500_PARAM_MH  -7357 // Calibration parameter
#define SMD500_PARAM_MI   3791 // Calibration parameter
#define SMD500_PARAM_MJ  64385 // Calibration parameter

#define SMD500_STANDBY             0    // set the device in stand-by modus to reduce power consumption
#define SMD500_MASTERCLOCK_32768HZ 0x04 // external Master clock 32.768kHz
#define SMD500_MASTERCLOCK_1MHZ    0    // external Master clock 1MHz
#define SMD500_T_RESOLUTION_13BIT  0    // 13 Bit resolution temperature
#define SMD500_T_RESOLUTION_16BIT  0x80 // 16 Bit resolution temperature
#define SMD500_T_MEASURE           0x6A // temperature measurent 
#define SMD500_P_MEASURE           0xF0 // pressure measurement

#define SMD500_TEMP_CONVERSION_TIME_13   9
#define SMD500_TEMP_CONVERSION_TIME_16  34
 

/* Register write and read delays */
#define BMP085_MDELAY_DATA_TYPE unsigned int
#define BMP085_MDELAY_RETURN_TYPE  void

/*
 * This structure holds all device specific calibration parameters 
 */
typedef struct {
   short ac1;
   short ac2;
   short ac3;
   unsigned short ac4;
   unsigned short ac5;
   unsigned short ac6;
   short b1;
   short b2;
   short mb;
   short mc;
   short md;               
} bmp085_smd500_calibration_param_t;


/*
 * BMP085 image registers data structure
 */
typedef struct {   
  bmp085_smd500_calibration_param_t cal_param;  
  unsigned char mode;
  unsigned char chip_id;
  unsigned char ml_version;
  unsigned char al_version;
  unsigned char dev_addr;   
  unsigned char sensortype;
  
  long param_b5;
  int number_of_samples;
  short oversampling_setting;
  short smd500_t_resolution;
  short smd500_masterclock;

  BMP085_BUS_WR_RETURN_TYPE (*bus_write)( BMP085_BUS_WR_PARAM_TYPES );
  BMP085_BUS_RD_RETURN_TYPE (*bus_read)( BMP085_BUS_RD_PARAM_TYPES );
  BMP085_MDELAY_RETURN_TYPE (*delay_msec)( BMP085_MDELAY_DATA_TYPE );
} bmp085_t;

/* 
 * Bit slice positions in registers
 */
#define BMP085_CHIP_ID__POS  0
#define BMP085_CHIP_ID__MSK  0xFF
#define BMP085_CHIP_ID__LEN  8
#define BMP085_CHIP_ID__REG  BMP085_CHIP_ID_REG


#define BMP085_ML_VERSION__POS  0
#define BMP085_ML_VERSION__LEN  4
#define BMP085_ML_VERSION__MSK  0x0F
#define BMP085_ML_VERSION__REG  BMP085_VERSION_REG



#define BMP085_AL_VERSION__POS  4
#define BMP085_AL_VERSION__LEN  4
#define BMP085_AL_VERSION__MSK  0xF0
#define BMP085_AL_VERSION__REG  BMP085_VERSION_REG


/* DATA REGISTERS */



/* LG/HG thresholds are in LSB and depend on RANGE setting */
/* no range check on threshold calculation */
#define BMP085_GET_BITSLICE(regvar, bitname)      (regvar & bitname##__MSK) >> bitname##__POS
#define BMP085_SET_BITSLICE(regvar, bitname, val) (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)  


/* General Setup Functions */



/*
 * BMP085_init 
 *
 * input :  Pointer to bmp085_t 
 * output:  -       
 * return:  result of communication function
 * notes :   
 */
int bmp085_init( bmp085_t* );

//short bmp085_calc_temperature(unsigned long ut);
short bmp085_get_temperature( unsigned long ut );

//long bmp085_calc_pressure(unsigned long up);
long bmp085_get_pressure( unsigned long up );

//unsigned short bmp085_read_ut(void);
//unsigned long  bmp085_read_up(void);
unsigned short bmp085_get_ut( void );
unsigned long  bmp085_get_up( void );



/* MISC RAW functions */

/* read: address, data-pointer, length */
//char bmp085_read_reg(unsigned char , unsigned char *, unsigned char);
char bmp085_get_reg( unsigned char , unsigned char*, unsigned char );

/* write: address, data-pointer, length */
//char bmp085_write_reg(unsigned char , unsigned char*, unsigned char );
char bmp085_set_reg( unsigned char , unsigned char*, unsigned char );


/* API internal helper functions */

//int bmp085_read_cal_param(void);
//int smd500_read_cal_param(void);
int bmp085_get_cal_param( void );
int smd500_get_cal_param( void );

#endif   // __BMP085_H__
