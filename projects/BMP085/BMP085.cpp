/*
 * BMP085 Pressure Sensor API
 * Copyright (C) 2009 Bosch Sensortec GmbH and (C) 2012 Rober Bosch LLC
 *
 * Modified by Andy Dalton - September 2013
 *     * Straight conversion from C to C++ for use with Arduino.
 *       NB: I did not refactor the implemntation to make it more
 *       object-oriented, and there are aspects regarding that that could use
 *       some attention.  I just wanted a reference implementation from Bosch.
 *     * Cleaned up the code a bit.  Again note that I didn't try to eliminate
 *       magic numbers or unmeaningful variable names -- I just try to reformat
 *       it to make it a bit more readable.
 *     * I made no changes to the original implementation to try to optimize
 *       for size.
 *     * Original implementation taken from revision 1691:
 *       http://svn.code.sf.net/p/bosch-ros-pkg/code/trunk/stacks/bosch_drivers/bmp085/
 *
 * License
 * -------
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License and the following
 * stipulations. The Apache License , Version 2.0 is applicable unless otherwise
 * stated by the stipulations of the disclaimer below.
 *
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Disclaimer
 * ----------
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

#include <Arduino.h>
#include <Wire.h>
#include "BMP085.h"


/*
 * initialize BMP085 / SMD500
 *
 * This function initializes the BMP085 pressure sensor/ the successor SMD500
 * is also supported. The function automatically detects the sensor type and
 * stores this for all future communication and calculation steps
 *
 * \param *BMP085 pointer to bmp085 device data structure
 * \return result of communication routines
 */
void BMP085::init() {
    uint8_t data;
    long dummy;

    Wire.begin();
    delay(1000);

    this->sensortype = E_SENSOR_NOT_DETECTED;
    this->dev_addr   = BMP085_I2C_ADDR;       /* preset BMP085 I2C_addr */

    // Read Chip ID
    this->readmem(BMP085_CHIP_ID__REG, &data, sizeof(data));

    this->chip_id              = BMP085_GET_BITSLICE(data, BMP085_CHIP_ID);
    this->number_of_samples    = 1;
    this->oversampling_setting = 0;

    // Get bitslice
    if (this->chip_id == BMP085_CHIP_ID) {
        this->sensortype = BOSCH_PRESSURE_BMP085;

        // read Version reg
        this->readmem(BMP085_VERSION_REG, &data, sizeof(data));

        // Get ML Version
        this->ml_version = BMP085_GET_BITSLICE(data, BMP085_ML_VERSION);

        // Get AL Version
        this->al_version = BMP085_GET_BITSLICE(data, BMP085_AL_VERSION);

        // Readout bmp085 calibparam structure
        this->bmp085_get_cal_param();
    } else {
        this->sensortype          = BOSCH_PRESSURE_SMD500;
        this->smd500_t_resolution = SMD500_T_RESOLUTION_16BIT;
        this->smd500_masterclock  = SMD500_MASTERCLOCK_32768HZ;

        this->smd500_get_cal_param();

        /*
         * Calculate B1
         */
        dummy   = (this->cal_param.ac3 + SMD500_PARAM_M3);
        dummy  *= SMD500_PARAM_ME;
        dummy >>= 11;
        dummy  += SMD500_PARAM_MF;
        this->cal_param.b1 = dummy;

        /*
         * Calculate B2
         */
        dummy   = (this->cal_param.ac2 + SMD500_PARAM_M2);
        dummy  *= this->cal_param.b1;
        dummy >>= 4;
        dummy  /= (this->cal_param.ac3 + SMD500_PARAM_M3);
        this->cal_param.b2 = dummy;


        this->cal_param.ac1  += SMD500_PARAM_M1;
        this->cal_param.ac1 <<= 1;

        this->cal_param.ac2  += SMD500_PARAM_M2;
        this->cal_param.ac2 <<= 2;

        this->cal_param.ac3  += SMD500_PARAM_M3;
        this->cal_param.ac3 <<= 3;

        this->cal_param.ac4  += SMD500_PARAM_M4;
        this->cal_param.ac4 <<= 1;

        this->cal_param.ac5  += SMD500_PARAM_M5;
        this->cal_param.ac5 <<= 3;

        this->cal_param.ac6  += SMD500_PARAM_M6;
        this->cal_param.ac6 <<= 3;

        this->cal_param.mb    = SMD500_PARAM_MB;
        this->cal_param.mc    = SMD500_PARAM_MC;
        this->cal_param.md    = SMD500_PARAM_MD;
    }
}

/*
 * Read parameters cal_param from BMP085 memory
 */
void BMP085::bmp085_get_cal_param() {
    uint8_t data[BMP085_PROM_DATA__LEN];

    this->readmem(BMP085_PROM_START__ADDR, data, sizeof(data));

    this->cal_param.ac1 = (data[ 0] << 8) | data[ 1];
    this->cal_param.ac2 = (data[ 2] << 8) | data[ 3];
    this->cal_param.ac3 = (data[ 4] << 8) | data[ 5];
    this->cal_param.ac4 = (data[ 6] << 8) | data[ 7];
    this->cal_param.ac5 = (data[ 8] << 8) | data[ 9];
    this->cal_param.ac6 = (data[10] << 8) | data[11];

    this->cal_param.b1  = (data[12] << 8) | data[13];
    this->cal_param.b2  = (data[14] << 8) | data[15];

    this->cal_param.mb  = (data[16] << 8) | data[17];
    this->cal_param.mc  = (data[18] << 8) | data[19];
    this->cal_param.md  = (data[20] << 8) | data[21];
}

/*
 * read parameters cal_param from SMD500 memory
 *
 * This routine generates parameters from bitsliced data
 */
void BMP085::smd500_get_cal_param() {
    uint8_t data[SMD500_PROM_DATA__LEN];

    this->readmem(SMD500_PROM_START__ADDR, data, sizeof(data));

    this->cal_param.ac1 = (word) ((data[0] << 8) | data[1]) & 0x3FFF;
    this->cal_param.ac2 = ((data[0] &  0xC0) >> 6) + ((data[2] & ~0x01) << 1);
    this->cal_param.ac3 = ((data[2] &  0x01) << 8) + data[3];
    this->cal_param.ac4 = ((data[4] &  0x1f) << 8) + data[5];
    this->cal_param.ac5 = ((data[4] & ~0x1f) >> 5) + ((data[6] & 0xf0) >> 1);
    this->cal_param.ac6 = ((data[6] &  0x0f) << 8) + data[7];
}

/*
 * calculate temperature from ut
 *
 * ut was read from the device via I2C and fed into the right calc path for
 * either SMD500 or BMP085
 *
 * \param ut parameter ut read from device
 * \return temperature in steps of 0.1 deg celsius
 * \see bmp085_read_ut()
 */
short BMP085::get_temperature() {
    unsigned short ut = this->get_ut();
    short temperature;
    long x1;
    long x2;

    if (this->sensortype == BOSCH_PRESSURE_BMP085) {
        x1 = ( (long(ut) - long(this->cal_param.ac6))
              * long(this->cal_param.ac5)) >> 15;
        x2 =   (long(this->cal_param.mc) << 11)
             / (x1 + this->cal_param.md);

        this->param_b5 = x1 + x2;
    } else { // SMD500
        long x3;
        long x4;
        long y2;
        long y3;
        long y4;

        // check for SMD500 temp resolution mode
        if (this->smd500_t_resolution == SMD500_T_RESOLUTION_16BIT) {
            x1 = ((unsigned long) ((( ut * SMD500_PARAM_MJ) >> 16)
                    - this->cal_param.ac6));
        } else { // 13-bit
            x1 = ut * 8  - this->cal_param.ac6;
        }

        x2 = (x1 * x1) >> 13;
        y2 = (SMD500_PARAM_MB * x2) >> 13;
        x3 = (x2 * x1) >> 16;
        y3 = (SMD500_PARAM_MC * x3) >> 12;
        x4 = (x2 * x2) >> 16;
        y4 = (SMD500_PARAM_MD * x4) >> 14;

        // Temperature in (1/160) degrees C
        this->param_b5 = ((this->cal_param.ac5) *
                            ((2 * x1) + y2 + y3 + y4)) >> 13;
    }
    // temperature in 0.1 degrees C
    temperature = ((this->param_b5 + 8) >> 4);

    return temperature;
}

double BMP085::get_temperature_C() {
    return this->get_temperature() / 10.0;
}

double BMP085::get_pressure_inMg() {
    enum {
        PA_PER_IN_MG = 3386,
    };

    return this->get_pressure() / double(PA_PER_IN_MG);
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
 * \return temperature in steps of 1.0 Pa
 * \see bmp085_read_up()
 */
long BMP085::get_pressure() {
    unsigned long up = this->get_up();
    long pressure;
    long x1;
    long x2;
    long x3;
    long b3;
    long b6;
    unsigned long b4;
    unsigned long b7;

    b6 = this->param_b5 - 4000;

    // ***** Calculate B3 ************
    x1 =  (b6 * b6) >> 12;
    x1 *= this->cal_param.b2;
    x1 >>= 11;

    x2 = (this->cal_param.ac2 * b6);
    x2 >>= 11;

    x3 = x1 + x2;

    b3 = ((((long(this->cal_param.ac1)) * 4 + x3)
                << this->oversampling_setting) + 2) >> 2;

    // ***** Calculate B4 ************
    x1 = (this->cal_param.ac3 * b6) >> 13;
    x2 = (this->cal_param.b1 * ((b6 * b6) >> 12) ) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (this->cal_param.ac4 * (unsigned long) (x3 + 32768)) >> 15;

    b7 = ((unsigned long)(up - b3) * (50000 >> this->oversampling_setting));
    if (b7 < 0x80000000) {
        pressure = (b7 << 1) / b4;
    } else {
        pressure = (b7 / b4) << 1;
    }

    x1  = pressure >> 8;
    x1 *= x1;
    x1  = (x1 * SMD500_PARAM_MG) >> 16;
    x2  = (pressure * SMD500_PARAM_MH) >> 16;
    pressure += (x1 + x2 + SMD500_PARAM_MI) >> 4;    // pressure in Pa

    return pressure;
}


/*
 * Read out ut for temperature conversion
 *
 * \return ut parameter that represents the uncompensated temperature sensors
 * conversion value
 */
word BMP085::get_ut() {
    word    ut;
    uint8_t data[2];
    uint8_t ctrl_reg_data;
    int     wait_time;

    if (this->chip_id == BMP085_CHIP_ID) { /* get bitslice */
        ctrl_reg_data = BMP085_T_MEASURE;
        wait_time     = BMP085_TEMP_CONVERSION_TIME;
    } else {
        ctrl_reg_data =   SMD500_T_MEASURE
                        + this->smd500_t_resolution
                        + this->smd500_masterclock;
    }

    // Wait_time can be 9 ms for 13 bit smd500_t_resolution
    if (this->smd500_t_resolution == SMD500_T_RESOLUTION_13BIT) {
        wait_time = SMD500_TEMP_CONVERSION_TIME_13;
    } else {
        wait_time = SMD500_TEMP_CONVERSION_TIME_16;
    }
    this->writemem(BMP085_CTRL_MEAS_REG, ctrl_reg_data);

    delay(wait_time);
    this->readmem(BMP085_ADC_OUT_MSB_REG, data, sizeof(data));
    ut = (data[0] << 8) | data[1];

    return ut;
}


/*
 * read out up for pressure conversion
 *
 * depending on the oversampling ratio setting up can be 16 to 19 bit
 * \return up parameter that represents the uncompensated pressure value
 */
unsigned long BMP085::get_up() {
    unsigned long up = 0;
    uint8_t       ctrl_reg_data;

    if (this->chip_id == BMP085_CHIP_ID) {
        uint8_t data[3];

        ctrl_reg_data = BMP085_P_MEASURE + (this->oversampling_setting << 6);
        this->writemem(BMP085_CTRL_MEAS_REG, ctrl_reg_data);

        delay(2 + (3 << (this->oversampling_setting)));
        this->readmem(BMP085_ADC_OUT_MSB_REG, data, sizeof(data));

        up = (   ((unsigned long) data[0] << 16)
               | ((unsigned long) data[1] <<  8)
               | ((unsigned long) data[2] <<  0)
             ) >> (8 - this->oversampling_setting);
        this->number_of_samples = 1;
    } else {
        // SMD500
        uint8_t data[2];

        ctrl_reg_data = SMD500_P_MEASURE + this->smd500_masterclock;
        this->number_of_samples = (1 << (this->oversampling_setting));

        for (int i = 0; i < this->number_of_samples; i++) {
            this->writemem(BMP085_CTRL_MEAS_REG, ctrl_reg_data);
            delay(34);

            this->readmem(BMP085_ADC_OUT_MSB_REG, data, sizeof(data));
            up += (((unsigned long) data[0] << 8) | (unsigned long)data[1]);
        }
    }

    return up;
}

void BMP085::writemem(uint8_t addr, uint8_t val) {
    Wire.beginTransmission(this->dev_addr);
    Wire.write(addr);
    Wire.write(val);
    Wire.endTransmission();
}

void BMP085::readmem(uint8_t addr, uint8_t buff[], uint8_t nbytes) {
    Wire.beginTransmission(this->dev_addr);
    Wire.write(addr);
    Wire.endTransmission();

    Wire.beginTransmission(this->dev_addr);
    Wire.requestFrom(this->dev_addr, nbytes);

    for (uint8_t i = 0; Wire.available() && i < nbytes; ++i) {
        buff[i] = Wire.read();
    }
    Wire.endTransmission();
}
