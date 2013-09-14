/*
 * Dth21.cpp
 *
 * Author:  Rob Tillaart (modified by Andy Dalton)
 * Version: 0.2
 * Purpose: Implementation of functionality specific to the DTH21 temperature
 *          and humidity sensor.
 * URL:     http://arduino.cc/playground/Main/DHTLib
 *
 * History:
 *     0.2    - by Andy Dalton (14/09/2013) refactored to OO
 *     0.1.07 - added support for DHT21
 *     0.1.06 - minimize footprint (2012-12-27)
 *     0.1.05 - fixed negative temperature bug (thanks to Roseman)
 *     0.1.04 - improved readability of code using DHTLIB_OK in code
 *     0.1.03 - added error values for temp and humidity when read failed
 *     0.1.02 - added error codes
 *     0.1.01 - added support for Arduino 1.0, fixed typos (31/12/2011)
 *     0.1.0  - by Rob Tillaart (01/04/2011)
 *
 * inspired by DHT11 library
 *
 * Released to the public domain.
 */
#include "Dht21.h"

Dht::ReadStatus Dht21::processData(uint8_t buffer[]) {
    double temp;

    this->setHumidity(  buffer[HUMIDITY_INT_INDEX]
                      + (buffer[HUMIDITY_FRACT_INDEX] * (1.0 / 256.0)));

    temp = LSBs(buffer[TEMPERATURE_INT_INDEX]) +
           (buffer[TEMPERATURE_FRACT_INDEX] * (1.0 / 256.0));

    /*
     * If the most significant bit (MSB) is set, the value is negative.
     */
    if (MSB_IS_SET(buffer[TEMPERATURE_INT_INDEX])) {
        temp = -temp;
    }

    this->setTemperature(temp);

    return Dht::OK;
}
