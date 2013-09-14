/*
 * Dth11.h
 *
 * Author:  Rob Tillaart (modified by Andy Dalton)
 * Version: 0.2
 * Purpose: Provide abstract base class for DHT-based sensors
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
#ifndef dht11_h
#define dht11_h

#include "Dht.h"

/*
 * Dht11
 *
 * A class that models a DHT11 humidity/temperature sensor.
 */
class Dht11 : public Dht {
public:
    /*
     * Dht11
     *
     * Constructs a new Dht11 object that communicates with a DHT11 sensor
     * over the given pin.
     */
    Dht11(uint8_t newPin) : Dht(newPin) {
    }

    /*
     * processData
     *
     * Process the data read by the sensor.
     */
    virtual Dht::ReadStatus processData(uint8_t buffer[]);
};

#endif
