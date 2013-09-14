/*
 * Dth.cpp
 *
 * Author:  Rob Tillaart (modified by Andy Dalton)
 * Version: 0.2
 * Purpose: Implementation of common functionality across DHT-based temperature
 *          and humidity sensors.
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

#include "Dht.h"

// Various named constants.
enum {
    /*
     * Time required to signal the DHT11 to switch from low power mode to
     * running mode.  18 ms is the minimal, add a few extra ms to be safe.
     */
    START_SIGNAL_WAIT = 20,

    /*
     * Once the start signal has been sent, we wait for a response.  The doc
     * says this should take 20-40 us, we wait 5 ms to be safe.
     */
    RESPONSE_WAIT = 4,

    /*
     * The time threshold between a 0 bit and a 1 bit in the response.  Times
     * greater than this (in ms) will be considered a 1; otherwise they'll be
     * considered a 0.
     */
    ONE_THRESHOLD = 40,

    /*
     * The number of bytes we expect from the sensor.  This consists of one
     * byte for the integral part of the humidity, one byte for the fractional
     * part of the humidity, one byte for the integral part of the temperature,
     * one byte for the fractional part of the temperature, and one byte for
     * a checksum.  The DHT11 doesn't capture the fractional parts of the
     * temperature and humidity, but it doesn't transmit data during those
     * times.
     */
    RESPONSE_SIZE =  5,

    /*
     * The number of bits in a bytes.
     */
    BITS_PER_BYTE =  8,

    /*
     * The 0-base most significant bit in a byte.
     */
    BYTE_MS_BIT =  7,

};

const char* const Dht::VERSION = "0.2";

Dht::ReadStatus Dht::read() {
    uint8_t    buffer[RESPONSE_SIZE] = { 0 };
    uint8_t    bitIndex              = BYTE_MS_BIT;
    ReadStatus status                = OK;

    // Request sample
    pinMode(this->pin, OUTPUT);
    digitalWrite(this->pin, LOW);
    delay(START_SIGNAL_WAIT);

    // Wait for response
    digitalWrite(this->pin, HIGH);
    delayMicroseconds(RESPONSE_WAIT);
    pinMode(this->pin, INPUT);

    // Acknowledge or timeout
    // Response signal should first be low for 80us...
    if ((status = this->waitForPinChange(LOW)) != OK) {
        goto done;
    }

    // ... then be high for 80us ...
    if ((status = this->waitForPinChange(HIGH)) != OK) {
        goto done;
    }

    /*
     * ... then provide 5 bytes of data that include the integral part of the
     * humidity, the fractional part of the humidity, the integral part of the
     * temperature, the fractional part of the temperature, and a checksum
     */
    for (size_t i = 0; i < BITS_IN(buffer); i++) {
        if ((status = this->waitForPinChange(LOW)) != OK) {
            goto done;
        }

        unsigned long highStart = micros();

        if ((status = this->waitForPinChange(HIGH)) != OK) {
            goto done;
        }

        // 26-28 us = 0, 50 us = 1.  40 us is a good threshold between 0 and 1
        if ((micros() - highStart) > ONE_THRESHOLD) {
            buffer[i / BITS_PER_BYTE] |= (1 << bitIndex);
        }

        // Decrement or reset bitIndex
        bitIndex = (bitIndex > 0) ? bitIndex - 1 : BYTE_MS_BIT;
    }

    // Check the checksum.  Only if it's good, record the new values.
    if (buffer[CHECKSUM_INDEX] == (  buffer[HUMIDITY_INT_INDEX]
                                   + buffer[HUMIDITY_FRACT_INDEX]
                                   + buffer[TEMPERATURE_INT_INDEX]
                                   + buffer[TEMPERATURE_FRACT_INDEX])) {
        /*
         * Call the abstract method - this will differ based on the type of
         * sensor we're dealing with.
         */
        status = this->processData(buffer);
    } else {
        status = ERROR_CHECKSUM;
    }

done:
    return status;
}
