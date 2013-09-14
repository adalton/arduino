/*
 * Dth.h
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
#ifndef dht_h
#define dht_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#    include <Arduino.h>
#else
#    include <WProgram.h>
#endif

/* How may bits are in the given object? */
#define BITS_IN(object)  (8 * sizeof(object))

/* Is the Most Significant Bit of the given byte set? */
#define MSB_IS_SET(byte) ((byte) & 0b10000000)

/* Get everything but the most significant bit of the given byte. */
#define LSBs(byte)       ((byte) & 0b01111111)

/*
 * Dht
 *
 * An abstract class that models a DHT humidity/temperature sensor.
 */
class Dht {
private:
    // The last read humidity value
    double humidity;

    // The last read temperature value
    double temperature;

    // The pin over which we communicate with the sensor
    uint8_t pin;

public:
    // An enumeration modeling the read status of the sensor.
    enum ReadStatus {
        OK,
        ERROR_CHECKSUM,
        ERROR_TIMEOUT,
    };

    static const char* const VERSION;

    /*
     * Dht
     *
     * Constructs a new Dht object that communicates with a DHT11 sensor
     * over the given pin.
     */
    Dht(uint8_t newPin) : humidity(-1.0), temperature(-1.0), pin(newPin) {
    }

    /*
     * read
     *
     * Update the humidity and temperature of this object from the sensor.
     * Returns OK if the update was successful, ERROR_TIMEOUT if it times out
     * waiting for a response from the sensor, or ERROR_CHECKSUM if the
     * calculated checksum doesn't match the checksum provided by the sensor.
     */
    Dht::ReadStatus read();

    /*
     * getHumidity
     *
     * Gets the last read relative humidity percentage.
     */
    inline double getHumidity() const {
        return this->humidity;
    }

    /*
     * getTemperature
     *
     * Gets the last read temperature value in degrees Celsius.
     */
    inline double getTemperature() const {
        return this->temperature;
    }

protected:
    enum {
        /*
         * Default value for the maximum number of iterations performed by
         * the waitForPinChange function.
         */
        MAX_PIN_CHANGE_ITERATIONS = 10000,

        /*
         * The index in the response where the integral part of the humidity
         * reading is stored.
         */
        HUMIDITY_INT_INDEX = 0,

        /*
         * The index in the response where the fractional part of the humidity
         * reading is stored.
         */
        HUMIDITY_FRACT_INDEX = 1,

        /*
         * The index in the response where the temperature is stored.
         */
        TEMPERATURE_INT_INDEX =  2,

        /*
         * The index in the response where the fractional part of the
         * temperaturereading is stored.
         */
        TEMPERATURE_FRACT_INDEX = 3,

        /*
         * The index in the response where the checksum is stored.
         */
        CHECKSUM_INDEX =  4,
    };

    /*
     * processData
     *
     * Abstract method to processing the data read by the sensor.  Subclasses
     * will implement this method to provide concrete realizations for concrete
     * DHT-based sensors.
     */
    virtual Dht::ReadStatus processData(uint8_t buffer[]) = 0;

    /*
     * waitForPinChange
     *
     * Wait for the the data pin on the sensor to change from the given oldValue
     * to the opposite value (i.e., from HIGH -> LOW or from LOW -> HIGH).  The
     * function performs a tight loop decrementing the given maxIterations.  If
     * the state of the pin changes before maxIterations reaches 0, the function
     * returns OK; otherwise it returns ERROR_TIMEOUT.
     *
     * This is a private method used only by the Dht class.
     */
    inline ReadStatus waitForPinChange(const int oldValue,
                                       unsigned  maxIterations =
                                              MAX_PIN_CHANGE_ITERATIONS) const {
        while ((--maxIterations > 0) && (digitalRead(this->pin) == oldValue)) {
            // Just keep looping...
        }

        return (maxIterations > 0) ? OK : ERROR_TIMEOUT;
    }

    /*
     * setHumidity
     *
     * Updates the humidity value.
     */
    inline void setHumidity(const double humidity) {
        this->humidity = humidity;
    }

    /*
     * setTemperature
     *
     * Updates the temperature value.
     */
    inline void setTemperature(const double temperature) {
        this->temperature = temperature;
    }
};

#endif
