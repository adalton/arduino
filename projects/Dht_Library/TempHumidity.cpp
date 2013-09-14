/*
 * TempHumidity.cpp
 *
 * Andy Dalton (14/09/2013)
 *
 * Sample driver application to test the Dht module.
 */
#include "Arduino.h"
#include "Dht11.h"
#include "Dht21.h"
#include "Dht22.h"

enum {
    // The data I/O pin connected to the DHT11 sensor
    DHT11_DATA_PIN = 2,

    // The data I/O pin connected to the DHT21 sensor
    DHT21_DATA_PIN = 3,

    // The data I/O pin connected to the DHT22 sensor
    DHT22_DATA_PIN = 4,

    // The baud rate of the serial interface
    SERIAL_BAUD  = 9600,

    // The delay between sensor polls.
    POLL_DELAY   = 2000,
};

/*
 * readSensor
 *
 * Note that this takes a generic Dht reference and not a concrete DhtXY sensor.
 * It can read _any_ DHT-derived sensor.
 */
static void readSensor(Dht& sensor) {
    switch (sensor.read()) {
    case Dht::OK:
        Serial.print("Humidity (%): ");
        Serial.println(sensor.getHumidity());

        Serial.print("Temperature (C): ");
        Serial.println(sensor.getTemperature());
        break;

    case Dht::ERROR_CHECKSUM:
        Serial.println("Checksum error");
        break;

    case Dht::ERROR_TIMEOUT:
        Serial.println("Timeout error");
        break;

    default:
        Serial.println("Unknown error");
        break;
    }
}

/*
 * setup
 *
 * One-time initialization of the module.
 */
void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.print("Dht Lib version ");
    Serial.println(Dht::VERSION);
}

/*
 * loop
 *
 * Code to be executed repeatedly.
 */
void loop() {
    static Dht11 dht11(DHT11_DATA_PIN);
    static Dht21 dht21(DHT21_DATA_PIN);
    static Dht22 dht22(DHT22_DATA_PIN);

    delay(POLL_DELAY);

    readSensor(dht11);
    readSensor(dht21);
    readSensor(dht22);
}
