/*
 * Driver.cpp
 *
 * Simple driver application for use with the BMP085 barometric pressure
 * driver.
 *
 * Andy Dalton - September 2013
 */
#include <Arduino.h>
#include "BMP085.h"

enum {
    SERIAL_BAUD_RATE = 9600,
};

static BMP085 sensor;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    sensor.init();
}

void loop() {
    double tempC        = sensor.get_temperature_C();
    long   pressurePa   = sensor.get_pressure();
    double pressureInHg = sensor.get_pressure_inMg();

    Serial.print("Temp: ");
    Serial.print(tempC);
    Serial.print(" degrees C, Pressure: ");
    Serial.print(pressurePa);
    Serial.print(" Pa (");
    Serial.print(pressureInHg);
    Serial.println(" inHg)");

    delay(1000);
}
