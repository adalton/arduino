/*
 * LedControl.cpp - A library for controling Leds with a MAX7219/MAX7221
 * Copyright (c) 2007 Eberhard Fahle
 *
 * Originally from: http://playground.arduino.cc/Main/LedControl
 *
 * Minor modifications by Andy Dalton, December 2013
 *     * Added a ton of whitespace to increase readibility
 *     * Added some inline helper functions to eliminate code duplication
 *     * Made 'charTable' a static field of the LedControl class rather than
 *       a global
 *     * Converted opcodes from #defines to enums
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following
 * conditions:
 *
 * This permission notice shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "LedControl.h"

/*
 * Segments to be switched on for characters and digits on
 * 7-Segment Displays
 */
const byte LedControl::charTable[] = {
    0b01111110, 0b00110000, 0b01101101, 0b01111001, 0b00110011, 0b01011011, 0b01011111, 0b01110000,
    0b01111111, 0b01111011, 0b01110111, 0b00011111, 0b00001101, 0b00111101, 0b01001111, 0b01000111,
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b10000000, 0b00000001, 0b10000000, 0b00000000,
    0b01111110, 0b00110000, 0b01101101, 0b01111001, 0b00110011, 0b01011011, 0b01011111, 0b01110000,
    0b01111111, 0b01111011, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
    0b00000000, 0b01110111, 0b00011111, 0b00001101, 0b00111101, 0b01001111, 0b01000111, 0b00000000,
    0b00110111, 0b00000000, 0b00000000, 0b00000000, 0b00001110, 0b00000000, 0b00000000, 0b00000000,
    0b01100111, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00001000,
    0b00000000, 0b01110111, 0b00011111, 0b00001101, 0b00111101, 0b01001111, 0b01000111, 0b00000000,
    0b00110111, 0b00000000, 0b00000000, 0b00000000, 0b00001110, 0b00000000, 0b00000000, 0b00000000,
    0b01100111, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000
};

/* The opcodes for the MAX7221 and MAX7219 */
enum {
    OP_NOOP        =  0,
    OP_DIGIT0      =  1,
    OP_DIGIT1      =  2,
    OP_DIGIT2      =  3,
    OP_DIGIT3      =  4,
    OP_DIGIT4      =  5,
    OP_DIGIT5      =  6,
    OP_DIGIT6      =  7,
    OP_DIGIT7      =  8,
    OP_DECODEMODE  =  9,
    OP_INTENSITY   = 10,
    OP_SCANLIMIT   = 11,
    OP_SHUTDOWN    = 12,
    OP_DISPLAYTEST = 15,
};

enum {
    MAX_DEVICES = 8,
    NUM_ROWS    = 8,
    NUM_COLS    = 8,
};

LedControl::LedControl(int dataPin, int clkPin, int csPin, int numDevices) {
    this->SPI_MOSI = dataPin;
    this->SPI_CLK  = clkPin;
    this->SPI_CS   = csPin;

    if ((numDevices <= 0) || (numDevices > MAX_DEVICES)) {
        numDevices = MAX_DEVICES;
    }

    this->maxDevices = numDevices;

    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_CLK, OUTPUT);
    pinMode(SPI_CS, OUTPUT);

    digitalWrite(SPI_CS, HIGH);

    this->SPI_MOSI = dataPin;

    for (size_t i = 0; i < sizeof(this->status); i++)  {
	    this->status[i] = 0x0;
    }

    for (int i = 0; i < maxDevices; i++) {
        this->spiTransfer(i, OP_DISPLAYTEST, 0);

        // Scanlimit is set to max on startup
        this->setScanLimit(i, 7);

        // Decode is done in source
        this->spiTransfer(i, OP_DECODEMODE, 0);

        this->clearDisplay(i);

        // We go into shutdown-mode on startup
        this->shutdown(i, true);
    }
}

int LedControl::getDeviceCount() {
    return this->maxDevices;
}

void LedControl::shutdown(int addr, bool b) {
    if (this->isValidAddress(addr)) {
        this->spiTransfer(addr, OP_SHUTDOWN, b ? 0 : 1);
    }
}
	
void LedControl::setScanLimit(int addr, int limit) {
    if (this->isValidAddress(addr)) {
        if ((limit >= 0) || (limit < 8)) {
            this->spiTransfer(addr, OP_SCANLIMIT, limit);
        }
    }
}

void LedControl::setIntensity(int addr, int intensity) {
    if (this->isValidAddress(addr)) {
        if ((intensity >= 0) || (intensity < 16)) {
            this->spiTransfer(addr, OP_INTENSITY, intensity);
        }
    }

}

void LedControl::clearDisplay(int addr) {
    if (this->isValidAddress(addr)) {
        int offset = (addr * 8);

        for (int i = 0; i < 8; i++) {
            this->status[offset + i] = 0;
            this->spiTransfer(addr, i + 1, status[offset + i]);
        }
    }
}

void LedControl::setLed(int addr, int row, int column, boolean state) {
    if (this->isValidAddress(addr)) {
        if (this->isValidRow(row) && this->isValidColumn(column)) {
            int offset = (addr * 8) + row;
            byte val = 0b10000000 >> column;

            if (state) {
                this->status[offset] |= val;
            } else {
                val = ~val;
                this->status[offset] &= val;
            }

            this->spiTransfer(addr, row + 1, this->status[offset]);
        }
    }
}
	
void LedControl::setRow(int addr, int row, byte value) {
    if (this->isValidAddress(addr) && this->isValidRow(row)) {
        int offset = (addr * 8) + row;

        this->status[offset] = value;
        this->spiTransfer(addr, row + 1, this->status[offset]);
    }
}

void LedControl::setColumn(int addr, int col, byte value) {
    if (this->isValidAddress(addr) && this->isValidColumn(col)) {
        for(int row = 0; row < NUM_ROWS; row++) {
            byte val = (value >> (7 - row)) & 0x01;
            this->setLed(addr, row, col, val);
        }
    }
}

void LedControl::setDigit(int addr, int digit, byte value,
                          boolean decimalPoint) {
    if (       this->isValidAddress(addr) && this->isValidDigit(digit)
            && this->isValidValue(value)) {

        int  offset = (addr * 8) + digit;
        byte v      = this->charTable[value];

        if (decimalPoint) {
            v |= 0b10000000;
        }

        this->status[offset] = v;
        this->spiTransfer(addr, digit + 1, v);
    }
}

void LedControl::setChar(int addr, int digit, char value,
                         boolean decimalPoint) {
    if (this->isValidAddress(addr)) {
        if (this->isValidDigit(digit)) {
            int  offset = (addr * 8) + digit;
            byte index  = (byte) value;

            if (index > sizeof(charTable)) {
                // Invalid value (nothing defined) -- we'll use the space char
                index = 32;
            }

            byte v = this->charTable[index];

            if (decimalPoint) {
                v |= 0b10000000;
            }

            this->status[offset] = v;
            this->spiTransfer(addr, digit + 1, v);
        }
    }
}

void LedControl::spiTransfer(int addr, volatile byte opcode,
                             volatile byte data) {

    // Create an array with the data to shift out
    int offset   = (addr * 2);
    int maxbytes = (maxDevices * 2);

    for (int i = 0; i < maxbytes; i++) {
        this->spidata[i] = (byte) 0;
    }

    // Put our device data into the array
    this->spidata[offset + 1] = opcode;
    this->spidata[offset]     = data;

    // Enable the line
    digitalWrite(SPI_CS, LOW);

    // Now shift out the data
    for (int i = maxbytes; i > 0; i--) {
        shiftOut(SPI_MOSI, SPI_CLK, MSBFIRST, this->spidata[i - 1]);
    }

    // Latch the data onto the display
    digitalWrite(SPI_CS, HIGH);
}

bool LedControl::isValidAddress(int addr) {
    return ((addr >= 0) && (addr < this->maxDevices));
}

bool LedControl::isValidRow(int row) {
    return ((row >= 0) && (row < NUM_ROWS));
}

bool LedControl::isValidColumn(int col) {
    return ((col >= 0) && (col < NUM_ROWS));
}

bool LedControl::isValidDigit(int digit) {
    return ((digit >=0) && (digit < NUM_ROWS));
}

bool LedControl::isValidValue(int value) {
    return (value < 16);
}
