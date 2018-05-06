/*
 * Driver.cpp - A sample application for controling Leds with a MAX7219/MAX7221
 * Copyright (c) 2007 Eberhard Fahle
 *
 * Originally from: http://playground.arduino.cc/Main/LedControl
 * 
 * Minor modifications by Andy Dalton, December 2013
 *     * Cleaned up the formatting a bit
 *     * Introduced named constants
 *     * Changed the row/column examples to be a bit more dynamic
 *     * Changed example text from "Arduino" to "ANDY"
 *     * Cleared the display at the start of each demo function
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
#include <Arduino.h>
#include "LedControl.h"

enum {
    DATA_IN_PIN =  12,
    CLK_PIN     =  11,
    CS_PIN      =  10,
    NUM_ROWS    =   8,
    NUM_COLS    =   8,
    TEXT_DELAY  = 500,
    DELAY_TIME  = 100,
};

/*
 * LedControl object with which to work.
 *     pin 12 is connected to the DataIn
 *     pin 11 is connected to the CLK
 *     pin 10 is connected to LOAD
 *
 * We have only a single MAX72XX.
 */
static LedControl lc = LedControl(DATA_IN_PIN, CLK_PIN, CS_PIN);


void setup() {
    /*
     * The MAX72XX is in power-saving mode on startup,
     * we have to do a wakeup call
     */
    lc.shutdown(0, false);

    /* Set the brightness to a medium values */
    //lc.setIntensity(0, 8);

    /* and clear the display */
    lc.clearDisplay(0);
}

/*
 * This function will display the characters for the  word "ANDY" one after the
 * other on the matrix.  (You need at least 5x7 leds to see the whole chars.)
 */
static void writeAndy() {
    enum { NAME_LEN = 4, };

    lc.clearDisplay(0);

    byte andy[NAME_LEN][NUM_COLS] = {
         { 0b01110000,
           0b10001000,
           0b11111000,
           0b10001000,
           0b10001000
         },
         { 0b10001000,
           0b11001000,
           0b10101000,
           0b10011000,
           0b10001000
         },
         { 0b11100000,
           0b10010000,
           0b10010000,
           0b10010000,
           0b11100000
         },
         { 0b10001000,
           0b01010000,
           0b00100000,
           0b00100000,
           0b00100000
         }
    };

    /* Now display them one by one with a small delay */
    for (int i = 0; i < NAME_LEN; ++i) {
        lc.setRow(0, 0, andy[i][0]);
        lc.setRow(0, 1, andy[i][1]);
        lc.setRow(0, 2, andy[i][2]);
        lc.setRow(0, 3, andy[i][3]);
        lc.setRow(0, 4, andy[i][4]);

        delay(TEXT_DELAY);
    }
}

/*
 * This function lights up a some Leds in a row.  The first row will have one
 * LED lit, the second two, and so on.  As each row is lit, it will blink row
 * number of times.
 */
static void rows() {
    byte display   = 0b00000000;
    byte turnOnBit = 0b10000000;

    lc.clearDisplay(0);

    for (int row = 0; row < NUM_ROWS; row++) {
        display |= turnOnBit;
        turnOnBit >>= 1;

        for (int i = 0; i < row; i++) {
            delay(DELAY_TIME);
            lc.setRow(0, row, display);

            delay(DELAY_TIME);
            lc.setRow(0, row, (byte) 0);
        }

        delay(DELAY_TIME);
        lc.setRow(0, row, display);

        delay(DELAY_TIME);
    }
}

/*
 * This function lights up a some Leds in a column.  The first column will have
 * one LED lit, the second two, and so on.  As each column is lit, it will blink
 * column number of times.
 */
static void columns() {
    byte display   = 0b00000000;
    byte turnOnBit = 0b00000001;

    lc.clearDisplay(0);

    for (int col = 0; col < NUM_COLS; col++) {
        display |= turnOnBit;
        turnOnBit <<= 1;

        for (int i = 0; i < col; i++) {
            delay(DELAY_TIME);
            lc.setColumn(0, col, display);

            delay(DELAY_TIME);
            lc.setColumn(0, col, (byte) 0);
        }

        delay(DELAY_TIME);
        lc.setColumn(0, col, display);

        delay(DELAY_TIME);
    }
}

/*
 * This function will light up every Led on the matrix.  The led will blink
 * along with the row number.
 */
static void single() {
    lc.clearDisplay(0);

    for (int row = 0; row < NUM_ROWS; row++) {
        for (int col = 0; col < NUM_COLS; col++) {
            delay(DELAY_TIME);
            lc.setLed(0, row, col, true);

            delay(DELAY_TIME);

            for (int i = 0; i < col; i++) {
                lc.setLed(0, row, col, false);
                delay(DELAY_TIME);

                lc.setLed(0, row, col, true);
                delay(DELAY_TIME);
            }
        }
    }
}

/*
 * Turn the led at the given row/colum on for DELAY TIME then off for
 * DELAY_TIME.
 */
static void blinkLed(int row, int col) {
    lc.setLed(0, row, col, true);
    delay(DELAY_TIME);

    lc.setLed(0, row, col, false);
    delay(DELAY_TIME);
}

/*
 * Demo to blink a single LED that moves in a zig-zag pattern across the
 * display, first from size to size then up and down.
 */
static void zigzag() {
    lc.clearDisplay(0);

    for (int row = 0; row < NUM_ROWS; ++row) {
        if ((row % 2) == 0) {
            for (int col = 0; col < NUM_COLS; ++col) {
                blinkLed(row, col);
            }
        } else {
            for (int col = NUM_COLS - 1; col >= 0; --col) {
                blinkLed(row, col);
            }
        }
    }

    for (int col = 0; col < NUM_COLS; ++col) {
        if ((col % 2) == 0) {
            for (int row = 0; row < NUM_ROWS; ++row) {
                blinkLed(row, col);
            }
        } else {
            for (int row = NUM_COLS - 1; row >= 0; --row) {
                blinkLed(row, col);
            }
        }
    }



}

void loop() {
    zigzag();
    writeAndy();
    rows();
    columns();
    single();
}
