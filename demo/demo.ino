// hp34401a-lcd.ino: Arduino sketch to display HP 34401A readings on a 16x2 LCD
// as well as send them out the serial port.

// Copyright 2022 Jason Pepas.
// Released under the terms of the MIT License.
// See https://opensource.org/licenses/MIT.

// Setup:

// First, set your 34401A to "talk only" mode:
// Menu -> I/O Menu -> HP-IB Addr -> 31.

// Next, prepare a RS232-to-TTL adapter by soldering a pull-up resistor from
// the DB9 pin 6 (the DSR pin) to VCC (5V).
// See https://gist.github.com/cellularmitosis/1582236f226e9d98075a0c971eb4168c for soldering details.

// Thanks to:
// https://www.eevblog.com/forum/testgear/external-display-for-agilent-34401a-(or-any-dmm-with-rs232-output-stream)/msg326470/#msg326470
// "Set DB9 pin 6 DSR high 5V"


// Pins:

#define LED_pin (13)  // The onboard Arduino LED.

// The RS232-to-TTL adapter will have four pins:
//  Pin 1: GND, connect to Arduino GND.
//  Pin 2: RX, connect to hp_rx_pin.
//  Pin 3: TX, connect to hp_tx_pin.
//  Pin 4: VCC, connect to Arduino 5V.
#define hp_rx_pin (8)
#define hp_tx_pin (9)

// A standard 16x2 "1602" LCD module.
// Pin 1: VSS, connect to Arduino GND.
// Pin 2: VDD, connect to Arduino 5V.
// Pin 3: VO (contrast):
//  Create a 0.6V reference by connecting Arduino 5V or 3.3V to a 10k resistor
//  to a diode to GND.  Connect VO to the 0.6V point of the reference.
// Pin 4: RS, connect to LCD_RS_pin.
// Pin 5: RW, connect to Arduino GND.
// Pin 6: E, connect to LCD_EN_pin.
// Pin 11: D4, connect to LCD_D4_pin.
// Pin 11: D5, connect to LCD_D5_pin.
// Pin 11: D6, connect to LCD_D6_pin.
// Pin 11: D7, connect to LCD_D7_pin.
// Pin 12: A (backlight LED anode):
//  Some 1602 boards need a current limiting resistor and some don't.
//  Connect a 220R from Arduino 5V to the A pin.  If it is very dim, try
//  omitting the resistor.
// Pin 13: K (backlight LED cathode), connect to Arduino GND.
#define LCD_RS_pin (3)
#define LCD_EN_pin (2)
#define LCD_D4_pin (4)
#define LCD_D5_pin (5)
#define LCD_D6_pin (6)
#define LCD_D7_pin (7)


#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS_pin, LCD_EN_pin, LCD_D4_pin, LCD_D5_pin, LCD_D6_pin, LCD_D7_pin);

#include "SoftwareSerial2Stop.h"
SoftwareSerial2Stop hp_serial(hp_rx_pin, hp_tx_pin);

// Note: backslash is not a standard character for these LCD's.
uint8_t backslash[8] = {
    0b00000000,
    0b00010000,
    0b00001000,
    0b00000100,
    0b00000010,
    0b00000001,
    0b00000000,
    0b00000000
};
uint8_t backslash_lcd_index = 0;

char spinner[5] = "-?|/";
uint8_t spinner_index = 0;

void rotate_spinner() {
    spinner_index += 1;
    if (spinner_index >= 4) {
        spinner_index = 0;
    }
}

void print_spinner_char() {
    if (spinner_index == 1) {
        lcd.write(byte(backslash_lcd_index));
    } else {
        lcd.print(spinner[spinner_index]);
    }
}

// Data will stream from the 34401A in the following format:
// -9.34000000E-07\r\n
// +2.24900000E-06\r\n

#define BUF_LEN (17)  // e.g. '+2.24900000E-06\r\n'
char buf[BUF_LEN+1];

#define LCD_WIDTH (16)
char prev_line0[LCD_WIDTH] = {0};
char prev_line1[LCD_WIDTH] = {0};

// only update the changed characters.
void update_lcd_line(int line_idx, char* text) {
    char* prev;
    if (line_idx == 0) {
        prev = prev_line0;
    } else {
        prev = prev_line1;
    }

    bool eof = false;
    for (int i=0; i < LCD_WIDTH; i++) {
        char ch;
        if (eof) {
            ch = ' ';
        } else if (text[i] == '\0') {
            eof = true;
            ch = ' ';
        } else {
            ch = text[i];
        }

        if (ch != prev[i]) {
            lcd.setCursor(i, 0);
            lcd.print(ch);
            prev[i] = ch;
        }
    }
}

void setup() {
    pinMode(LED_pin, OUTPUT);

    lcd.begin(16, 2);
    // Note: backslash is not a standard character for these LCD's.
    lcd.createChar(backslash_lcd_index, backslash);

    Serial.begin(9600);
    hp_serial.begin(9600);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("HP 34401A");
    delay(1000);
}

void loop() {
    // Reset to read the next value.
    memset(buf, '\0', BUF_LEN+1);
    int chars_read = 0;

    while (true) {
        // Spin until there is a char available to read.
        if (hp_serial.available() == 0) {
            continue;
        }

        // Read one char.
        char ch = hp_serial.read();

        // Handle the end of the line.
        if (ch == '\r' || ch == '\n') {
            // The line ending is '\r\n'.  Throw away the '\n'.
            if (chars_read == 0) {
                break;
            }

            // Convert the line to a float.
            // float f = atof(buf);

            // Determine the exponent value, e.g. 'E-02' == 2.
            int exponent = (buf[14] - '0') + (10 * (buf[13] - '0'));
            if (buf[12] == '-') {
                exponent = -exponent;
            }

            // Format the value in non-scientific notation.
            char formatted[12+1];  // e.g. '-123456789.1', '-0.000000001'
            int dot_index = -1;
            // -1.0E-3 = -0.001
            // -1.0E-2 = -0.01
            // -1.0E-1 = -0.1
            // -1.0E0 = -1.0
            // -1.0E1 = -10.0
            // -1.0E2 = -100.0
            // -1.0E3 = -1000.0
            if (exponent < 1) {
                dot_index = 2;
            } else {
                dot_index = 2 + exponent;
            }
            formatted[0] = buf[0]; // the leading '+' or '-'.
            int in_index = 1;
            int out_index = 1;
            int trailing_zeros = 0;
            if (exponent < -1) {
                trailing_zeros = (-exponent) - 1; // e.g. if the exponent is -5, there will be 4 trailing zeros.
            }
            while (in_index < (11 /* the 'E' */ - trailing_zeros)) {
                if (in_index == 2 /* the '.' */) {
                    in_index += 1;
                    continue;
                } else if (out_index == dot_index) {
                    formatted[out_index] = '.';
                    out_index += 1;
                    continue;
                } else if (exponent < 0 && out_index < (-exponent)+2) {
                    formatted[out_index] = '0'; // leading zero's, e.g. 0.001.
                    out_index += 1;
                    continue;
                } else {
                    formatted[out_index] = buf[in_index];
                    in_index += 1;
                    out_index += 1;
                    continue;
                }
            }
            formatted[out_index] = '\0';

            // Format the value in non-scientific notation, with commas and underscores.
            char formatted2[14+1];  // e.g. '-123,456,789.1', '-0.000_000_001'
            int comma1_index = -1;
            int comma2_index = -1;
            int under1_index = -1;
            int under2_index = -1;
            if (exponent >= 6) { // at or above 6 we have two commas and no unders.
                // -1.0E7 = -10,000,000.0
                // -1.0E6 = -1,000,000.0
                dot_index += 2;
                comma1_index = exponent - 4;
                comma2_index = exponent;
            } else if (exponent == 4 || exponent == 5) { // for 4 and 5 we have one comma and no unders.
                // -1.0E+6 = -1,234,567.8
                // -1.0E+5 = -123,456.78
                // -1.0E+4 = -12,345.678
                // -1.0E+3 = -1,234.567_8
                dot_index += 1;
                comma1_index = exponent - 1;
            } else if (exponent == 3) { // for 3 we have one comma and one under.
                // -1.0E+4 = -12,345.678
                // -1.0E+3 = -1,234.567_8
                // -1.0E+2 = -123.456_78
                dot_index += 1;
                under1_index = 10;
            } else if (exponent == 1 || exponent == 2) { // for 2 and 1 we have no commas and one under.
                // -1.0E+3 = -1,234.567_8
                // -1.0E+2 = -123.456_78
                // -1.0E+1 = -12.345_678
                // -1.0E+0 = -1.234_567_8
                under1_index = exponent + 6;
            } else if (exponent <= 0) { // at or below zero, we no commas and two unders.
                // -1.0E+1 = -12.345_678
                // -1.0E+0 = -1.234_567_8
                // -1.0E-1 = -0.123_456_78
                // -1.0E-2 = -0.012_345_678
                // -1.0E-3 = -0.001
                // -1.0E-4 = -0.000_1
                // -1.0E-5 = -0.000_01
                under1_index = 6;
                under2_index = 10;
            }
            formatted2[0] = buf[0]; // the leading '+' or '-'.
            in_index = 1;
            out_index = 1;
            int copied_digits = 0;
            while (in_index < (11 /* the 'E' */ - trailing_zeros)) {
                if (in_index == 2 /* the '.' */) {
                    in_index += 1;
                    continue;
                } else if (out_index == dot_index) {
                    formatted2[out_index] = '.';
                    out_index += 1;
                    continue;
                } else if (out_index == comma1_index || out_index == comma2_index) {
                    formatted2[out_index] = ',';
                    out_index += 1;
                    continue;
                } else if (out_index == under1_index || out_index == under2_index) {
                    formatted2[out_index] = ',';
                    out_index += 1;
                    continue;
                } else if (exponent < 0 && out_index < (-exponent)+2) {
                    formatted2[out_index] = '0'; // leading zero's, e.g. 0.001.
                    out_index += 1;
                    continue;
                } else if (exponent <= -5 && out_index < (-exponent)+3) {
                    formatted2[out_index] = '0'; // leading zero's, e.g. 0.001.
                    out_index += 1;
                    continue;
                } else {
                    formatted2[out_index] = buf[in_index];
                    in_index += 1;
                    out_index += 1;
                    copied_digits += 1;
                    if (copied_digits == 8) {
                        break;
                    }
                    continue;
                }
            }
            formatted2[out_index] = '\0';

            // Send the formatted value out the (hardware) serial port.
            Serial.print(formatted);
            Serial.print("\n");
            digitalWrite(LED_pin, LOW);

            uint8_t line_index = 0;

            // Print the formatted value to the LCD.
            update_lcd_line(line_index, formatted2);

            // Print a spinner.
            lcd.setCursor(LCD_WIDTH-1, line_index);
            print_spinner_char();
            rotate_spinner();

            break;
        }

        // Accumulate chars into the buffer and echo them to the LCD.
        digitalWrite(LED_pin, HIGH);
        buf[chars_read] = ch;
        // if (chars_read == 0) {
        //     lcd.clear();
        // }
        // lcd.setCursor(chars_read, 0);
        // lcd.print(ch);
        chars_read += 1;
        continue;
    }
}
