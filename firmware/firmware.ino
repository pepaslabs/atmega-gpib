// Arduino firmware for the atmega-gpib project.
// See https://github.com/pepaslabs/atmega-gpib

// Copyright 2017 Jason Pepas
// Released under the terms of the MIT License.  See https://opensource.org/licenses/MIT


// Uncomment one of these lines, depending upon which board you are using:
// #define BOARDV1
#define BOARDV2


// ATmega328/Arduino -> GPIB pin mapping for the v2 board (using the MCP2221A)
//
//                       +------------\_/------------+
//                      -|1  PC6 INT14   INT13 PC5 28|- A5     GPIB 11: ATN
// MCP2221 6: TX     D0 -|2  PD0 INT16   INT12 PC4 27|- A4     GPIB 10: SRQ
// MCP2221 5: RX     D1 -|3  PD1 INT17   INT11 PC3 26|- A3     GPIB 9:  IFC
//                   D2 -|4  PD2 INT18   INT10 PC2 25|- A2     GPIB 8:  NDAC
//  GPIB 17: REN     D3 -|5  PD3 INT19    INT9 PC1 24|- A1     GPIB 7:  NRFD
//   GPIB 5: EOI     D4 -|6  PD4 INT20    INT8 PC0 23|- A0     GPIB 6:  DAV
//                  Vcc -|7                        22|- GND
//                  GND -|8                        21|- AREF
//                      -|9  PB6 INT6              20|- AVcc
//                      -|10 PB7 INT7     INT5 PB5 19|- D13
//   GPIB 4: DIO4    D5 -|11 PD5 INT21    INT4 PB4 18|- D12    GPIB 16: DIO8
//   GPIB 3: DIO3    D6 -|12 PD6 INT22    INT3 PB3 17|- D11    GPIB 15: DIO7
//   GPIB 2: DIO2    D7 -|13 PD7 INT23    INT2 PB2 16|- D10    GPIB 14: DIO6
//   GPIB 1: DIO1    D8 -|14 PB0 INT0     INT1 PB1 15|- D9     GPIB 13: DIO5
//                       +---------------------------+


// ATmega328/Arduino -> GPIB pin mapping for the v1 board (using the FT230X)
//
// NOTE: I got the RX/TX pins backwards on this board, which is why it uses
// a software serial port (and can't be updated via bootloader).
// Also, REN used D13, which conflicted with the built-in Arduino LED.
//
//                       +------------\_/------------+
//                      -|1  PC6 INT14   INT13 PC5 28|- A5     GPIB 11: ATN
//  FT230X 4: RX     D0 -|2  PD0 INT16   INT12 PC4 27|- A4     GPIB 10: SRQ
//  FT230X 1: TX     D1 -|3  PD1 INT17   INT11 PC3 26|- A3     GPIB 9:  IFC
//  FT230X 6: CTS    D2 -|4  PD2 INT18   INT10 PC2 25|- A2     GPIB 8:  NDAC
//  FT230X 2: RTS    D3 -|5  PD3 INT19    INT9 PC1 24|- A1     GPIB 7:  NRFD
//                   D4 -|6  PD4 INT20    INT8 PC0 23|- A0     GPIB 6:  DAV
//                  Vcc -|7                        22|- GND
//                  GND -|8                        21|- AREF
//                      -|9  PB6 INT6              20|- AVcc
//   GPIB 5: EOI        -|10 PB7 INT7     INT5 PB5 19|- D13    GPIB 17: REN
//   GPIB 4: DIO4    D5 -|11 PD5 INT21    INT4 PB4 18|- D12    GPIB 16: DIO8
//   GPIB 3: DIO3    D6 -|12 PD6 INT22    INT3 PB3 17|- D11    GPIB 15: DIO7
//   GPIB 2: DIO2    D7 -|13 PD7 INT23    INT2 PB2 16|- D10    GPIB 14: DIO6
//   GPIB 1: DIO1    D8 -|14 PD8 INT0     INT1 PB1 15|- D9     GPIB 13: DIO5
//                       +---------------------------+


#define FIRMWARE_VERSION "2.1.1"

/*
Firmware history:
- 2.0.1: Adding "++ver" command.  Adding support for v1 and v2 boards.
- 2.1.1: Adding "++baud" command.
*/


// --- Command documentation ---

// ++ver
// Prints the version of this firmware.

// ++baud 19200
// Switch the serial port to a different baud rate.
// Note: this setting is not remembered across reboots.

// ++addr 17
// Set the address of the remote GPIB device.

// ++read
// Address the remote GPIB to talk, read from the device until EOI, then
// print the result.

// ++stream
// Similar to ++read, but in an infinite loop.

// Any string not beginning with "++" is sent to the remote GPIB device.

// Commands are terminated with '\n' or '|'.


// ----------------------------------------------------------------------------

// The v1 board has to use a software serial port.
#ifdef BOARDV1

#define SOFT_UART

#endif


#ifdef SOFT_UART
#include <SoftwareSerial.h>
#endif

#include "buffer.h"
#include "error.h"
#include "bitmanip.h"
#include "hex.h"
#include "parse.h"


// --- Types ---

typedef uint8_t arduino_pin_t;
typedef uint8_t arduino_pinmode_t;

typedef enum {
    DAV,
    NRFD,
    NDAC,
    IFC,
    ATN,
    SRQ,
    REN,
    EOI,
    DIO1,
    DIO2,
    DIO3,
    DIO4,
    DIO5,
    DIO6,
    DIO7,
    DIO8
} gpib_line_t;


// --- Global vars ---

const uint8_t gpib_buffer_len = 128;
uint8_t gpib_buffer_bytes[gpib_buffer_len];
buffer_t gpib_buffer;

const uint8_t serial_buffer_len = 128;
uint8_t serial_buffer_bytes[serial_buffer_len];
buffer_t serial_buffer;

uint8_t self_gpib_address = 21; // By convention, the controller is typically address 21.
int8_t remote_addr = -1;


// --- Serial ---

#ifdef BOARDV1
#define RX_ARDUINO_PIN   1  // connect to FT230X TX (pin 1)
#define TX_ARDUINO_PIN   0  // connect to FT230X RX (pin 4)
#define CTS_ARDUINO_PIN  3  // (Note: I ended up not using these flow-control pins)
#define RTS_ARDUINO_PIN  4  // (Note: I ended up not using these flow-control pins)
#endif

const uint32_t default_baud = 9600;

#ifdef SOFT_UART
SoftwareSerial serial = SoftwareSerial(RX_ARDUINO_PIN, TX_ARDUINO_PIN);
#endif

int serial_available() {
    #ifdef SOFT_UART
        return serial.available();
    #else
        return Serial.available();
    #endif   
}

size_t serial_write_str(const char *str) {
    #ifdef SOFT_UART
        return serial.write(str);
    #else
        return Serial.write(str);
    #endif
}

int serial_read() {
    #ifdef SOFT_UART
        return serial.read();
    #else
        return Serial.read();
    #endif    
}

void serial_begin(uint32_t baud_rate) {
    #ifdef SOFT_UART
        // Configure the serial pins
        pinMode(RX_ARDUINO_PIN, INPUT);
        pinMode(TX_ARDUINO_PIN, OUTPUT);
        serial.begin(baud_rate);
    #else
        Serial.begin(baud_rate);
    #endif
}


// --- GPIB implementation ---

// See this excellent resource on how GPIB works:
// http://www.pearl-hifi.com/06_Lit_Archive/15_Mfrs_Publications/20_HP_Agilent/HP_7470A/HP-IB_Tutorial_Description.pdf

arduino_pin_t gpib_arduino_pin_map(gpib_line_t line) {
    switch (line) {
        case DAV:
            return A0;
        case NRFD:
            return A1;
        case NDAC:
            return A2;
        case IFC:
            return A3;
        case ATN:
            return A5;
        case SRQ:
            return A4;
        case REN:
            #ifdef BOARDV1
                return 13;
            #elif defined BOARDV2
                return 3;
            #endif
        case EOI:
            #ifdef BOARDV2
                return 4;
            #endif
        case DIO1:
            return 8;
        case DIO2:
            return 7;
        case DIO3:
            return 6;
        case DIO4:
            return 5;
        case DIO5:
            return 9;
        case DIO6:
            return 10;
        case DIO7:
            return 11;
        case DIO8:
            return 12;
        default:
            return 255;
    }
}


void assert_gpib_line(gpib_line_t line) {

#ifdef BOARDV1
    if (line == EOI) {
        // Configure pin as output
        SET_BIT(DDRB, DDB7);        
        // Set the pin low
        CLEAR_BIT(PORTB, PB7);
        return;
    }
#endif

    assert_arduino_pin(gpib_arduino_pin_map(line));
}


void unassert_gpib_line(gpib_line_t line) {

#ifdef BOARDV1
    if (line == EOI) {
        // Configure pin as input
        CLEAR_BIT(DDRB, DDB7);
        // Enable the built-in pull-up resistor
        SET_BIT(PORTB, PB7);
        return;
    }
#endif

    unassert_arduino_pin(gpib_arduino_pin_map(line));
}


bool check_gpib_line(gpib_line_t line) {

    #ifdef BOARDV1
    if (line == EOI) {
        // Configure pin as input
        CLEAR_BIT(DDRB, DDB7);
        // Enable the built-in pull-up resistor
        SET_BIT(PORTB, PB7);
        return GET_BIT(PINB, PINB7) == 0 ? true : false;
    }
    #endif

    return digitalRead(gpib_arduino_pin_map(line)) == LOW ? true : false;
}


bool wait_for_line_to_assert(gpib_line_t line) {
    // this will time out after 6.5535 seconds.
    for (uint16_t i = 0; i < UINT16_MAX; i++) {
        if (check_gpib_line(line) == true) {
            return true;
        }
        else {
            delayMicroseconds(100);
            settle();
            continue;
        }
    }

    // timeout!
    return false;
}


bool wait_for_line_to_unassert(gpib_line_t line) {
    // this will time out after 6.5535 seconds.
    for (uint16_t i = 0; i < UINT16_MAX; i++) {
        if (check_gpib_line(line) == false) {
            return true;
        }
        else {
            delayMicroseconds(100);
            continue;
        }
    }

    // timeout!
    return false;
}


// --- GPIBG data bus functions ---

void gpib_set_data_bus(uint8_t data) {
    set_gpib_data_bus_mode(OUTPUT);
    uint8_t inverted_data = ~data;
    for (uint8_t offset = 0; offset < 8; offset++) {
        gpib_line_t line = (gpib_line_t)(DIO1 + offset);
        arduino_pin_t pin = gpib_arduino_pin_map(line);
        uint8_t bit_index = 0 + offset;
        uint8_t value = GET_BIT(inverted_data, bit_index) ? HIGH : LOW;
        digitalWrite(pin, value);
    }
}


uint8_t gpib_get_data_bus() {
    set_gpib_data_bus_mode(INPUT_PULLUP);
    uint8_t data = 0x0;
    for (uint8_t offset = 0; offset < 8; offset++) {
        gpib_line_t line = (gpib_line_t)(DIO1 + offset);
        arduino_pin_t pin = gpib_arduino_pin_map(line);
        uint8_t bit_index = 0 + offset;
        digitalRead(pin) ? CLEAR_BIT(data, bit_index) : SET_BIT(data, bit_index);
    }
    return data;
}


void gpib_release_data_bus() {
    set_gpib_data_bus_mode(INPUT_PULLUP);
}


void set_gpib_data_bus_mode(arduino_pinmode_t mode) {
    for (uint8_t offset = 0; offset < 8; offset++) {
        gpib_line_t line = (gpib_line_t)(DIO1 + offset);
        arduino_pin_t pin = gpib_arduino_pin_map(line);
        pinMode(pin, mode);
    }
}


// --- GPIB communication functions ---

bool send_byte(uint8_t data) {
    bool ret;

    // wait for all devices to be ready for data.
    ret = wait_for_line_to_unassert(NRFD);
    if (ret == false) { return false; }

    // place our data onto the bus
    gpib_set_data_bus(data);
    settle();

    // inform the listeners that the data is ready.
    assert_gpib_line(DAV);
    settle();

    // wait for all devices to read the data.
    ret = wait_for_line_to_unassert(NDAC);
    if (ret == false) { return false; }

    // indicate that we are done transmitting this byte.
    unassert_gpib_line(DAV);
    settle();

    // release the data bus lines.
    gpib_release_data_bus();
    settle();

    return true;
}


bool send_last_byte(byte data) {
    assert_gpib_line(EOI);
    if (send_byte(data) == false) { return false; }
    unassert_gpib_line(EOI);
    settle();
    return true;
}


bool send_str(const char *buff) {
    if (remote_addr < 0) {
        return false;        
    }

    address_listener(remote_addr);

    size_t len = strlen(buff);
    for (size_t i = 0; i < len; i++) {
        if (i == len-1) {
            send_last_byte(buff[i]);
        } else {
            send_byte(buff[i]);
        }
    }

    return true;
}


bool receive_byte(uint8_t *data, bool *eoi) {
    bool ret;

    if (check_gpib_line(NRFD) == false || check_gpib_line(NDAC) == false) {
        assert_gpib_line(NRFD);        
        assert_gpib_line(NDAC);
        settle();
    }

    if (check_gpib_line(ATN) == true) {
        unassert_gpib_line(ATN);
        settle();
    }

    unassert_gpib_line(NRFD);

    // wait for the talker to place a byte on the bus
    ret = wait_for_line_to_assert(DAV);
    if (ret == false) { return false; }

    // read the byte
    *data = gpib_get_data_bus();
    *eoi = check_gpib_line(EOI);

    // inform the talker that we have read the byte
    unassert_gpib_line(NDAC);
    settle();

    // wait for the talker to retract the byte from the bus
    ret = wait_for_line_to_unassert(DAV);
    if (ret == false) { return false; }
    
    assert_gpib_line(NDAC);
    settle();

    return true;
}


// Uni-line (single) commands.
// Uni-line commands only require that one bus line be asserted, and do not require a handshake.
// Use the assert_x functions for these commands:
void clear_all_interfaces() {
    assert_gpib_line(IFC);
    settle();
    unassert_gpib_line(IFC);
    settle();
}


// Universal multi-line commands
// Universal multi-line commands work by asserting ATN while putting a data byte on the bus.
// The data byte determines which command is issued.
// Universal multi-line commands apply to all devices on the bus.
#define LAD_BASE  0x20
#define TAD_BASE  0x40
#define SAD_BASE  0x60


// "Device Listen Address": configure the device at the given address to be a listener.
void LAD(uint8_t address) {
    send_byte(LAD_BASE + address);
}
               
// "Device Talk Address": configure the device at the given address to be a talker.
void TAD(uint8_t address) {
    send_byte(TAD_BASE + address);
}

// "Secondary Device Address".
void SAD(uint8_t address) {
    send_byte(SAD_BASE + address);
}

// "Unlisten": change all listeners back to an idle state.
void UNL() {
    LAD(31);
}

// "Untalk": change all talkers back to an idle state.
void UNT() {
    TAD(31);
}


// Address a remote device to listen.
void address_listener(uint8_t address) {
    assert_gpib_line(REN);
    settle();

    assert_gpib_line(ATN);
    settle();

    UNL();
    UNT();
    LAD(address);
    TAD(self_gpib_address);

    unassert_gpib_line(ATN);
    settle();
}


// Address a remote device to talk.
void address_talker(uint8_t address) {
    assert_gpib_line(REN);
    settle();

    assert_gpib_line(ATN);
    settle();

    UNL();
    UNT();
    LAD(self_gpib_address);
    TAD(address);

    unassert_gpib_line(ATN);
    settle();
}


// --- Arduino pin driver ---

void assert_arduino_pin(arduino_pin_t pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}


void unassert_arduino_pin(arduino_pin_t pin) {
    pinMode(pin, INPUT_PULLUP);
}


// --- Misc utils functions ---

void settle() {
    delayMicroseconds(150);
}


// --- Command parsing ---

// Read from serial until '\n', writing to a buffer.
error_t read_serial_line(buffer_t *buffer) {

    uint8_t num_chars_consumed = 0;
    char *buff_ptr = buffer->str;
    boolean has_read_first_char = false;

    while (true) {
        // busy-wait for serial data to become available
        while (serial_available() == 0) {
            continue;
        }

        *buff_ptr = serial_read();

        // throw away any leading \n \r garbage leftover from the previous line
        if (is_sentinel(*buff_ptr) && (has_read_first_char == false)) {
            continue;
        }
        has_read_first_char = true;
        
        num_chars_consumed++;

        if (is_sentinel(*buff_ptr)) {
            *buff_ptr = '\0';
            return OK_NO_ERROR;
        } else if (num_chars_consumed == buffer->len - 1) {
            *buff_ptr = '\0';
            return ERROR_BUFFER_FILLED_UP_BEFORE_SENTINEL_REACHED;
        } else {
            buff_ptr++;
            continue;
        }
    }
}


const char *sentinels = "\r\n|";

boolean is_sentinel(char ch) {
    for (size_t i = 0; i < strlen(sentinels); i++) {
        if (ch == sentinels[i]) {
            return true;
        }
    }
    return false;
}

    
bool try_ver_cmd(char *buff) {
    
    if (strncmp(buff, "++ver", strlen("++ver")) != 0) {
        return false;
    }

    serial_write_str(FIRMWARE_VERSION);
    return true;
}


bool try_baud_cmd(char *buff) {
    
    if (strncmp(buff, "++baud ", strlen("++baud ")) != 0) {
        return false;
    }

    buff += strlen("++baud ");

    bool ret;
    uint32_t baud;
    ret = parse_uint32(buff, &baud);
    if (ret == false) { return false; }

    serial_begin(baud);
    return true;
}


bool try_addr_cmd(char *buff, int8_t *addr) {

    if (strncmp(buff, "++addr ", strlen("++addr ")) != 0) {
        return false;
    }

    buff += strlen("++addr ");

    uint8_t address = 0;
    bool ret;
    ret = parse_uint8(buff, &address);
    *addr = (int8_t)address;
    return ret;
}


bool try_read_cmd(char *buff) {

    if (strncmp(buff, "++read", strlen("++read")) != 0) {
        return false;
    }

    if (remote_addr < 0) {
        return false;        
    }

    address_talker(remote_addr);

    bool is_eoi;
    bool ret;
    for (uint8_t i = 0; i < gpib_buffer.len - 1; i++) {
        ret = receive_byte(gpib_buffer.bytes + i, &is_eoi);
        if (ret == false || is_eoi == true) {
            gpib_buffer.bytes[i+1] = '\0';
            break;
        }
    }

    serial_write_str(gpib_buffer.str);

    return ret;
}


bool try_stream_cmd(char *buff) {

    if (strncmp(buff, "++stream", strlen("++stream")) != 0) {
        return false;
    }

    if (remote_addr < 0) {
        return false;        
    }

    address_talker(remote_addr);

    while (true) {        
        bool is_eoi;
        bool ret;
        for (uint8_t i = 0; i < gpib_buffer.len - 1; i++) {
            ret = receive_byte(gpib_buffer.bytes + i, &is_eoi);
            if (ret == false || is_eoi == true) {
                gpib_buffer.bytes[i+1] = '\0';
                break;
            }
        }
        serial_write_str(gpib_buffer.str);
        if (ret == false) { return ret; }
        is_eoi = false;
    }

    return true;
}


bool try_eevblog_cmd(char *buff) {

    if (strncmp(buff, "++eevblog", strlen("++eevblog")) != 0) {
        return false;
    }

    if (remote_addr < 0) {
        return false;        
    }

    while (true) {
        send_str("D@@@@@@@@@@X");
        delay(250);
        send_str("DG@@@@@@@@@X");
        delay(250);
        send_str("DOG@@@@@@@@X");
        delay(250);
        send_str("DLOG@@@@@@@X");
        delay(250);
        send_str("DBLOG@@@@@@X");
        delay(250);
        send_str("DVBLOG@@@@@X");
        delay(250);
        send_str("DEVBLOG@@@@X");
        delay(250);
        send_str("DEEVBLOG@@@X");
        delay(250);
        send_str("D@EEVBLOG@@X");
        delay(250);
        send_str("D@@EEVBLOG@X");
        delay(250);
        send_str("D@@@EEVBLOGX");
        delay(250);
        send_str("D@@@@EEVBLOX");
        delay(250);
        send_str("D@@@@@EEVBLX");
        delay(250);
        send_str("D@@@@@@EEVBX");
        delay(250);
        send_str("D@@@@@@@EEVX");
        delay(250);
        send_str("D@@@@@@@@EEX");
        delay(250);
        send_str("D@@@@@@@@@EX");
        delay(250);
        send_str("D@@@@@@@@@@X");
        delay(250);
    }
}


bool parse_and_run_prologix_style_cmd(char *buff) {

    if (strncmp(buff, "++", 2) == 0) {
        if (try_ver_cmd(buff)) { return true; }
        if (try_baud_cmd(buff)) { return true; }
        if (try_addr_cmd(buff, &remote_addr)) { return true; }
        if (try_stream_cmd(buff)) { return true; }
        if (try_read_cmd(buff)) { return true; }
        if (try_eevblog_cmd(buff)) { return true; }
    } else {
        return send_str(buff);
    }

    return false;
}


void print_error(error_t err) {
    switch (err) {
        case OK_NO_ERROR:
            return;
        case ERROR_BUFFER_FILLED_UP_BEFORE_SENTINEL_REACHED:
            serial_write_str("Error: buffer filled up before sentinel reached.\n");
            break;
        case ERROR_UNKNOWN_COMMAND:
            serial_write_str("Error: unknown command.\n");
            break;
        case ERROR_MALFORMED_COMMAND:
            serial_write_str("Error: malformed command.\n");
            break;
        case ERROR_UNKNOWN_BUS_LINE:
            serial_write_str("Error: unknown bus line.\n");
            break;
        case ERROR_INVALID_HEX:
            serial_write_str("Error: invalid hex value.\n");
            break;
    }
}


// --- Main program ---

void setup() {
    serial_begin(default_baud);

    gpib_buffer.len = gpib_buffer_len;
    gpib_buffer.bytes = gpib_buffer_bytes;

    serial_buffer.len = serial_buffer_len;
    serial_buffer.bytes = serial_buffer_bytes;

    // Blink the built-in LED to show we are running.
    // (The Arduino bootloader can take up to 10 seconds before our program
    // start running, so this blink is a useful indicator.)
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);

    // Initialize the GPIB bus
    unassert_gpib_line(ATN);
    unassert_gpib_line(IFC);
    unassert_gpib_line(SRQ);
    unassert_gpib_line(REN);
    unassert_gpib_line(EOI);

    unassert_gpib_line(DAV);
    unassert_gpib_line(NDAC);
    unassert_gpib_line(NRFD);

    gpib_release_data_bus();

    settle();
}


void loop() {
    error_t err = read_serial_line(&serial_buffer);

    if (err != OK_NO_ERROR) {
        print_error(err);
        return;
    }

    if (parse_and_run_prologix_style_cmd(serial_buffer.str)) {
        serial_write_str("ok\n");
    } else {
        serial_write_str("error\n");        
    }
}


// I burn this program onto an ATmega328 using the Arduino 1.6.x IDE by writing it to the
// ATmega328 in an Arduino Uno board.  I then remove the ATmega328 from the Uno and insert
// it into the atmega-gpib board.

// To accomplish this, you'll need to set the ATmega328's fuse bits so that it is configured
// to use its internal 8MHz oscillator.  See the instructions at the bottom of this page:
// https://www.arduino.cc/en/Tutorial/ArduinoToBreadboard

// Download this Arduino hardware configuration:
// https://www.arduino.cc/en/uploads/Tutorial/breadboard-1-6-x.zip
// Create a "hardware" folder in your Arduino folder and move the breadboard folder there.
// On my Linux computer, the "boards.txt" file sits at this path:
//  /home/cell/Arduino/hardware/breadboard/avr/boards.txt
// On my Mac, the "boards.txt" file sits at this path:
//  /Users/cell/Documents/Arduino/hardware/breadboard/avr/boards.txt
// Note: on Mac, Arduino complained about not being able to find avrdude.conf.
// I resolved this by creating a symlink:
//  cd /Users/cell/Documents/Arduino/hardware/breadboard/avr
//  ln -s /Applications/Arduino.app/Contents/Java/hardware/tools/avr/etc/avrdude.conf

// Note: if you get a "permission denied" error when trying to use the USBTiny programmer
// from linux, do the following:
// - Run `lsusb` and look for a line like the following:
//     Bus 001 Device 016: ID 1781:0c9f Multiple Vendors USBtiny
// - The above device corresponds to the device file /dev/bus/usb/001/016.  Chmod it:
//     sudo chmod ugo+rw /dev/bus/usb/001/016
