// ATmega328/Arduino -> GPIB pin mapping
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

// Notes:

// Next board revision:
// - Move EOI to Arduino pin D4 so that it is controllable via Arduino code.

// Sounds like the listening device is the one which should enable pull-up resistors.
// See http://www.eevblog.com/forum/testgear/gpib-to-usb-controlleradapter/msg349323/#msg349323


#include <SoftwareSerial.h>
#include "buffer.h"
#include "error.h"


// --- Bit manipulation ---

#define SET_BIT(ADDRESS, BITNUM)   (ADDRESS |= (1<<BITNUM))
#define CLEAR_BIT(ADDRESS, BITNUM) (ADDRESS &= ~(1<<BITNUM))
#define FLIP_BIT(ADDRESS, BITNUM)  (ADDRESS ^= (1<<BITNUM))
// note: this returns a byte with the desired bit in its original position
#define GET_BIT(ADDRESS, BITNUM)   (ADDRESS & (1<<BITNUM))


// --- Types ---

typedef uint8_t arduino_pin_t;
typedef uint8_t arduino_pinmode_t;
typedef uint8_t byte_t;
typedef uint8_t bit_t;


// --- TTL Serial pin mapping ---

#define RX_ARDUINO_PIN   1  // connect to FT230X TX (pin 1)
#define TX_ARDUINO_PIN   0  // connect to FT230X RX (pin 4)
#define CTS_ARDUINO_PIN  3
#define RTS_ARDUINO_PIN  4


// --- Handshake bus ---

#define DAV_ARDUINO_PIN   A0
#define NRFD_ARDUINO_PIN  A1
#define NDAC_ARDUINO_PIN  A2


void assert_dav() {
    assert_arduino_pin(DAV_ARDUINO_PIN);
}

void assert_nrfd() {
    assert_arduino_pin(NRFD_ARDUINO_PIN);
}

void assert_ndac() {
    assert_arduino_pin(NDAC_ARDUINO_PIN);
}


void unassert_dav() {
    unassert_arduino_pin(DAV_ARDUINO_PIN);
}

void unassert_nrfd() {
    unassert_arduino_pin(NRFD_ARDUINO_PIN);
}

void unassert_ndac() {
    unassert_arduino_pin(NDAC_ARDUINO_PIN);
}


boolean check_dav() {
    return digitalRead(DAV_ARDUINO_PIN) == LOW ? true : false;
}

boolean check_nrfd() {
    return digitalRead(NRFD_ARDUINO_PIN) == LOW ? true : false;
}

boolean check_ndac() {
    return digitalRead(NDAC_ARDUINO_PIN) == LOW ? true : false;
}


// --- Interface management bus ---

#define IFC_ARDUINO_PIN  A3
#define ATN_ARDUINO_PIN  A5
#define SRQ_ARDUINO_PIN  A4
#define REN_ARDUINO_PIN  13


void assert_ifc() {
    assert_arduino_pin(IFC_ARDUINO_PIN);
}

void assert_atn() {
    assert_arduino_pin(IFC_ARDUINO_PIN);
}

void assert_srq() {
    assert_arduino_pin(SRQ_ARDUINO_PIN);
}

void assert_ren() {
    assert_arduino_pin(REN_ARDUINO_PIN);
}

void assert_eoi() {
    // Configure pin as output
    SET_BIT(DDRB, DDB7);
    // Set the pin low
    CLEAR_BIT(PORTB, PB7);
}


void unassert_ifc() {
    unassert_arduino_pin(IFC_ARDUINO_PIN);
}

void unassert_atn() {
    unassert_arduino_pin(IFC_ARDUINO_PIN);
}

void unassert_srq() {
    unassert_arduino_pin(SRQ_ARDUINO_PIN);
}

void unassert_ren() {
    unassert_arduino_pin(REN_ARDUINO_PIN);
}

void unassert_eoi() {
    // Configure pin as input
    CLEAR_BIT(DDRB, DDB7);
    // Enable the built-in pull-up resistor
    SET_BIT(PORTB, PB7);
}


boolean check_ifc() {
    return digitalRead(IFC_ARDUINO_PIN) == LOW ? true : false;
}

boolean check_atn() {
    return digitalRead(ATN_ARDUINO_PIN) == LOW ? true : false;
}

boolean check_srq() {
    return digitalRead(SRQ_ARDUINO_PIN) == LOW ? true : false;
}

boolean check_ren() {
    return digitalRead(REN_ARDUINO_PIN) == LOW ? true : false;
}

boolean check_eoi() {
    // Configure pin as input
    CLEAR_BIT(DDRB, DDB7);
    // Enable the built-in pull-up resistor
    SET_BIT(PORTB, PB7);

    return GET_BIT(PINB, PINB7) == 0 ? true : false;
}


// --- Data bus ---

#define DIO1_ARDUINO_PIN  8
#define DIO2_ARDUINO_PIN  7
#define DIO3_ARDUINO_PIN  6
#define DIO4_ARDUINO_PIN  5
#define DIO5_ARDUINO_PIN  9
#define DIO6_ARDUINO_PIN  10
#define DIO7_ARDUINO_PIN  11
#define DIO8_ARDUINO_PIN  12


void gpib_set_data_bus(byte_t data) {
    set_gpib_data_bus_mode(OUTPUT);
    digitalWrite(DIO1_ARDUINO_PIN, GET_BIT(data, 0));
    digitalWrite(DIO2_ARDUINO_PIN, GET_BIT(data, 1));
    digitalWrite(DIO3_ARDUINO_PIN, GET_BIT(data, 2));
    digitalWrite(DIO4_ARDUINO_PIN, GET_BIT(data, 3));
    digitalWrite(DIO5_ARDUINO_PIN, GET_BIT(data, 4));
    digitalWrite(DIO6_ARDUINO_PIN, GET_BIT(data, 5));
    digitalWrite(DIO7_ARDUINO_PIN, GET_BIT(data, 6));
    digitalWrite(DIO8_ARDUINO_PIN, GET_BIT(data, 7));
}

byte_t gpib_get_data_bus() {
    set_gpib_data_bus_mode(INPUT_PULLUP);
    byte_t data = 0x0;
    digitalRead(DIO1_ARDUINO_PIN) ? CLEAR_BIT(data, 0) : SET_BIT(data, 0);
    digitalRead(DIO2_ARDUINO_PIN) ? CLEAR_BIT(data, 1) : SET_BIT(data, 1);
    digitalRead(DIO3_ARDUINO_PIN) ? CLEAR_BIT(data, 2) : SET_BIT(data, 2);
    digitalRead(DIO4_ARDUINO_PIN) ? CLEAR_BIT(data, 3) : SET_BIT(data, 3);
    digitalRead(DIO5_ARDUINO_PIN) ? CLEAR_BIT(data, 4) : SET_BIT(data, 4);
    digitalRead(DIO6_ARDUINO_PIN) ? CLEAR_BIT(data, 5) : SET_BIT(data, 5);
    digitalRead(DIO7_ARDUINO_PIN) ? CLEAR_BIT(data, 6) : SET_BIT(data, 6);
    digitalRead(DIO8_ARDUINO_PIN) ? CLEAR_BIT(data, 7) : SET_BIT(data, 7);
    return data;
}

void gpib_release_data_bus() {
    set_gpib_data_bus_mode(INPUT_PULLUP);    
}

void set_gpib_data_bus_mode(arduino_pinmode_t mode) {
    pinMode(DIO1_ARDUINO_PIN, mode);
    pinMode(DIO2_ARDUINO_PIN, mode);
    pinMode(DIO3_ARDUINO_PIN, mode);
    pinMode(DIO4_ARDUINO_PIN, mode);
    pinMode(DIO5_ARDUINO_PIN, mode);
    pinMode(DIO6_ARDUINO_PIN, mode);
    pinMode(DIO7_ARDUINO_PIN, mode);
    pinMode(DIO8_ARDUINO_PIN, mode);
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


// Useful resources:
// - http://www.hp9845.net/9845/tutorials/hpib/


// --- GPIB implementation ---


void send_byte(byte_t data) {
    // wait for all devices to be ready for data.
    while (check_nrfd() == true) { settle(); }

    // place our data onto the bus
    gpib_set_data_bus(data);
    settle();

    // inform the listeners that the data is ready.
    assert_dav();
    settle();

    // wait for all devices to read the data.
    while (check_ndac() == true) { settle(); }

    // indicate that we are done transmitting this byte.
    unassert_dav();
    settle();

    // release the data bus lines.
    gpib_release_data_bus();
    settle();  
}

// Uni-line (single) commands.
// Uni-line commands only require that one bus line be asserted, and do not require a handshake.
// Use the assert_x functions for these commands:
void IFC() {
    assert_ifc();
    settle();
    unassert_ifc();
    settle();
}


// Universal multi-line commands
// Universal multi-line commands work by asserting ATN while putting a data byte on the bus.
// The data byte determines which command is issued.
// Universal multi-line commands apply to all devices on the bus.
#define LAD_BASE  0x20
#define TAD_BASE  0x40
#define SAD_BASE  0x60

byte_t self_gpib_address = 21; // By convention, the controller is typically address 21.

// "Device Listen Address": configure the device at the given address to be a listener.
void LAD(byte_t address) {
    send_byte(LAD_BASE + address);
}

// "Device Talk Address": configure the device at the given address to be a talker.
void TAD(byte_t address) {
    send_byte(TAD_BASE + address);
}

// "Secondary Device Address".
void SAD(byte_t address) {
    send_byte(SAD_BASE + address);
}

// "My Listen Address": declare self to be the listener.
void MLA() {
    LAD(self_gpib_address);
}

// "My Talk Address": declare self to be the talker.
void MTA() {
    TAD(self_gpib_address);
}

// "Unlisten": change all listeners back to an idle state.
void UNL() {
    LAD(31);
}

// "Untalk": change all talkers back to an idle state.
void UNT() {
    TAD(31);
}


// REN: place the device at the given address into REMOTE (listen) mode.
void REN(byte_t address) {
    assert_ren();
    assert_atn();
    settle();

    UNL();
    LAD(address);
    TAD(self_gpib_address);

    unassert_atn();
    settle();
}


void test_K196() {
    REN(17);
    send_byte('D');
    send_byte('A');
    send_byte('T');
    send_byte('M');
    send_byte('E');
    send_byte('G');
    send_byte('A');
    send_byte('X');
}


// Addressed commands
// Addressed commands use multiple bus lines.
// Addressed commands work by asserting ATN while putting a data byte on the bus.
// The data byte determines which command is issued.
// Addressed commands apply to only one device on the bus, so they require some setup.
// Before you can issue one of these commands, you need to set up a device as a listener (or talker).




// --- Low-level remote control protocol ---


// Handshake bus line names:
// v  DAV
// n  NRFD
// k  NDAC

#define LINE_DAV   'v'
#define LINE_NRFD  'n'
#define LINE_NDAC  'k'


// Interface management bus line names:
// i  IFC
// a  ATN
// s  SRQ
// r  REN
// e  EOI

#define LINE_IFC   'i'
#define LINE_ATN   'a'
#define LINE_SRQ   's'
#define LINE_REN   'r'
#define LINE_EOI   'e'


// Data bus names:
// c  Data bus (ASCII character mode)
// x  Data bus (Binary / Hex mode)

#define LINE_DATA_CHAR  'c'
#define LINE_DATA_HEX   'x'


// Commands for individual bus lines:
// t  Assert a line
// f  Unassert a line
// ?  Check the value of a line
// @  Wait until a line is asserted
// !  Wait until a line is unasserted

// Examples:
// tv  Assert DAV
// fn  Unassert NRFD
// ?s  Check the value of SRQ
// @n  Wait until NRFD is asserted
// !e  Wait until EOI is unasserted

#define CMD_ASSERT           't'
#define CMD_UNASSERT         'f'
#define CMD_CHECK            '?'
#define CMD_WAIT_ASSERTED    '@'
#define CMD_WAIT_UNASSERTED  '!'


// Commands for the data bus:
// ?c  Read the data bus lines (prints an ASCII character)
// c   Set the data bus lines (takes one ASCII character as its argument)
// ?x  Read the data bus lines (prints a hexidecimal byte)
// x   Set the data bus lines (takes a hexidecimal byte as its argument)

// Examples:
// ?c   Read and print a ASCII character from the data bus
// cY   Place the ASCII character 'Y' onto the data bus lines
// ?x   Read and print (as hexidecimal) a byte from the data bus lines
// xFF  Place the byte 0xFF onto the data bus lines

#define CMD_WRITE_CHAR  'c'
#define CMD_WRITE_HEX   'x'


// Misc. commands:
// %  Print an informational message ("atmega-gpib\n")

#define CMD_PING   '%'
#define CMD_PERFORM_TEST_1   '1'


// --- GPIB commands ---


void assert_bus_line(char bus_line, SoftwareSerial *serial) {
    switch (bus_line) {
        case LINE_DAV:
            assert_dav();
            break;
        case LINE_NRFD:
            assert_nrfd();
            break;
        case LINE_NDAC:
            assert_ndac();
            break;
        case LINE_IFC:
            assert_ifc();
            break;
        case LINE_ATN:
            assert_atn();
            break;
        case LINE_SRQ:
            assert_srq();
            break;
        case LINE_REN:
            assert_ren();
            break;
        case LINE_EOI:
            assert_eoi();
            break;
    }
    serial->write("ok\n");
}


void unassert_bus_line(char bus_line, SoftwareSerial *serial) {
    switch (bus_line) {
        case LINE_DAV:
            unassert_dav();
            break;
        case LINE_NRFD:
            unassert_nrfd();
            break;
        case LINE_NDAC:
            unassert_ndac();
            break;
        case LINE_IFC:
            unassert_ifc();
            break;
        case LINE_ATN:
            unassert_atn();
            break;
        case LINE_SRQ:
            unassert_srq();
            break;
        case LINE_REN:
            unassert_ren();
            break;
        case LINE_EOI:
            unassert_eoi();
            break;
    }
    serial->write("ok\n");
}


void check_bus_line(char bus_line, SoftwareSerial *serial) {
    boolean line_status;
    switch (bus_line) {
        case LINE_DAV:
            line_status = check_dav();
            break;
        case LINE_NRFD:
            line_status = check_dav();
            break;
        case LINE_NDAC:
            line_status = check_dav();
            break;
        case LINE_IFC:
            line_status = check_dav();
            break;
        case LINE_ATN:
            line_status = check_dav();
            break;
        case LINE_SRQ:
            line_status = check_dav();
            break;
        case LINE_REN:
            line_status = check_dav();
            break;
        case LINE_EOI:
            line_status = check_dav();
            break;
        default:
            print_error(ERROR_UNKNOWN_BUS_LINE, serial);
            return;
    }

    if (line_status == true) {
        serial->write("asserted\n");
    } else {
        serial->write("unasserted\n");        
    }
}


void wait_until_asserted(char bus_line, SoftwareSerial *serial) {
    switch (bus_line) {
        case LINE_DAV:
            while (check_dav() == false) { continue; }
            break;
        case LINE_NRFD:
            while (check_nrfd() == false) { continue; }
            break;
        case LINE_NDAC:
            while (check_ndac() == false) { continue; }
            break;
        case LINE_IFC:
            while (check_ifc() == false) { continue; }
            break;
        case LINE_ATN:
            while (check_atn() == false) { continue; }
            break;
        case LINE_SRQ:
            while (check_srq() == false) { continue; }
            break;
        case LINE_REN:
            while (check_ren() == false) { continue; }
            break;
        case LINE_EOI:
            while (check_eoi() == false) { continue; }
            break;
    }
    serial->write("ok\n");
}


void wait_until_unasserted(char bus_line, SoftwareSerial *serial) {
    switch (bus_line) {
        case LINE_DAV:
            while (check_dav() == true) { continue; }
            break;
        case LINE_NRFD:
            while (check_nrfd() == true) { continue; }
            break;
        case LINE_NDAC:
            while (check_ndac() == true) { continue; }
            break;
        case LINE_IFC:
            while (check_ifc() == true) { continue; }
            break;
        case LINE_ATN:
            while (check_atn() == true) { continue; }
            break;
        case LINE_SRQ:
            while (check_srq() == true) { continue; }
            break;
        case LINE_REN:
            while (check_ren() == true) { continue; }
            break;
        case LINE_EOI:
            while (check_eoi() == true) { continue; }
            break;
    }
    serial->write("ok\n");
}


void write_data(byte_t data, SoftwareSerial *serial) {
    gpib_set_data_bus(data);
    serial->write("ok\n");    
}


void read_char_data(SoftwareSerial *serial) {
    char ch = (char)gpib_get_data_bus();
    serial->write("data: ");
    serial->write(ch);
    serial->write("\n");
}


void read_hex_data(SoftwareSerial *serial) {
    byte_t data = gpib_get_data_bus();
    char buff[3];
    byte_to_hex(data, buff);
    serial->write("data: ");
    serial->write(buff);
    serial->write("\n");
}


// --- Command parsing ---


boolean hex_to_nibble(char ch, byte_t *out) {
    switch (ch) {
        case '0':
            *out = 0x0;
            return true;
        case '1':
            *out = 0x1;
            return true;
        case '2':
            *out = 0x2;
            return true;
        case '3':
            *out = 0x3;
            return true;
        case '4':
            *out = 0x4;
            return true;
        case '5':
            *out = 0x5;
            return true;
        case '6':
            *out = 0x6;
            return true;
        case '7':
            *out = 0x7;
            return true;
        case '8':
            *out = 0x8;
            return true;
        case '9':
            *out = 0x9;
            return true;
        case 'a':
        case 'A':
            *out = 0xA;
            return true;
        case 'b':
        case 'B':
            *out = 0xB;
            return true;
        case 'c':
        case 'C':
            *out = 0xC;
            return true;
        case 'd':
        case 'D':
            *out = 0xD;
            return true;
        case 'e':
        case 'E':
            *out = 0xE;
            return true;
        case 'f':
        case 'F':
            *out = 0xF;
            return true;
        default:
            return false;
    }
}


boolean hex_to_byte(char high, char low, byte_t *out) {
    // originally inspired by http://stackoverflow.com/a/12839870/558735

    byte_t high_bin = 0x0;
    byte_t low_bin = 0x0;
    boolean ret;
    
    ret = hex_to_nibble(high, &high_bin);
    if (ret == false) {
        return ret;
    }

    ret = hex_to_nibble(low, &low_bin);
    if (ret == false) {
        return ret;
    }

    *out = (high_bin << 4) || low_bin;
    return true;
}


void byte_to_hex(byte_t data, char *buff) {
    // originally inspired by http://stackoverflow.com/a/12839870/558735
 
    char map[16+1] = "0123456789ABCDEF";

    byte_t high_nibble = (data & 0xF0) >> 4;
    *buff = map[high_nibble];
    buff++;

    byte_t low_nibble = data & 0x0F;
    *buff = map[low_nibble];
    buff++;

    *buff = '\0';
}


#define MAX_EXPECTED_LINE_LEN 5  // "xFF\r\n"
#define BUFF_LEN (MAX_EXPECTED_LINE_LEN + 1)  // add one to account for terminating NULL char.

char buffer_bytes[BUFF_LEN];
char_buffer_t buffer = { .len = BUFF_LEN, .bytes = buffer_bytes };


// Read from serial until '\n', writing to a buffer.
error_t read_serial_line(SoftwareSerial *serial, char_buffer_t *buffer) {

    uint8_t num_chars_consumed = 0;
    char *buff_ptr = buffer->bytes;
    boolean has_read_first_char = false;

    while (true) {
        // busy-wait for serial data to become available
        while (serial->available() == 0) {
            continue;
        }

        *buff_ptr = serial->read();

        // throw away any leading \n \r garbage leftover from the previous line
        if ((*buff_ptr == '\r' || *buff_ptr == '\n') && (has_read_first_char == false)) {
            continue;
        }
        has_read_first_char = true;
        
        num_chars_consumed++;

        if (*buff_ptr == '\r' || *buff_ptr == '\n') {
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


void parse_and_run_cmd(char_buffer_t *buffer, SoftwareSerial *serial) {
    int buff_len;
    char cmd;
    char bus_line;
    byte_t data_bin;
    char data_hex_high;
    char data_hex_low;
    boolean ret;

    buff_len = strlen(buffer->bytes);
    if (buff_len < 1) {
        print_error(ERROR_MALFORMED_COMMAND, serial);
        return;
    }

    cmd = buffer->bytes[0];
    switch (cmd) {

        case CMD_ASSERT:
            if (buff_len < 2) {
                print_error(ERROR_MALFORMED_COMMAND, serial);
                return;
            }
            bus_line = buffer->bytes[1];
            assert_bus_line(bus_line, serial);
            break;

        case CMD_UNASSERT:
            if (buff_len < 2) {
                print_error(ERROR_MALFORMED_COMMAND, serial);
                return;
            }
            bus_line = buffer->bytes[1];
            unassert_bus_line(bus_line, serial);
            break;

        case CMD_CHECK:
            if (buff_len < 2) {
                print_error(ERROR_MALFORMED_COMMAND, serial);
                return;
            }
            bus_line = buffer->bytes[1];
            if (bus_line == LINE_DATA_CHAR) {
                read_char_data(serial);
            } else if (bus_line == LINE_DATA_HEX) {
                read_hex_data(serial);
            } else {
                check_bus_line(bus_line, serial);
            }
            break;

        case CMD_WAIT_ASSERTED:
            if (buff_len < 2) {
                print_error(ERROR_MALFORMED_COMMAND, serial);
                return;
            }
            bus_line = buffer->bytes[1];
            wait_until_asserted(bus_line, serial);
            break;

        case CMD_WAIT_UNASSERTED:
            if (buff_len < 2) {
                print_error(ERROR_MALFORMED_COMMAND, serial);
                return;
            }
            bus_line = buffer->bytes[1];
            wait_until_unasserted(bus_line, serial);
            break;

        case CMD_WRITE_CHAR:
            if (buff_len < 2) {
                print_error(ERROR_MALFORMED_COMMAND, serial);
                return;
            }
            data_bin = buffer->bytes[1];
            write_data(data_bin, serial);
            break;

        case CMD_WRITE_HEX:
            if (buff_len < 3) {
                print_error(ERROR_MALFORMED_COMMAND, serial);
                return;
            }
            data_hex_high = buffer->bytes[1];
            data_hex_low = buffer->bytes[2];
            ret = hex_to_byte(data_hex_high, data_hex_low, &data_bin);
            if (ret == false) {
                print_error(ERROR_INVALID_HEX, serial);
                return;
            }
            write_data(data_bin, serial);
            break;

        case CMD_PING:
            serial->write("atmega-gpib\n");
            break;

        case CMD_PERFORM_TEST_1:
            test_K196();
            serial->write("ok\n");
            break;

        default:
            print_error(ERROR_UNKNOWN_COMMAND, serial);
            char buff[3];
            byte_to_hex(buffer->bytes[0], buff);
            serial->write("bad command hex: ");
            serial->write(buff);
            serial->write("\n");
            break;
    }
}


void print_error(error_t err, SoftwareSerial *serial) {
    switch (err) {
        case OK_NO_ERROR:
            return;
        case ERROR_BUFFER_FILLED_UP_BEFORE_SENTINEL_REACHED:
            serial->write("Error: buffer filled up before sentinel reached.\n");
            break;
        case ERROR_UNKNOWN_COMMAND:
            serial->write("Error: unknown command.\n");
            break;
        case ERROR_MALFORMED_COMMAND:
            serial->write("Error: malformed command.\n");
            break;
        case ERROR_UNKNOWN_BUS_LINE:
            serial->write("Error: unknown bus line.\n");
            break;
        case ERROR_INVALID_HEX:
            serial->write("Error: invalid hex value.\n");
            break;
    }
}


// --- Main program ---

SoftwareSerial serial = SoftwareSerial(RX_ARDUINO_PIN, TX_ARDUINO_PIN);

void setup() {
    // Configure the serial pins
    pinMode(RX_ARDUINO_PIN, INPUT);
    pinMode(TX_ARDUINO_PIN, OUTPUT);
    serial.begin(9600);
    
    // Initialize the GPIB bus
    unassert_atn();
    unassert_ifc();
    unassert_srq();
    unassert_ren();
    unassert_eoi();

    unassert_dav();
    unassert_ndac();
    unassert_nrfd();

    gpib_release_data_bus();

    settle();
}

void loop() {
    error_t err = read_serial_line(&serial, &buffer);

    if (err != OK_NO_ERROR) {
        print_error(err, &serial);
        return;
    }

    parse_and_run_cmd(&buffer, &serial);
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
// For example, on my linux computer, the "boards.txt" file sits at this path:
// /home/cell/Arduino/hardware/breadboard/avr/boards.txt

// Note: if you get a "permission denied" error when trying to use the USBTiny programmer
// from linux, do the following:
// - Run `lsusb` and look for a line like the following:
//     Bus 001 Device 016: ID 1781:0c9f Multiple Vendors USBtiny
// - The above device corresponds to the device file /dev/bus/usb/001/016.  Chmod it:
//     sudo chmod ugo+rw /dev/bus/usb/001/016
