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


void gpib_write(byte_t data) {
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

byte_t gpib_read() {
    set_gpib_data_bus_mode(INPUT_PULLUP);
    byte_t data;
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


// --- Main program ---

#include <SoftwareSerial.h>
SoftwareSerial serial = SoftwareSerial(RX_ARDUINO_PIN, TX_ARDUINO_PIN);

void setup() {
    // Configure the serial pins
    pinMode(RX_ARDUINO_PIN, INPUT);
    pinMode(TX_ARDUINO_PIN, OUTPUT);
    serial.begin(9600);
}

void loop() {
    // let's start by just getting a simple "hello" program to work.
    serial.write("hello!\n");
    delay(1000);
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
