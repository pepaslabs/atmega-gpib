/*
  serial.cpp
  Copyright 2017 Jason Pepas
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/

#include "serial.h"

void serial_setup(serial_t *serial, HardwareSerial *hard, SoftwareSerial *soft) {
    if (hard != NULL) {
        serial->hard = hard;
    } else if (soft != NULL) {
        serial->soft = soft;
    }
}

void serial_begin(serial_t *serial, baud_rate_t baud_rate) {
    if (serial->hard != NULL) {
        return hard->begin(baud_rate);
    } else if (serial->soft != NULL) {
        return soft->begin(baud_rate);
    }
}

uint8_t serial_available(serial_t *serial) {
    if (serial->hard != NULL) {
        return (uint8_t)(hard->available());
    } else if (serial->soft != NULL) {
        return (uint8_t)(soft->available());
    }
}

uint8_t serial_read(serial_t *serial) {
    if (serial->hard != NULL) {
        return hard->read();
    } else if (serial->soft != NULL) {
        return soft->read();
    }
}

uint8_t serial_write_available(serial_t *serial) {
    if (serial->hard != NULL) {
        return hard->availableForWrite();
    } else if (serial->soft != NULL) {
        return soft->availableForWrite();
    }
}

size_t serial_write_byte(serial_t *serial, uint8_t data) {
    if (serial->hard != NULL) {
        return hard->write(data);
    } else if (serial->soft != NULL) {
        return soft->write(data);
    }
}

size_t serial_write_str(serial_t *serial, const char *str) {
    if (serial->hard != NULL) {
        return hard->write(str);
    } else if (serial->soft != NULL) {
        return soft->write(str);
    }
}
