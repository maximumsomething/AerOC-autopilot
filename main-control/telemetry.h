#pragma once
#include <Arduino.h>
#include "telemetry_autogen.h"

void telem_strmessage(const char* string);

void print_telem_timestamp();
// try to send a telemetry packet.
void dispatch_telem_packet(uint8_t id, uint16_t length, const void* data);

extern HardwareSerial* telem_serial;
extern Stream* telem_save_stream;
