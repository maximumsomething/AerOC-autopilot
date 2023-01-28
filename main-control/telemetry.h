#pragma once
#include <Arduino.h>
#include "telemetry_autogen.h"

#define TELEM_SAVE

// High priority guarantees delivery
void telem_strmessage(const char* string, bool priority = false);

void print_telem_timestamp();
// try to send a telemetry packet.
void dispatch_telem_packet(uint8_t id, uint16_t length, const void* data, bool priority = false);

extern HardwareSerial* telem_serial;
extern Print* telem_save_stream;

bool setupSdCardTelem();
void flushSdCardTelem();
