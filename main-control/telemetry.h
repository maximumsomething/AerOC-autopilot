#pragma once
#include <Arduino.h>
#include "telemetry_autogen.h"

void telem_strmessage(const char* string);
void send_telem_packet(uint16_t id, uint16_t length, const void* data);

extern Stream* telem_serial;
extern Stream* telem_save_stream;
