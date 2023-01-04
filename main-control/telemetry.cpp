#include <Arduino.h>
#include "telemetry.h"

#include <stdint.h>

Stream* telem_serial = &Serial;
Stream* telem_save_stream = &Serial1;


void send_telem_packet(uint16_t id, uint16_t length, const void* data) {
	telem_serial->write((uint8_t *)&id, sizeof(id));
	telem_serial->write((uint8_t *)&length, sizeof(length));
	telem_serial->write((uint8_t *)data, length);
}

void telem_strmessage(const char* string) {
	send_telem_packet(0 /*telem_id_special_strmessage*/, (uint16_t) strlen(string), (const void*) string);
}
