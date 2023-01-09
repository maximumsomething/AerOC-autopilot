#include <Arduino.h>
#include "telemetry.h"

#include <stdint.h>

Stream* telem_serial = &Serial1;
Stream* telem_save_stream = &Serial;

void print_telem_timestamp() {
	int32_t curMillis = millis();
	int32_t seconds = curMillis / 1000;
	int32_t minutes = seconds / 60;
	int32_t hours = minutes / 60;
	telem_save_stream->printf("%02d:%02d:%02d.%04d ", hours, minutes % 60, seconds % 60, curMillis % 1000);
}

void send_telem_packet(uint16_t id, uint16_t length, const void* data) {
	telem_serial->write((uint8_t *)&id, sizeof(id));
	telem_serial->write((uint8_t *)&length, sizeof(length));
	telem_serial->write((uint8_t *)data, length);
}

void telem_strmessage(const char* string) {
	telem_serial->print("strmessage: ");
	telem_serial->println(string);
	send_telem_packet(0 /*telem_id_special_strmessage*/, (uint16_t) strlen(string), (const void*) string);
}
