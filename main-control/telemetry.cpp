#include <Arduino.h>
#include "telemetry.h"

#include <stdint.h>

constexpr uint16_t telem_id_special_strmessage = 0xFFFF;

HardwareSerial* telem_serial = &Serial1;
Stream* telem_save_stream = &Serial;

void print_telem_timestamp() {
	int32_t curMillis = millis();
	int32_t seconds = curMillis / 1000;
	int32_t minutes = seconds / 60;
	int32_t hours = minutes / 60;
	telem_save_stream->printf("%02d:%02d:%02d.%04d ", hours, minutes % 60, seconds % 60, curMillis % 1000);
}

int32_t pauseStartMillis = -10000;
constexpr int NACK_PAUSE_TIME = 200; // milliseconds to pause for resynchronization

void checkNack() {
	// For now, any message sent back over the serial port means a NACK
	if (telem_serial->available() > 0) {
		// debug: what was it?
		Serial.print("received NACK:");
		while (telem_serial->available() > 0)
			Serial.write(telem_serial->read());
		Serial.println();
		// clear and end the transmission
		telem_serial->clear();
		//int availBefore = telem_serial->availableForWrite();
		telem_serial->end();
		telem_serial->begin(9600);
		//Serial.printf("Available before reset: %d; after: %d", availBefore, telem_serial->availableForWrite()); // these better be different // they are!
		pauseStartMillis = millis();
	}

}

void send_telem_packet(uint16_t id, uint16_t length, const void* data) {
	checkNack();
	if ((int32_t) millis() < pauseStartMillis + NACK_PAUSE_TIME) return;
	if (length > telem_serial->availableForWrite()) return;

	telem_serial->write((uint8_t *)&id, sizeof(id));
	telem_serial->write((uint8_t *)&length, sizeof(length));
	telem_serial->write((uint8_t *)data, length);
}

void telem_strmessage(const char* string) {
	telem_serial->print("strmessage: ");
	telem_serial->println(string);
	send_telem_packet(telem_id_special_strmessage, (uint16_t) strlen(string), (const void*) string);
}
