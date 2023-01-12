#include <Arduino.h>
#include "telemetry.h"

#include <stdint.h>

constexpr uint16_t telem_id_special_strmessage = 0xFF;

HardwareSerial* telem_serial = &Serial1;
Stream* telem_save_stream = &Serial;

void print_telem_timestamp() {
	int32_t curMillis = millis();
	int32_t seconds = curMillis / 1000;
	int32_t minutes = seconds / 60;
	int32_t hours = minutes / 60;
	telem_save_stream->printf("%02d:%02d:%02d.%04d ", hours, minutes % 60, seconds % 60, curMillis % 1000);
}

int32_t pauseEndMillis = -10000;
constexpr int NACK_PAUSE_TIME = 100; // milliseconds to pause for resynchronization
constexpr int ACK_TIMEOUT = 100;
uint8_t telem_seq = 0; // overflows from 255 to 1 (zero is error)
int32_t lastMsgMillis = 0;
int32_t lastAckMillis = 0;

void reset_telem() {
	// debug: what was it?
	Serial.print("Resetting telemetry:");
	while (telem_serial->available() > 0)
		Serial.write(telem_serial->read());
	Serial.println();
	// clear the receive buffer
	telem_serial->clear();
	// clear the transmit buffer
	telem_serial->end();
	telem_serial->begin(9600);
	pauseEndMillis = millis() + NACK_PAUSE_TIME;

	lastAckMillis = millis(); // it isn't really, but
}

void checkAcks() {
	while (telem_serial->available() > 0) {
		uint8_t ack = telem_serial->read();
		if (ack == 0) { // error, pause and reset
			reset_telem();
		}
		// for now, we aren't checking sequence numbers
		else {
			lastAckMillis = millis();
		}
	}
	if (lastAckMillis < lastMsgMillis && millis() - lastAckMillis > ACK_TIMEOUT) {
		reset_telem();
	}
}

void send_telem_packet(uint8_t id, uint16_t length, const void* data) {
	checkAcks();
	// todo: figure out which messages are best to drop.
	if ((int32_t) millis() < pauseEndMillis) return;
	if (length > telem_serial->availableForWrite()) return;

	++telem_seq;
	if (telem_seq == 0) telem_seq = 1;
	//telem_serial->write((uint8_t *)&id, sizeof(id));
	telem_serial->write(id);
	telem_serial->write(telem_seq);
	telem_serial->write((uint8_t *)&length, sizeof(length));
	telem_serial->write((uint8_t *)data, length);

	lastMsgMillis = millis();
}

void telem_strmessage(const char* string) {
	telem_save_stream->print("strmessage: ");
	telem_save_stream->println(string);
	send_telem_packet(telem_id_special_strmessage, (uint16_t) strlen(string), (const void*) string);
}
