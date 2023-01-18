#include <Arduino.h>
#include "ringbuffer.h"
#include "telemetry.h"
#include <assert.h>

#include <stdint.h>
#include <vector>

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
constexpr int ACK_TIMEOUT = 200;
uint8_t telem_seq = 0; // overflows from 255 to 1 (zero is error)
int32_t lastMsgMillis = 0;
int32_t lastAckMillis = 0;


// message queue is a linked list
struct MessageNode {
	MessageNode* older = nullptr, *newer = nullptr;
	uint8_t id;
	uint16_t length;
	void* contents = nullptr;
};
// Contains one node for each message type
MessageNode queuedMessages[telem_num_ids];

MessageNode* getQueuedMessage(enum telem_id id) {
	assert(id > 0 && id <= telem_num_ids);
	return &queuedMessages[id - 1];
}

MessageNode* queueHead = nullptr, *queueTail = nullptr;

struct PriorityMessage {
	uint8_t id;
	uint8_t seq; //
	uint16_t length;
	char contents[];
};
std::vector<PriorityMessage *> priorityMessages;
// keep these around in case we have to retransmit
std::vector<PriorityMessage *> sentPriorityMessages;


void reset_telem() {
	while (telem_serial->available() > 0)
		Serial.write(telem_serial->read());
	Serial.println();
	// clear the receive buffer
	telem_serial->clear();
	// clear the transmit buffer
	telem_serial->end();
	telem_serial->begin(9600);

	// Queue for resending all unsent priority messages
	for (PriorityMessage* msg : sentPriorityMessages) {
		priorityMessages.push_back(msg);
	}
	sentPriorityMessages.clear();

	pauseEndMillis = millis() + NACK_PAUSE_TIME;
	lastAckMillis = millis(); // it isn't really, but
}

void checkAcks() {
	while (telem_serial->available() > 0) {
		uint8_t ack = telem_serial->read();
		//Serial.printf("Recieved ack of %d\n", ack);
		if (ack == 0) { // error, pause and reset
			Serial.println("NACK, resetting telemetry");
			reset_telem();
		}
		else {
			lastAckMillis = millis();
			// check for receipt of priority messages
			for (int i = sentPriorityMessages.size() - 1; i >= 0; --i) {
				PriorityMessage* msg = sentPriorityMessages[i];
				if (msg->seq == ack) {
					// message confirmed, can delete.
					sentPriorityMessages.erase(sentPriorityMessages.begin() + i);
					free(msg);
					continue;
				}
				// check if we received an ack ahead of the message's sequence number
				// (actually checks for if it's up to 50 ahead because seq numbers wrap around)
				int16_t diff = (int16_t) ack - (int16_t) msg->seq;
				if ((diff > 0 && diff < 50) || diff < (50 - 255)) {
					// queue the message for resending.
					sentPriorityMessages.erase(sentPriorityMessages.begin() + i);
					priorityMessages.push_back(msg);
					Serial.println("retransmitting unacknowledged priority message");
				}
			}
		}
	}
	if (lastAckMillis < lastMsgMillis && millis() - lastAckMillis > ACK_TIMEOUT) {
		Serial.printf("No acks for %d millis, resetting telemetry\n", millis() - lastAckMillis);
		reset_telem();
	}
}

// do the actual sending.
// returns the sequence number.
uint8_t send_telem_packet(uint8_t id, uint16_t length, const void* data) {
	++telem_seq;
	// skip 0
	if (telem_seq == 255) telem_seq = 1;
	telem_serial->write(id);
	telem_serial->write(telem_seq);
	telem_serial->write((uint8_t *)&length, sizeof(length));
	telem_serial->write((uint8_t *)data, length);

	lastMsgMillis = millis();
	return telem_seq;
}

bool canSendMessage(int length) {
	int curTime = millis();
	if ((int32_t) curTime < pauseEndMillis) return false;
	if (length > telem_serial->availableForWrite()) return false;

	// Pause for 100 ms every 50 ms to allow acks to get through the half-duplex link
	if (curTime - pauseEndMillis > 50) {
		pauseEndMillis = curTime + 100;
		return false;
	}

	return true;
}

void sendQueuedMessages() {
	while (priorityMessages.size() > 0
	&& canSendMessage(priorityMessages.back()->length)) {

		PriorityMessage* msg = priorityMessages.back();
		priorityMessages.pop_back();
		msg->seq = send_telem_packet(msg->id, msg->length, &msg->contents);
		Serial.printf("sent prio msg of seq=%d\n", msg->seq);

		sentPriorityMessages.push_back(msg);
	}
	/*Serial.print("queue state:");
	for (MessageNode* msg = queueHead; msg != nullptr; msg = msg->newer) {
		Serial.printf(" %d", msg->id);
	}
	Serial.println();*/
	while (queueHead != nullptr) {
		MessageNode* msg = queueHead;
		if (!canSendMessage(msg->length)) {
			//Serial.printf("not sending %d\n", msg->id);
			return;
		}

		send_telem_packet(msg->id, msg->length, msg->contents);
		// Remove the node from the list
		//Serial.printf("sent %d\n", msg->id);
		if (msg == queueTail) {
			//Serial.println("empty queue");
			queueTail = nullptr;
		}
		queueHead = msg->newer;
		if (queueHead) queueHead->older = nullptr;
		msg->older = nullptr; msg->newer = nullptr;
	}
}

void dispatch_telem_packet(uint8_t id, uint16_t length, const void* data) {
	//Serial.printf("queue %d\n", id);
	// Every time we enter telemetry code, check to make sure we're keeping up
	checkAcks();

	// todo: have a way to make priority messages other than checking the type here
	if (id == telem_id_special_strmessage) {
		// add to the priority messages
		// must use malloc instead of new here because of the variable-length member
		PriorityMessage* msg = (PriorityMessage*) malloc(sizeof(PriorityMessage) + length);
		msg->id = id;
		msg->length = length;
		memcpy(&msg->contents, data, length);
		priorityMessages.push_back(msg);
	}
	else {

		// enqueue packet, replacing any previously queued packet of this message type
		MessageNode* node = getQueuedMessage(id);

		// if the node is not already in the queue
		if (node->newer == nullptr && node->older == nullptr) {
			// add at the tail
			node->older = queueTail;
			node->newer = nullptr;
			if (queueTail) queueTail->newer = node;
			if (queueHead == nullptr) queueHead = node;
			queueTail = node;
		}

		// Since messages are always the same length, allocate the contents only once
		if (node->contents == nullptr) node->contents = malloc(length);
		// make sure id and length are set
		node->length = length; node->id = id;
		// set data
		memcpy(node->contents, data, length);
	}
	// send this packet and maybe others
	sendQueuedMessages();
}

void telem_strmessage(const char* string) {
	telem_save_stream->print("strmessage: ");
	telem_save_stream->println(string);
	dispatch_telem_packet(telem_id_special_strmessage, (uint16_t) strlen(string), (const void*) string);
}
