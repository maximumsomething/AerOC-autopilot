#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <unistd.h>
#include <chrono>
#include <iomanip>

// defined in autogen.cpp
void recievePacket(uint8_t id, uint16_t length, void* data);
int expectedLength(uint8_t id);
constexpr uint8_t telem_id_special_strmessage = 0xFF;


bool readPacket(FILE* serialIn) {
	size_t partsRead;
	struct {
		uint8_t id;
		uint8_t seq;
		uint16_t packetLength;
	} header;
	partsRead = fread(&header, sizeof(header), 1, serialIn);
	if (partsRead < 1) {
		std::cerr << "Could not read header" << std::endl;
		return false;
	}

	if ((header.id != telem_id_special_strmessage)
		&& header.packetLength != expectedLength(header.id)) {
		std::cerr << "Error: packet length mismatch: header for " << (int) header.id << " contains "
		<< header.packetLength << "; expected " << expectedLength(header.id) << std::endl;;
		return false;
	}
	char buf[header.packetLength];
	partsRead = fread(buf, header.packetLength, 1, serialIn);
	if (partsRead < 1) {
		std::cerr << "Could not read packet" << std::endl;
		return false;
	}

	// print timestamp
	/*time_t now = time(nullptr);
	struct tm* local = localtime(&now);
	l*/
	auto now = std::chrono::system_clock::now();
	auto time = std::chrono::system_clock::to_time_t(now);
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) -
			std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());

	std::cout << std::put_time(std::localtime(&time), "%H:%M:%S.");
	std::cout << ms.count() << " ";

	if (header.id == telem_id_special_strmessage) {
		std::string msg(buf, header.packetLength);
		// todo: escape string
		std::cout << "strmessage: " << msg << std::endl;
	}
	else recievePacket(header.id, header.packetLength, buf);

	// send acknowledgment
	fwrite(&header.seq, 1, 1, serialIn);
	return true;
}

FILE* openSerial(const char* filename) {

	// todo: this the proper way, with termios and cfmakeraw
	FILE* serialIn = fopen(filename, "r+");
	if (serialIn == nullptr) {
		perror("Could not open serialIn file");
		exit(1);
	}
	return serialIn;
}

int main(int argc, char** argv) {

	if (argc != 2) {
		std::cerr << "usage: telemetry_ground serialIn" << std::endl;
		return 1;
	}
	const char* filename = argv[1];

	system(("stty 9600 -F '" + std::string(filename) + "' raw").c_str());
	
	FILE* serialIn = openSerial(filename);

	while (true) {
		if (readPacket(serialIn)) {
			//continue;
		}
		else {
			// determine if there's an actual I/O error
			if (ferror(serialIn)) {
				perror("Error reading serial port");
			}
			else if (feof(serialIn)) {
				std::cerr << "Reached end-of-file" << std::endl;
				return 0;
			}

			// Send NACK (negative-acknowledgment) over the serial port
			char msg = 0;
			fwrite(&msg, 1, 1, serialIn);
			// This will cause the other side to wait 100 ms before sending anything else.


			usleep(40000);
			// close and reopen the file
			fclose(serialIn);
			//std::cerr << "file closed" << std::endl;
			serialIn = openSerial(filename);
		}
	}
}


