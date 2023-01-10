#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <cstring>
#include <thread>

// defined in autogen.cpp
void recievePacket(uint16_t id, uint16_t length, void* data);
int expectedLength(uint16_t id);
constexpr uint16_t telem_id_special_strmessage = 0xFFFF;


bool readPacket(FILE* serialIn) {
	size_t partsRead;
	struct {
		uint16_t id;
		uint16_t packetLength;
	} header;
	partsRead = fread(&header, sizeof(header), 1, serialIn);
	if (partsRead < 1) {
		std::cerr << "Could not read header" << std::endl;
		return false;
	}

	if ((header.id != telem_id_special_strmessage)
		&& header.packetLength != expectedLength(header.id)) {
		std::cerr << "Error: packet length mismatch: header for " << header.id << " contains "
		<< header.packetLength << "; expected " << expectedLength(header.id) << std::endl;;
		return false;
	}
	char buf[header.packetLength];
	partsRead = fread(buf, header.packetLength, 1, serialIn);
	if (partsRead < 1) {
		std::cerr << "Could not read packet" << std::endl;
		return false;
	}
	if (header.id == telem_id_special_strmessage) {
		std::string msg(buf, header.packetLength);
		// todo: escape string
		std::cout << "strmessage: " << msg << std::endl;
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


	recievePacket(header.id, header.packetLength, buf);
	return true;
}

FILE* openSerial(const char* filename) {

	// todo: this the proper way, with termios and cfmakeraw
	system(("stty 9600 -F '" + std::string(filename) + "' raw").c_str());
	FILE* serialIn = fopen(filename, "r");
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


	FILE* serialIn = openSerial(filename);

	/*char buf[1024];
	size_t bytesRead;
	while ((bytesRead = fread(buf, sizeof(buf), serialIn)) > 0) {

	}*/
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
			FILE* serialOut = fopen(filename, "w");
			if (serialOut == nullptr) {
				perror("Could not file for writing NACK");
			}
			char msg[] = "NACK";
			fwrite(msg, strlen(msg), 1, serialOut);

			fclose(serialOut);
			// This will cause the other side to wait 200 ms before sending anything else.


			// jank solution to keep the read buffer clear.
			/*std::thread([&] {
				char buf[100];
				// keep reading and discarding.
				while(fread(buf, 1, sizeof(buf), serialIn) == sizeof(buf));
				std::cerr << "reset complete" << std::endl;
			}).detach();*/

			// Wait for most of it.
			usleep(100000);
			// close and reopen the file
			close(fileno(serialIn)); // will cause above thread to end
			fclose(serialIn);
			//std::cerr << "file closed" << std::endl;
			serialIn = openSerial(filename);
		}
	}
}


