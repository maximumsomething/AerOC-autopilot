#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <string>

// defined in autogen.cpp
void recievePacket(uint16_t id, uint16_t length, void* data);
int expectedLength(uint16_t id);
constexpr int telem_id_special_strmessage = 0;

int main(int argc, char** argv) {

	if (argc != 2) {
		std::cerr << "usage: telemetry_ground serialIn" << std::endl;
		return 1;
	}

	const char* filename = argv[1];

	// todo: this the proper way, with termios and cfmakeraw
	system(("stty -F '" + std::string(filename) + "' raw").c_str());

	FILE* serialIn = fopen(filename, "r");
	if (serialIn == nullptr) {
		perror("Could not open serialIn file");
		return 1;
	}

	/*char buf[1024];
	size_t bytesRead;
	while ((bytesRead = fread(buf, sizeof(buf), serialIn)) > 0) {

	}*/
	while (true) {
		size_t partsRead;
		struct {
			uint16_t id;
			uint16_t packetLength;
		} header;
		partsRead = fread(&header, sizeof(header), 1, serialIn);
		if (partsRead < 1) {
			std::cerr << "Could not read header" << std::endl;
			break;
		}

		if ((header.id != telem_id_special_strmessage)
			&& header.packetLength != expectedLength(header.id)) {
			std::cerr << "Error: packet length mismatch: header for " << header.id << " contains "
			<< header.packetLength << "; expected " << expectedLength(header.id) << std::endl;;
			fseek(serialIn, header.packetLength, SEEK_CUR);
			continue;
		}
		char buf[header.packetLength];
		partsRead = fread(buf, header.packetLength, 1, serialIn);
		if (partsRead < 1) {
			std::cerr << "Could not read packet" << std::endl;
			break;
		}
		if (header.id == telem_id_special_strmessage) {
			std::string msg(buf, header.packetLength);
			// todo: escape string
			std::cout << "strmessage: " << msg << std::endl;
		}

		recievePacket(header.id, header.packetLength, buf);
	}



	if (ferror(serialIn)) {
		perror("Error reading serial port");
	}
	else if (feof(serialIn)) {
		std::cerr << "Reached end-of-file" << std::endl;
		return 0;
	}
	else {
		// should never be reached
		std::cerr << "mystery termination?" << std::endl;
	}
	return 1;
}
