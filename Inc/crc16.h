#ifndef INC_CRC16_H_
#define INC_CRC16_H_

#include <stdint.h>

class CRC16class {
public:
	uint16_t crc16(const uint16_t crc16, const uint8_t byte);
};

#endif /* INC_CRC16_H_ */
