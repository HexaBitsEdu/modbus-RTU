#include "modbus.h"

ModbusSlaveRTU::ModbusSlaveRTU(uint8_t addr, uint16_t regQuantity) {
	this->rcvBuffer = new uint8_t[255];
	this->deviceAdress = addr;
	memset(&(this->packet), 0x00, sizeof(ModbusPacket));
	this->g_userError = MBUS_RESPONSE_OK;
}
ModbusSlaveRTU::~ModbusSlaveRTU() {

}

mbus_status_t ModbusSlaveRTU::flashBuffer(void) {
	this->packet.errorCheck = 0xFFFF;
	this->packet.state = MBUS_STATE_IDLE;
	return MBUS_OK;
}

mbus_status_t ModbusSlaveRTU::pullPacket(uint8_t recieveByte) {
	switch (this->packet.state) {
	case MBUS_STATE_IDLE:
		this->flashBuffer();
		this->packet.state = MBUS_STATE_FUNCTION;
		this->packet.header.devAddress = recieveByte;
		break;
	case MBUS_STATE_FUNCTION:
		this->packet.header.functionCode = recieveByte;
		switch (recieveByte) {
		case MBUS_FUNC_WRITE_REG:
			this->packet.header.rnum = 1;
			this->packet.header.num = 1;
			this->packet.state = MBUS_STATE_REGADDR_HI;
			break;
		case MBUS_FUNC_WRITE_REGS:
		case MBUS_FUNC_WRITE_COILS:
			this->packet.header.rnum = 1;
			this->packet.header.num = 0;
			this->packet.state = MBUS_STATE_REGADDR_HI;
			break;
		case MBUS_FUNC_READ_INPUT_REGS:
		case MBUS_FUNC_READ_COILS:
		case MBUS_FUNC_READ_REGS:
			this->packet.state = MBUS_STATE_REGADDR_HI;
			this->packet.header.rnum = 0;
			this->packet.header.num = 0;
			break;
		case MBUS_FUNC_WRITE_COIL:
			this->packet.header.rnum = 1;
			this->packet.header.num = 1;
			this->packet.state = MBUS_STATE_REGADDR_HI;
			break;
		default:
			this->flashBuffer();
			// this->packet.state = MBUS_STATE_IDLE;
			break;
		}
		break;
	case MBUS_STATE_REGADDR_HI:
		this->packet.state = MBUS_STATE_REGADDR_LO;
		this->packet.header.registerAddr = recieveByte << 8;
		break;
	case MBUS_STATE_REGADDR_LO:
		this->packet.header.registerAddr |= recieveByte;
		if (this->packet.header.num == 1 && this->packet.header.rnum == 1) {
			this->packet.state = MBUS_STATE_DATA_HI;
		} else {
			this->packet.state = MBUS_STATE_REGNUM_HI;
		}
		break;
	case MBUS_STATE_DATA_HI:
		this->packet.state = MBUS_STATE_DATA_LO;
		this->rcvBuffer[2 * (this->packet.header.num - this->packet.header.rnum)
				+ 1] = recieveByte;
		break;
	case MBUS_STATE_DATA_LO:
		this->rcvBuffer[2 * (this->packet.header.num - this->packet.header.rnum)] =
				recieveByte;
		this->packet.header.rnum--;
		if (this->packet.header.rnum == 0) {
			this->packet.state = MBUS_STATE_CRC_LO;
		} else {
			this->packet.state = MBUS_STATE_DATA_HI;
		}
		break;
	case MBUS_STATE_DATA_SIZE:
		this->packet.state = MBUS_STATE_DATA;
		this->packet.header.size = recieveByte;
		this->packet.header.rsize = recieveByte;
		break;
	case MBUS_STATE_DATA:
		this->rcvBuffer[this->packet.header.size - this->packet.header.rsize] =
				recieveByte;
		this->packet.header.rsize--;
		if (this->packet.header.rsize == 0) {
			this->packet.state = MBUS_STATE_CRC_LO;
		}
		break;
	case MBUS_STATE_REGNUM_HI:
		this->packet.state = MBUS_STATE_REGNUM_LO;
		this->packet.header.num = recieveByte << 8;
		break;
	case MBUS_STATE_REGNUM_LO:
		this->packet.state = MBUS_STATE_CRC_LO;
		this->packet.header.num |= recieveByte;
		if (this->packet.header.rnum == 0) {
			this->packet.state = MBUS_STATE_CRC_LO;
		} else {
			this->packet.header.rnum = this->packet.header.num;
			this->packet.state = MBUS_STATE_DATA_HI;
			if (this->packet.header.functionCode == MBUS_FUNC_WRITE_REGS) {
				this->packet.state = MBUS_STATE_DATA_SIZE;
			}
		}
		break;
	case MBUS_STATE_CRC_LO:
		this->packet.state = MBUS_STATE_CRC_HI;
		break;
	case MBUS_STATE_CRC_HI:
		this->packet.state = MBUS_STATE_FINISH;
		break;
		// We can't processing any more before callback not returned
	case MBUS_STATE_RESPONSE:
		return MBUS_ERROR;
	default:
		this->flashBuffer();
		break;
	}

	this->packet.errorCheck = this->crc16(this->packet.errorCheck, recieveByte);

	if (this->packet.state == MBUS_STATE_FINISH) {
		// CRC error
		if (this->packet.errorCheck != 0) {
			this->flashBuffer();
			return MBUS_ERROR;
		}

		// TODO: Add broadcast messages
		if (this->packet.header.devAddress == this->deviceAdress) {
			this->packet.state = MBUS_STATE_RESPONSE;
			if (this->pullResponse() == MBUS_OK) {
				this->flashBuffer();
				return MBUS_OK;
			}
			//ctx->conf.send(mb_context, (uint8_t*)&(ctx->header), sizeof(ctx->header));
			this->flashBuffer();
			return MBUS_ERROR;
		}
		this->flashBuffer();
	}
	return MBUS_OK;
}

mbus_status_t ModbusSlaveRTU::pullResponse(void) {
	int read = 1;
	uint16_t la;
	if (this->packet.header.functionCode == 0x04)
		la = 10;

	switch (this->packet.header.functionCode) {
	case MBUS_FUNC_WRITE_COILS:
	case MBUS_FUNC_READ_COILS:
		if ((this->packet.header.num == 0)
				|| (this->packet.header.num > 0x07D0)) {
			return mbus_response(MBUS_RESPONSE_ILLEGAL_DATA_VALUE);
		}
		if ((this->packet.header.registerAddr + this->packet.header.num)
				>= this->coilsQuntity) {
			return mbus_response(MBUS_RESPONSE_ILLEGAL_DATA_ADDRESS);
		}
		break;
	case MBUS_FUNC_READ_DISCRETE:
		if ((this->packet.header.num == 0)
				|| (this->packet.header.num > 0x07D0)) {
			return mbus_response(MBUS_RESPONSE_ILLEGAL_DATA_VALUE);
		}
		if ((this->packet.header.registerAddr + this->packet.header.num)
				> this->discreteQuntity) {
			return mbus_response(MBUS_RESPONSE_ILLEGAL_DATA_ADDRESS);
		}
		break;
	default:
		break;
	}

	la = mbus_proto_address(
			(Modbus_ConnectFuncType) this->packet.header.functionCode, &read);
	if (la <= 50000 && this->modbusCallbacks != nullptr) {
		mbus_status_t sendState = MBUS_ERROR;
		if (read) {
			this->g_userError = MBUS_RESPONSE_OK;
			uint16_t sendBufferLen = ((this->packet.header.num) * 2) + 3;
			uint8_t *sendBuffer = new uint8_t[sendBufferLen + CRC_LEN];
			sendBuffer[2] = this->packet.header.num * 2;
			for (int i = 0; i < this->packet.header.num; i++) {
				uint16_t cellValue = this->modbusCallbacks->OnReadCell(la + i);
				sendBuffer[3 + (2 * i)] = (uint8_t) (cellValue >> 8);
				sendBuffer[3 + ((2 * i) + 1)] = cellValue & 0xFF;
			}
			if (g_userError == MBUS_RESPONSE_OK) {
				sendState = mbus_send_data(sendBuffer, sendBufferLen);
			} else {
				sendState = mbus_response(g_userError);
			}
			delete[] sendBuffer;
			return sendState;
		} else {
			uint16_t *value;
			uint8_t *sendBuffer = new uint8_t[6 + CRC_LEN];
			switch (this->packet.header.functionCode) {
			case MBUS_FUNC_WRITE_REG:
				value = (uint16_t*) this->rcvBuffer;
				la = this->packet.header.registerAddr;
				this->modbusCallbacks->OnWriteCell(la, *value);
				sendBuffer[2] = la >> 8;
				sendBuffer[3] = la & 0xFF;
				sendBuffer[4] = this->rcvBuffer[1];
				sendBuffer[5] = this->rcvBuffer[0];
				sendState = mbus_send_data(sendBuffer, 6);
				break;
			case MBUS_FUNC_WRITE_COIL:
				// in both these cases, we should return the same packet that we
				// received. in both cases, the packet have 6 bytes of data + 2 CRC
				// bytes = 8 bytes
				break;
			case MBUS_FUNC_WRITE_REGS:
				value = (uint16_t*) this->rcvBuffer;
				for (int i = 0; i < this->packet.header.num; i++) {
					this->modbusCallbacks->OnWriteCell(la + i, *value);
				}
				sendBuffer[2] = this->packet.header.registerAddr >> 8;
				sendBuffer[3] = this->packet.header.registerAddr & 0xFF;
				sendBuffer[4] = this->packet.header.num >> 8;
				sendBuffer[5] = this->packet.header.num & 0xFF;
				sendState = mbus_send_data(sendBuffer, 6);
				break;
			}
			delete[] sendBuffer;
			return sendState;
		}
	}
	return mbus_response(MBUS_RESPONSE_ILLEGAL_FUNCTION);

	return MBUS_ERROR;
}

mbus_status_t ModbusSlaveRTU::mbus_send_data(uint8_t *pSendBuffer,
		uint16_t len) {
	// if size > ( conf.send_sz-2) error
	pSendBuffer[0] = this->packet.header.devAddress;
	pSendBuffer[1] = this->packet.header.functionCode;
	uint16_t crc32 = 0xFFFF;
	if (this->modbusCallbacks == 0 || pSendBuffer == 0)
		return MBUS_ERROR;
	for (int i = 0; i < len; i++) {
		crc32 = this->crc16(crc32, pSendBuffer[i]);
	}
	pSendBuffer[len] = crc32 & 0xFF;
	pSendBuffer[len + 1] = (crc32 >> 8);

	if (this->modbusCallbacks->sendPacket(pSendBuffer, len + CRC_LEN) == true)
		return MBUS_ERROR;
	return MBUS_OK;
}

mbus_status_t ModbusSlaveRTU::mbus_response(Modbus_ResponseType response) {
	if (response != MBUS_RESPONSE_OK) {
		return mbus_send_error(response);
	} else {
	}
	return MBUS_ERROR;
}

int ModbusSlaveRTU::mbus_proto_address(Modbus_ConnectFuncType func, int *r) {
	int adr = 0;
	*r = 1;
	switch (func) {
	case MBUS_FUNC_WRITE_REG:
	case MBUS_FUNC_WRITE_REGS:
	case MBUS_FUNC_READ_WRITE_MASK_REGS:
	case MBUS_FUNC_READ_WRITE_REGS:
		*r = 0;
	case MBUS_FUNC_READ_REGS:
		adr = 40000;
		break;
	case MBUS_FUNC_READ_INPUT_REGS:
		adr = 30000;
		break;
	case MBUS_FUNC_READ_DISCRETE:
		adr = 10000;
		break;
	case MBUS_FUNC_WRITE_COILS:
	case MBUS_FUNC_WRITE_COIL:
		*r = 0;
	case MBUS_FUNC_READ_COILS:
		adr = 0000;
		break;
	default:
		break;
	}
	return adr;
}

mbus_status_t ModbusSlaveRTU::mbus_send_error(Modbus_ResponseType response) {
	uint8_t *pSendBuffer = new uint8_t[4 + CRC_LEN];
	*pSendBuffer = 0x8300 | (uint8_t) response;
	mbus_status_t state = mbus_send_data(pSendBuffer, 4);
	delete[] pSendBuffer;
	return state;
}
