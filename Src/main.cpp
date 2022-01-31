#include "main.h"
#include "modbus.h"
#include "usart.h"
#include <stdint.h>
#include <stdlib.h>

#define RS485ReciveEn()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define RS485TranmitEn()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)

uint8_t rcvData;
ModbusSlaveRTU myModbus(0x01, 50);

uint16_t value1,value2,value3;

class myModbusCallback: public ModbusCallbacks {
	uint16_t OnReadCell(const uint32_t la) {
		if (la == 40000) {
			return SparklerState();
		} else if (la == 40001) {
			return 0x0001;
		} else if (la == 40002) {
			return 0x0002;
		} else if (la == 40003) {
			return 0x0003;
		} else if (la == 40004) {
			return 0x0004;
		} else if (la == 40005) {
			return 0x0005;
		}
		return 0;
	}
	uint16_t OnWriteCell(const uint32_t la, const uint16_t value) {
		if (la == 40000) {
			return 0;
		} else if (la == 40001) {
			value1 = value;
		} else if (la == 40002) {
			value2 = value;
		} else if (la == 40003) {
			value3 = value;
		}
		return 1;
	}
	bool sendPacket(uint8_t *packet, uint16_t len) {
		RS485TranmitEn();
		bool state = HAL_UART_Transmit(&huart1, packet, len, 1000);
		RS485ReciveEn();
		return state;
	}
};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		myModbus.pullPacket(rcvData);
		HAL_UART_Receive_IT(&huart1, &rcvData, 1);
	}
}

void main(void) {
	MX_USART1_UART_Init();
	myModbus.setAdress(0x01);
	myModbus.setModbusCallbacks(new myModbusCallback());
	RS485ReciveEn();
	__enable_irq();
	while (1) {
	}
}
