/**
 *  @filename   :   E32Lora.h
 *  @brief      :   Interface for the EByte E32 Series Lora Module
 *
 *  @author     :   Kevin Kessler
 *
 * Copyright (C) 2019 Kevin Kessler
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
 #ifndef E32LORA_H
 #define E32LORA_H
#include <Arduino.h>

#define NORMAL_MODE 0
#define WAKEUP_MODE 2
#define POWERSAVING_MODE 1
#define SLEEP_MODE 3

#define E32_OK 0
#define E32_TIMEOUT 1
#define E32_ERROR 100
#define E32_MESSAGE_TOO_LONG 101

typedef uint8_t E32_STATUS;

enum uartBaud {
	Baud_1200 = 0,
	Baud_2400 = 1,
	Baud_4800 = 2,
	Baud_9600 = 3,
	Baud_19200 = 4,
	Baud_38400 = 5,
	Baud_57600 = 6,
	Baud_115200 = 7
};

enum uartParity {
	Parity_8N1 = 0,
	Parity_8O1 = 1,
	Parity_8E1 = 2
};

enum airRate {
	AirRate_0300 = 0,
	AirRate_1200 = 1,
	AirRate_2400 = 2,
	AirRate_4800 = 3,
	AirRate_9600 = 4,
	AirRate_19200 = 5
};

enum wakeupTime {
	Wakeup_250 = 0,
	Wakeup_500 = 1,
	Wakeup_750 = 2,
	Wakeup_1000 = 3,
	Wakeup_1250 = 4,
	Wakeup_1500 = 5,
	Wakeup_1750 = 6,
	Wakeup_2000 = 7
};

enum txPower {
	TxPwr_20 = 0,
	TxPwr_17 = 1,
	TxPwr_14 = 2,
	TxPwr_10 = 3
};

enum txMode {
	TxMode_Transparent = 0,
	TxMode_Fixed = 1
};

enum ioMode {
	IOMode_Open = 0,
	IOMode_Pull = 1
};

enum fecSwitch {
	FEC_Off = 0,
	FEC_On = 1
};

class E32Lora {
public:
  E32Lora(HardwareSerial &uart):_uart(uart){}
  E32_STATUS begin(uint8_t m0, uint8_t m1, uint8_t aux);
  E32_STATUS getConfig(uint8_t *configBuffer);
  E32_STATUS getVersion(uint8_t *configBuffer);
  E32_STATUS reset();
  E32_STATUS saveParams();
  E32_STATUS setAddress(uint16_t addr);
  E32_STATUS setParity(enum uartParity parity);
  E32_STATUS setUartBaud(enum uartBaud baud);
  E32_STATUS setAirRate(enum airRate rate);
  E32_STATUS setChannel(uint8_t channel);
  E32_STATUS setTransmissionMode(enum txMode mode);
  E32_STATUS setIOMode(enum ioMode mode);
  E32_STATUS setWakeTime(enum wakeupTime wake);
  E32_STATUS setFECSwitch(enum fecSwitch fec);
  E32_STATUS setTXPower(enum txPower power);
  E32_STATUS setTargetChannel(uint8_t channel);
  E32_STATUS setTargetAddress(uint8_t addr);
  E32_STATUS transmit(uint8_t *message, uint16_t length);
  uint8_t dataAvailable(void);
  int16_t receiveData(uint8_t *buffer, uint16_t bufferLegth);
  void interruptHandler(void);
  E32_STATUS setMode(uint8_t mode);

private:
  uint8_t _m0;
  uint8_t _m1;
  uint8_t _aux;
  uint8_t _currentConfig[6];
  uint8_t _targetChannel;
  uint16_t _targetAddress;
  volatile uint8_t _dataAvailable;
  uint8_t _disableAuxIrq = 0;
  uint32_t _baudRateList[8] = {1200,2400,4800,9600,19200,38400,57600,115200};
  HardwareSerial &_uart;
  //static E32Lora *instance;

  E32_STATUS getMode(void);
  E32_STATUS waitForAux(uint8_t state);
  E32_STATUS uartRead(uint8_t *buffer, uint16_t length, uint16_t timeout);
  uint32_t getBaud(void);
  E32_STATUS configResponse(uint8_t *response, uint8_t responseLength);
  E32_STATUS configRequest(uint8_t *request, uint8_t requestLength,
  		uint8_t *response, uint8_t responseLength);
  //static void isr(void);



};
#endif /* E32LORA_H */
