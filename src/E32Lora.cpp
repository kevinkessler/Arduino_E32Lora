/**
 *  @filename   :   E32Lora.c
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
#include <Arduino.h>
#include "E32Lora.h"

static E32Lora *instance=NULL;
void isr(void)
{
  instance->interruptHandler();
}

E32_STATUS E32Lora::begin(uint8_t m0, uint8_t m1, uint8_t aux) {
  _dataAvailable = 0;

  _m0 = m0;
  _m1 = m1;
  _aux = aux;

  pinMode(_m0,OUTPUT);
  pinMode(_m1,OUTPUT);
  pinMode(_aux,INPUT);

  if(instance == NULL) {
    instance = this;
    attachInterrupt(digitalPinToInterrupt(_aux), isr, FALLING);
  }

  _uart.begin(9600);

  uint8_t dummy[6];
  getConfig(dummy);

  setMode(NORMAL_MODE);
  // SetMode set the baud rate before it knows what it is for the first time
  // This fixes it
  _uart.begin(getBaud());


  return E32_OK;
}



void E32Lora::interruptHandler(){
  if (!_disableAuxIrq)
    _dataAvailable = 1;
  else
    _dataAvailable = 0;
}

uint8_t E32Lora::getMode()
{
  return (digitalRead(_m1) << 1) | (digitalRead(_m0));
}

E32_STATUS E32Lora::uartRead(uint8_t *buffer, uint16_t length, uint16_t timeout){
    uint32_t start = millis();
    if (start > start+timeout) //millis have wrapped
      return E32_TIMEOUT;

    uint16_t idx = 0;
    while(millis() < start + timeout) {
      if(_uart.available()) {
        buffer[idx++]=_uart.read();
        if (idx == length)
          return E32_OK;
      }
    }

    return E32_TIMEOUT;
}

E32_STATUS E32Lora::waitForAux(uint8_t state) {
  uint16_t count = 0;
	while(digitalRead(_aux) != state)
	{
		if (count++ > 2500)
			return E32_TIMEOUT;

		delay(1);
	}

  _dataAvailable = 0;
  delay(2);
	return E32_OK;
}

E32_STATUS E32Lora::setMode(uint8_t mode)
{

  uint8_t prevMode = getMode();
	if (mode == prevMode)
			return E32_OK;


  _disableAuxIrq = 1;
	if (waitForAux(1) != E32_OK) {
		return E32_ERROR;
  }

	digitalWrite(_m0, (mode & 1));
	digitalWrite(_m1, (mode & 2));

  // Got to delay to catch the falling edge and then wait for rise again
  delay(2);

	if (waitForAux(1) != E32_OK) {
		return E32_ERROR;
  }

  if (mode == SLEEP_MODE)
  		_uart.begin(9600);
  	else
  		_uart.begin(getBaud());

  //Wake up needs a 200ms delay before things start to work
  if(prevMode == SLEEP_MODE) {
    delay(250);
    _disableAuxIrq=0;
  }
  else if(mode==CONFIG_MODE) {
    uint8_t message[]={0xc1, 0xc1, 0xc1 };
    E32_STATUS error = configRequest(message, 3, _currentConfig, 6);
    if(error != E32_OK)
      return error;
    _disableAuxIrq = 1;
  }
  else {
    delay(50);
    _disableAuxIrq = 0;
  }

	return E32_OK;
}

uint32_t E32Lora::getBaud() {
	return _baudRateList[(_currentConfig[3] & 0x38) >> 3];
}

E32_STATUS E32Lora::configResponse(uint8_t *response, uint8_t responseLength)
{
	E32_STATUS error;
	if ((error = waitForAux(0)) != E32_OK)
		return error;

	E32_STATUS status = uartRead(response, responseLength, 2000);

	return status;
}

E32_STATUS E32Lora::configRequest(uint8_t *request, uint8_t requestLength,
		uint8_t *response, uint8_t responseLength)
{
	E32_STATUS error;

	if (_uart.write(request, requestLength) != requestLength)
    return E32_ERROR;

	if(response != NULL)
		if ((error = configResponse(response, responseLength)) != E32_OK)
			return error;

	return E32_OK;
}

E32_STATUS E32Lora::setConfig(uint8_t *configBuffer)
{
  E32_STATUS error;

  uint8_t origMode = getMode();
  if ((error = setMode(SLEEP_MODE)) != E32_OK)
    return error;

  if((error=configRequest(configBuffer,6,NULL,0))!=E32_OK) {
    return error;
  }

  if ((error = setMode(origMode)) != E32_OK)
    return error;

  return error;
}

E32_STATUS E32Lora::getConfig(uint8_t *configBuffer)
{
	uint8_t message[]={0xc1, 0xc1, 0xc1 };
  E32_STATUS error;

  uint8_t origMode = getMode();
  if ((error = setMode(SLEEP_MODE)) != E32_OK)
    return error;

  error = configRequest(message, 3, configBuffer, 6);
  memcpy(_currentConfig,configBuffer,6);
  if ((error = setMode(origMode)) != E32_OK)
  	return error;

  return E32_OK;

}

E32_STATUS E32Lora::getVersion(uint8_t *versionBuffer)
{
  uint8_t message[]={0xc3, 0xc3, 0xc3 };
	E32_STATUS error;

	uint8_t origMode = getMode();
	if ((error = setMode(SLEEP_MODE)) != E32_OK)
		return error;

	error = configRequest(message, 3, versionBuffer, 6);

	if ((error = setMode(origMode)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32Lora::reset()
{
  uint8_t message[] = {0xc4, 0xc4, 0xc4};
	E32_STATUS error;

	if ((error = setMode(SLEEP_MODE)) != E32_OK)
		return error;

	if((error = configRequest(message,3,NULL,0)) != E32_OK)
		return error;

	if ((error=waitForAux(0)) != E32_OK)
		return error;

	if ((error=waitForAux(1)) != E32_OK)
		return error;

  if ((error=waitForAux(0)) != E32_OK)
		return error;

  //Hack to get around a little blip in the aux line
  delay(10);

  if ((error=waitForAux(1)) != E32_OK)
		return error;
    
	if((error = setMode(NORMAL_MODE)) != E32_OK)
		return error;

	return E32_OK;
}

// Save parameters in flash, survives restart
E32_STATUS E32Lora::saveParams()
{
  E32_STATUS error;

	if (getMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[0] = '\xc0';

	if((error=configRequest(_currentConfig,6,NULL,0))!=E32_OK)
		return error;

	return E32_OK;
}

// Set parameters in memory, does not survive a restart
E32_STATUS E32Lora::setParams()
{
  E32_STATUS error;

	if (getMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[0] = '\xc2';

	if((error=configRequest(_currentConfig,6,NULL,0))!=E32_OK)
		return error;

	return E32_OK;
}

uint8_t E32Lora::dataAvailable(){
  return _dataAvailable;
}

E32_STATUS E32Lora::setAddress(uint16_t addr) {
  if (getMode() != SLEEP_MODE)
    return E32_INVALID_MODE;

    _currentConfig[1] = (addr & 0xff00) >> 8;
    _currentConfig[2] = addr & 0xff;

	return E32_OK;
}

E32_STATUS E32Lora::setParity(enum uartParity parity) {
  	if (getMode() != SLEEP_MODE)
  		return E32_INVALID_MODE;

  	_currentConfig[3] = (_currentConfig[3] & 0x3f) | parity << 6;

  	return E32_OK;
}

E32_STATUS E32Lora::setUartBaud(enum uartBaud baud)
{
  if (getMode() != SLEEP_MODE) {
		_uart.begin(_baudRateList[baud]);
		return E32_OK;
	}

	_currentConfig[3] = (_currentConfig[3] & 0xc7) | baud << 3;

	return E32_OK;
}

E32_STATUS E32Lora::setAirRate(enum airRate rate)
{
  if (getMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[3] = (_currentConfig[3] & 0xF8) | rate;

	return E32_OK;
}

E32_STATUS E32Lora::setChannel(uint8_t channel)
{
  if (getMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[4] = channel & 0x1F;

	return E32_OK;
}

E32_STATUS E32Lora::setTransmissionMode(enum txMode mode)
{
  if (getMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[5] = (_currentConfig[5] & 0x7F) | mode << 7;

	return E32_OK;
}

E32_STATUS E32Lora::setIOMode(enum ioMode mode)
{
  if (getMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[5] = (_currentConfig[5] & 0xBF) | mode << 6;

	return E32_OK;
}

E32_STATUS E32Lora::setWakeTime(enum wakeupTime wake) {
  if (getMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[5] = (_currentConfig[5] & 0xC7) | wake << 3;

	return E32_OK;
}

E32_STATUS E32Lora::setFECSwitch(enum fecSwitch fec) {
  if (getMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[5] = (_currentConfig[5] & 0xFB) | fec << 2;

	return E32_OK;
}

E32_STATUS E32Lora::setTXPower(enum txPower power) {
  if (getMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[5] = (_currentConfig[5] & 0xFC) | power;

	return E32_OK;
}

E32_STATUS E32Lora::setTargetChannel(uint8_t channel) {
	_targetChannel = channel & 0x1F;

	return E32_OK;
}

E32_STATUS E32Lora::setTargetAddress(uint8_t addr) {
	_targetAddress = addr;

	return E32_OK;
}

E32_STATUS E32Lora::transmit(uint8_t *message, uint16_t length) {
  if (getMode() == SLEEP_MODE)
    return E32_INVALID_MODE;

	if (length > 512)
		return E32_MESSAGE_TOO_LONG;

	if (_currentConfig[5] & 0x80) {
		if(length > 509)
			return E32_MESSAGE_TOO_LONG;
		uint8_t header[3 + length];
		header[0] = (_targetAddress & 0xFF) >> 8;
		header[1] = _targetAddress &0xff;
		header[2] = _targetChannel;
		memcpy(&header[3],message,length);
		if(_uart.write(header, length+3) != length + 3)
			return E32_ERROR;
	}
	else
		if(_uart.write(message, length) != length)
			return E32_ERROR;

	return E32_OK;
}

int16_t E32Lora::receiveData(uint8_t *buffer, uint16_t bufferLength) {
  int16_t idx = -1;
  uint16_t firstByte = 0;

  if (getMode() == SLEEP_MODE)
    return E32_INVALID_MODE;

  _dataAvailable = 0;
  for (uint16_t n = 0;n<1000;n++) {

    if((firstByte == 1) && (!_uart.available()))
      break;

    while(_uart.available()) {
      if(firstByte == 0)
        firstByte = 1;

      buffer[++idx] = _uart.read();

      if (idx == bufferLength)
        break;
    }

    if (idx == bufferLength)
      break;

    delay(1);
  }

  return idx;
}
