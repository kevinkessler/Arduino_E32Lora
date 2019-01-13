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

class E32Lora {
public:
  E32Lora(Serial &uart, uint8_t m0, uint8_t m1, uint8_t aux);

private:
  uint8_t m0;
  uint8_t m1;
  uint8_t aux;
  Serial uart;

}
 #endif /* EPD1IN54_H */
