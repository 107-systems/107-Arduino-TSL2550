/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2022 LXRobotics.
 * Author: Bernhard Mayer <bernhard@generationmake.de>
 * Contributors: https://github.com/107-systems/107-Arduino-TSL2550/graphs/contributors.
 */

#ifndef _107_ARDUINO_TSL2550_H_
#define _107_ARDUINO_TSL2550_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "TSL2550/TSL2550_Io.h"
#include "Arduino.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ArduinoTSL2550
{

public:

  ArduinoTSL2550(TSL2550::I2cWriteFunc write,
                 TSL2550::I2cReadFunc read,
                 uint8_t const i2c_slave_addr);

  bool begin(bool const use_extended);
  float get_lux();
  void powerdown();

  TSL2550::Error error();

private:

  TSL2550::Error _error;
  TSL2550::TSL2550_Io _io;

};

#endif /* _107_ARDUINO_TSL2550_H_ */
