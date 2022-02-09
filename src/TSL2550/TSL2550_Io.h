/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2022 LXRobotics.
 * Author: Bernhard Mayer <bernhard@generationmake.de>
 * Contributors: https://github.com/107-systems/107-Arduino-TSL2550/graphs/contributors.
 */

#ifndef ARDUINO_TSL2550_TSL2550_IO_H_
#define ARDUINO_TSL2550_TSL2550_IO_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#undef max
#undef min
#include <functional>

#include "TSL2550_Const.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TSL2550
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::function<void(uint8_t const, uint8_t const, uint8_t const *, uint8_t const)> I2cWriteFunc;
typedef std::function<void(uint8_t const, uint8_t const, uint8_t       *, uint8_t const)> I2cReadFunc;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TSL2550_Io
{
public:

  TSL2550_Io(I2cWriteFunc write, I2cReadFunc read, uint8_t const i2c_slave_addr);


  uint8_t read    (Register const reg);
  void    write   (Register const reg, uint8_t const val);
  void    read    (Register const reg, uint8_t * buf, size_t const bytes);
  void    write   (Register const reg, uint8_t const * buf, size_t const bytes);


private:

  I2cWriteFunc _write;
  I2cReadFunc _read;
  uint8_t const _i2c_slave_addr;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TSL2550 */

#endif /* ARDUINO_TSL2550_TSL2550_IO_H_ */
