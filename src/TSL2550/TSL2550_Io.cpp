/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2022 LXRobotics.
 * Author: Bernhard Mayer <bernhard@generationmake.de>
 * Contributors: https://github.com/107-systems/107-Arduino-TSL2550/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "TSL2550_Io.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TSL2550
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

TSL2550_Io::TSL2550_Io(I2cWriteFunc write, I2cReadFunc read, uint8_t const i2c_slave_addr)
: _write{write}
, _read{read}
, _i2c_slave_addr{i2c_slave_addr}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void TSL2550_Io::write(Register const reg)
{
  uint8_t const val=0;
  write(reg, &val, 0);
}

uint8_t TSL2550_Io::read(Register const reg)
{
  uint8_t data = 0;
  read(reg, &data, 1);
  return data;
}

void TSL2550_Io::write(Register const reg, uint8_t const val)
{
  write(reg, &val, 1);
}

void TSL2550_Io::read(Register const reg, uint8_t * buf, size_t const bytes)
{
  _read(_i2c_slave_addr, to_integer(reg), buf, bytes);
}

void TSL2550_Io::write(Register const reg, uint8_t const * buf, size_t const bytes)
{
  _write(_i2c_slave_addr, to_integer(reg), buf, bytes);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TSL2550 */
