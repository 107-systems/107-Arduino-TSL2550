/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2022 LXRobotics.
 * Author: Bernhard Mayer <bernhard@generationmake.de>
 * Contributors: https://github.com/107-systems/107-Arduino-TSL2550/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "107-Arduino-TSL2550.h"

#include <cmath>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace TSL2550;

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ArduinoTSL2550::ArduinoTSL2550(TSL2550::I2cWriteFunc write,
                               TSL2550::I2cReadFunc read,
                               uint8_t const i2c_slave_addr)
: _error{TSL2550::Error::None}
, _io{write, read, i2c_slave_addr}
{
}


/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

bool ArduinoTSL2550::begin(bool const use_extended)
{
  /* Check the CHIP ID if it matches the expected value.
   */
  if (_io.read(TSL2550::Register::ReadCommandRegister) != TSL2550::ID_EXPECTED_ID) {
    _error = TSL2550::Error::ChipId;
    return false;
  }

  if (use_extended)
    _io.write(TSL2550::Register::WriteCommandExtendedRange, 0);
  else
    _io.write(TSL2550::Register::WriteCommandStandardRange, 0);

  return true;
}


float ArduinoTSL2550::get_lux()
{
  uint8_t adc_0 = _io.read(TSL2550::Register::ReadADCChannel0);
  uint8_t adc_1 = _io.read(TSL2550::Register::ReadADCChannel1);

  adc_0                 = (adc_0 & 0x7F);  /* remove valid bit */
  int const adc_0_chord = (adc_0 & 0xF0) >> 4;
  int const adc_0_step  = (adc_0 & 0x0F);
  int const adc_0_count = ((33*((1<<adc_0_chord)-1))>>1)+(adc_0_step*(1<<adc_0_chord));

  adc_1                 = (adc_1 & 0x7F); /* remove valid bit */
  int const adc_1_chord = (adc_1 & 0xF0) >>4;
  int const adc_1_step  = (adc_1 & 0x0F);
  int const adc_1_count = ((33*((1<<adc_1_chord)-1))>>1)+(adc_1_step*(1<<adc_1_chord));

  float light_level = 0.0f;

  if ((adc_0_count - adc_1_count) !=0 )
  {
    float const r= (float)adc_1_count/((float)(adc_0_count-adc_1_count));
    light_level = (float)(adc_0_count-adc_1_count)*5.0*0.39*exp(-0.181*r*r);
  }

  return light_level;
}

void ArduinoTSL2550::powerdown()
{
  _io.write(TSL2550::Register::PowerDownState, 0);
}

TSL2550::Error ArduinoTSL2550::error()
{
  return _error;
}
