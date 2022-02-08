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

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace TSL2550;

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ArduinoTSL2550::ArduinoTSL2550(TSL2550::I2cWriteFunc write,
                               TSL2550::I2cReadFunc read,
 //                              TSL2550::DelayFunc delay,
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
  if (_io.read(TSL2550::Register::TSL2550_ReadCommandRegister) != TSL2550::ID_EXPECTED_ID) {
    _error = TSL2550::Error::ChipId;
    return false;
  }

  if(use_extended == true) _io.write(TSL2550::Register::TSL2550_WriteCommandExtendedRange);
  else _io.write(TSL2550::Register::TSL2550_WriteCommandStandardRange);

  return true;
}


float ArduinoTSL2550::get_lux()
{
// variables for TLS2550
  uint8_t adc_0=0,adc_1=0;
  int adc_0_chord=0, adc_0_step=0, adc_0_count=0;
  int adc_1_chord=0, adc_1_step=0, adc_1_count=0;
  float r=0, light_level=0;

  adc_0 = _io.read(TSL2550::Register::TSL2550_ReadADCChannel0);
  adc_1 = _io.read(TSL2550::Register::TSL2550_ReadADCChannel1);

  adc_0=adc_0&0x7f;  // remove valid bit
  adc_0_chord=(adc_0&0xf0)>>4;;
  adc_0_step=adc_0&0x0f;
  adc_0_count=((33*((1<<adc_0_chord)-1))>>1)+(adc_0_step*(1<<adc_0_chord));

  adc_1=adc_1&0x7f; // remove valid bit
  adc_1_chord=(adc_1&0xf0)>>4;;
  adc_1_step=adc_1&0x0f;
  adc_1_count=((33*((1<<adc_1_chord)-1))>>1)+(adc_1_step*(1<<adc_1_chord));

  if((adc_0_count-adc_1_count)!=0)
  {
    r=(float)adc_1_count/((float)(adc_0_count-adc_1_count));
    light_level=(float)(adc_0_count-adc_1_count)*5.0*0.39*exp(-0.181*r*r);
  }

  return light_level;
}

void ArduinoTSL2550::powerdown()
{
  _io.write(TSL2550::Register::TSL2550_PowerDownState);
}

TSL2550::Error ArduinoTSL2550::error()
{
  return _error;
}
