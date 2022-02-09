/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2022 LXRobotics.
 * Author: Bernhard Mayer <bernhard@generationmake.de>
 * Contributors: https://github.com/107-systems/107-Arduino-TSL2550/graphs/contributors.
 */

#ifndef ARDUINO_TSL2550_TSL2550_CONST_H_
#define ARDUINO_TSL2550_TSL2550_CONST_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>

#include <type_traits>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace TSL2550
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class Register : uint8_t
{
  PowerDownState            = 0x00,
  ReadCommandRegister       = 0x03,
  WriteCommandExtendedRange = 0x1D,
  WriteCommandStandardRange = 0x18,
  ReadADCChannel0           = 0x43,
  ReadADCChannel1           = 0x83,
};

enum class Error : int
{
  None    =  0,
  ChipId  = -1,
};

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t constexpr ID_EXPECTED_ID   = 0x03;
static uint8_t constexpr DEFAULT_I2C_ADDR = 0x39;

/**************************************************************************************
 * CONVERSION FUNCTIONS
 **************************************************************************************/

template <typename Enumeration>
constexpr auto to_integer(Enumeration const value) -> typename std::underlying_type<Enumeration>::type
{
  return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

template <typename Enumeration>
constexpr auto bp(Enumeration const value) -> typename std::underlying_type<Enumeration>::type
{
  return to_integer(value);
}

template <typename Enumeration>
constexpr auto bm(Enumeration const value) -> typename std::underlying_type<Enumeration>::type
{
  return (1 << to_integer(value));
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* TSL2550 */

#endif /* ARDUINO_TSL2550_TSL2550_CONST_H_ */
