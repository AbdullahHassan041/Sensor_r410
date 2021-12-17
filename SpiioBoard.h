/*******************************************************************************
* Copyright (C) 2017, 2017 Spiio, Inc - All Rights Reserved
* Unauthorized copying of this file, via any medium is strictly prohibited
* Proprietary and confidential
* Written by Jens-Ole Graulund <jensole@spiio.com>, December 2017

* Contributors:
*    Jens-Ole Graulund - initial API and implementation
*******************************************************************************/
#ifndef _SPIIOBOARD_H_
#define _SPIIOBOARD_H_
#include "mbed.h"
typedef struct
{
  uint64_t    airp                :32;
  time_t      currentTime         :64;
  uint64_t    salinity            :32;
  uint64_t    battery             :32;
  uint64_t    light               :32;
  uint64_t    temp                :32;
  uint64_t    moisture            :32;
} datapacket;

//int get_Measurement(datapacket *information);
typedef enum
{
  s_battery = 0,
  s_temperature,
  s_llight,
  s_hlight,
  s_airp,
  s_salinity_air,
  s_salinity_water,
  s_moisture_air,
  s_moisture_water,
} sensor_type;
typedef enum
{
  s_off = 0,
  s_positive,
  s_negative
} salinityPwr_t;

namespace SPIIO
{
class SpiioBoard
{
public:
  void sensorSalinityPower(salinityPwr_t pwr);
  void sensorPwrPin(bool power_pin);
  void getMeasurement(datapacket *information);
  float getBlessData(sensor_type sensor);

//private:
  float _get_moisture();
  float _get_light();
  float _get_battery();
  float _get_temp();
  float _get_pressure();
  float _get_salinity();
  float _get_salinity_positive();
  float _get_salinity_negative();
  float _get_alternating_salinity();
  time_t _get_time();
};
} // namespace SPIIO

#endif // _SPIIOBOARD_H_
