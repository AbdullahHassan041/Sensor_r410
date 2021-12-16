/*******************************************************************************
* Copyright (C) 2017, 2017 Spiio, Inc - All Rights Reserved
* Unauthorized copying of this file, via any medium is strictly prohibited
* Proprietary and confidential
* Written by Jens-Ole Graulund <jensole@spiio.com>, December 2017

* Contributors:
*    Jens-Ole Graulund - initial API and implementation
*******************************************************************************/
#include "mbed.h"
#include "SpiioBoard.h"

#include "PwmOut_advanced.h"

#include "mbed_trace.h"
#define TRACE_GROUP "SPIIO"
DigitalOut en_sensor(EN_SENSOR_PIN, 0);
DigitalOut en_salinity_1(SALINITY_V_PIN1, 0);
DigitalOut en_salinity_2(SALINITY_V_PIN2, 0);

void SPIIO::SpiioBoard::sensorSalinityPower(salinityPwr_t pwr)
{
  if (pwr == s_positive)
  {
    //tr_debug("PIN39 GND, PIN40 1.8V");
    en_salinity_1.write(0);
    en_salinity_2.write(1);
  }
  else if (pwr == s_negative)
  {
    //tr_debug("PIN39 1.8V, PIN40 GND");
    en_salinity_1.write(1);
    en_salinity_2.write(0);
  }
  else
  {
    //tr_debug("PIN39 GND, PIN40 GND");
    en_salinity_1.write(0);
    en_salinity_2.write(0);
  }
}

void SPIIO::SpiioBoard::sensorPwrPin(bool power_pin)
{
  if (power_pin)
  {
    en_sensor.write(1);
  }
  else
  {
    en_sensor.write(0);
  }
}

void SPIIO::SpiioBoard::getMeasurement(datapacket information)
{
  gpio_t gpio; //Mearsure the battery need this line
  gpio_init_out_ex(&gpio, EN_3V6_UBLOX_PIN, 1);  //Mearsure the battery need this line
  wait_ms(50);
  float battery = _get_battery();
  gpio_init_out_ex(&gpio, EN_3V6_UBLOX_PIN, 0);  //Mearsure the battery need this line
  
  en_sensor.write(1);
  wait(1.0);
  
  // PwmOut mypwm(PWM_OUT);
  PwmOutAdvanced mypwm(PWM_OUT);
  mypwm.period_cycle(4);
  mypwm.write(0.5);
  
  wait(3.0);

  float moisture= _get_moisture();
  mypwm.write(0); 
  wait_ms(10);
  float temp = _get_temp();
  float light = _get_light(); 
  float salinity = _get_alternating_salinity();
  time_t currentTime = _get_time();
  float airp = 0; //_get_pressure();
  
  //sprintf(buffer, "[%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%li,%4.2f]", moisture, temp, light, battery, salinity, currentTime, airp);
 
  en_sensor.write(0);

  return;
};

float SPIIO::SpiioBoard::getBlessData(sensor_type sensor)
{
  float sen_read = 0;

  gpio_t gpio;
  //PwmOut mypwm(PWM_OUT);
  PwmOutAdvanced mypwm(PWM_OUT);
  mypwm.period_cycle(4);
  mypwm.write(0.5);
  en_sensor.write(1);
  gpio_init_out_ex(&gpio, EN_3V6_UBLOX_PIN, 1);
  wait(2.0);

  switch (sensor)
  {
  case s_battery:
    sen_read = _get_battery();
    break;
  case s_temperature:
    sen_read = _get_temp();
    break;
  case s_llight:
  case s_hlight:
    sen_read = _get_light();
    break;
  case s_airp:
    //sen_read = _get_pressure();
    break;
  case s_salinity_air:
  case s_salinity_water:
    sen_read = _get_alternating_salinity();
    break;
  case s_moisture_air:
  case s_moisture_water:
    sen_read = _get_moisture();
    break;
  default:
    break;
  }

  gpio_init_out_ex(&gpio, EN_3V6_UBLOX_PIN, 0);
  en_sensor.write(0);

  return sen_read;
};
