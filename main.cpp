/*******************************************************************************
* Copyright (C) 2017, Spiio Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Jens-Ole Graulund <jensole@spiio.com>, December 2017

 * Contributors:
 *    Jens-Ole Graulund - initial API and implementation
 *    Jesper Nielsen    - adaptations since 01.07.2020
 *******************************************************************************/
#include "SpiioCommon/Config.h"
#include "SpiioCommon/Device.h"
#include "SpiioCommon/SpiioUtils.h"
#include "SpiioCommon/StoreConfig.h"
#include "spiioBoard/ResetReason.h"
#include "spiioBoard/SpiioBoard.h"
#include "spiioBoard/SpiioConsole.h"
#include "spiioBoard/SpiioEeprom.h"
#include "spiioBoard/SpiioLed.h"
#include "spiioBoard/Watchdog.h"
#include "spiioBoard/low_power.h"
#include "spiioClient/SpiioClient.h"
#include "spiioNetwork/spiioNetwork.h"
#ifdef SPIIO_SENSOR_SPIFLASH_FT25LX04
#include "spiioBoard/SPIFBlockDevice.h"
#endif
#include "UbloxCellularHttp.h"

#include "mbed_trace.h"
#define TRACE_GROUP "SPIIO"

using namespace std;

#define BOOTLOADER_ADDRESS (unsigned long)0x8000000

// Time to suspend initialization process before rebooting sensor
#define SUSPEND_AND_BOOT_TIME       3600000   // 1 hour
#define MAGNET_SUSPEND_TIME         30        // in seconds
#define SLEEP_PERIOD                20000     // 20 secs
#define LED_RESET_INTERVAL_ONE      3600000   // 1 hour
#define LED_RESET_INTERVAL_TWO      79200000  // 22 hours

#define SUSPEND_AND_BOOT_TIME_DAY   86400000  // 1 day
#define SUSPEND_AND_BOOT_TIME_HOURS 21600     // 6 hours
#define SUSPEND_TIME_PERIOD         2
#define SCHED_ITEM_SIZE             2
#define NUMBER_OF_SCHEDULER         2

// Watchdog reset timer in unreachable part of code
#define WATCHDOG_RESET_TIMER        300000   // 5 mins

static int connection_timeout   = 180;  // in secs
static int suspend_count_days   = 7;
static int suspend_count_hours  = 6;

bool overrided_sleep_logic = false;

enum caseSleepInterval{
  SLEEP_CYCLE_ONE,
  SLEEP_CYCLE_TWO
};

// sleep schedule under disconnection scenerio
uint8_t sleep_schedule[SUSPEND_TIME_PERIOD][SCHED_ITEM_SIZE] = {
                                {6, 2},   // 6 hours with two retries
                                {24, 7}   // 24 hours with seven retries
                               };

void rtc_it_handle(void)
{
  //Empty
}

LowPower lowPower;
Ticker wdogTicker;
Watchdog watchdog = Watchdog();

// PWR Detect Pin
InterruptIn pwr_det(UBLOX_PWR_DET_PIN);
UbloxCellularHttp *interface;
SPIIO::SpiioLed LED;
SPIIO::SpiioEeprom e2prom;
bool shutdownRequired = false;

static bool suspendRequired = false;
static time_t suspendTime = 0;
static bool suspendInterrupted = 0;
static bool modemCanSuspend = false;
static uint8_t ledResetCounter = 1;
Queue<uint8_t, 1> queue;
void pwr_magnet_place(void);
void pwr_magnet_place_dummy(void);
void pwr_magnet_detach_dummy(void);

void suspend_and_boot(void)
{
  // sleep for SUSPEND_AND_BOOT_TIME ms and boot and retry with wathdog restart
  watchdog.start(28000);
  int interval = abs(SUSPEND_AND_BOOT_TIME / SLEEP_PERIOD);
  for (int count = 0; count < interval; count++)
  {
    watchdog.kick();
    lowPower.enterStop(SLEEP_PERIOD);
  }
  // Trigger watchdog restart - using watchdog restart to avoid reset state which would turn led's on.
  lowPower.enterStop(40000);
}

void watchdog_restart(void)
{
  watchdog.start(5000);
  // Trigger watchdog restart - using watchdog restart to avoid reset state which would turn led's on.
  lowPower.enterStop(10000);
}

void kick_handle(void)
{
  //tr_debug("Kick the dog !");
  watchdog.kick();
}

enum modemState{
  MDM_CANNOT_SUSPEND,
  MDM_CAN_SUSPEND,
  MDM_IS_SUSPENDING,
  MDM_SUSPENDED
};

enum caseLedInterval{
  LED_RESET_ONE,
  LED_RESET_TWO
};

void handle_led_reset(void)
{
  tr_info("LED reset handler");
  uint8_t case_led_interval = LED_RESET_ONE;
  uint32_t time_counter = 0;
  uint32_t interval_counter = 0;

  while(1)
  {
    switch(case_led_interval)
    {
      case LED_RESET_ONE:
        time_counter = (LED_RESET_INTERVAL_ONE / (5*1000 + 200) );
        tr_info("LED reset handler - CASE 1");

        /* assign interrupt callbacks as empty functions */
        pwr_det.rise(&pwr_magnet_place_dummy);
        pwr_det.fall(&pwr_magnet_detach_dummy);

        for(int count = 0; count < time_counter; count++)
        {
          watchdog.kick();
          LED.on(RED);
          pwr_det.enable_irq();
          lowPower.enterStop(200);
          LED.off();
          watchdog.kick();
          lowPower.enterStop(5000);

          if (pwr_det.read())
          {
            // Magnet placed
            tr_debug("Magnet HIGH");

            // detatch ticker to avoid watchdog reset while in reset sequence
            wdogTicker.detach();

            watchdog.kick();
            modemCanSuspend = true;
            pwr_magnet_place();
          }
        }
        case_led_interval = LED_RESET_TWO;
        break;

      case LED_RESET_TWO:
        time_counter = (LED_RESET_INTERVAL_TWO / (60*1000 + 200) );
        tr_info("LED reset handler - CASE 2");

        for(int count = 0; count < time_counter; count++)
        {
          watchdog.kick();
          LED.on(RED);
          lowPower.enterStop(200);
          LED.off();
          //interval_counter = (60000/20*1000);
          for(int interval=0; interval < 3; interval++)
          {
            watchdog.kick();
            lowPower.enterStop(20000);

            if (pwr_det.read())
            {
              // Magnet placed
              tr_debug("Magnet HIGH");

              // detatch ticker to avoid watchdog reset while in reset sequence
              wdogTicker.detach();

              watchdog.kick();
              modemCanSuspend = true;
              pwr_magnet_place();
            }
          }
        }
        case_led_interval = 3;
        break;

      default: 
      tr_info("LED reset handler - Default Case");
      break;
    }

    if(case_led_interval >= 3)
      break;
  }
  
}

uint8_t mdmState = MDM_CANNOT_SUSPEND;

void suspendSystem(void)
{
  bool magnetRemoved = false;

  suspendTime = time(NULL);

  tr_debug("Powering Off sequence started \r\n");

  watchdog.kick();
  /* deinit modem  */
  interface->deinit();
  tr_debug("Powering Off modem completed");
  
  /* wait for suspend time elapse */
  while(time(NULL) - suspendTime < MAGNET_SUSPEND_TIME)
  {
    watchdog.kick();
    wait(0.5);
    if (!pwr_det.read() && !magnetRemoved)
    {
      LED.blink(RED, 1.0);

      tr_debug("Error! Magnet is removed while the modem is suspending");

      magnetRemoved = true;
    }
  }

  suspendRequired = false;

  tr_debug("Power-off Unit...\r\n");

  // Going to stop mode would de-assert pin and hence we can power-off unit
  lowPower.powerDown();

  mdmState = MDM_SUSPENDED;

  /** Normally we shouldn't be here.
   *  If so then the user removed the
   *  magnet while the modem is being suspended.
   *  Raise error by blinking red led indefinitely
   */

  // There is almost zero chances system reach at this stage so lets ignore it.

  LED.off();

  uint32_t wd_rst_counter = WATCHDOG_RESET_TIMER / (5*1000 + 500);

  // /* loop until magnet is placed again */
  while ( wd_rst_counter > 0)
  {
    watchdog.kick();
    LED.off();
    wait(5.0);
    LED.on(MAGENTA);
    wait(0.5);
    wd_rst_counter--;
  }

  // Trigger a forced WD reset
  lowPower.enterStop(30000);
}


// pwr_magnet_place Interrupt routine - is interrupt activated by a falling edge of Magnet
void pwr_magnet_place(void)
{
  wait_ms(2000);

  if (pwr_det.read())
  {
    /* Enable system power_keep */
    lowPower.powerUp();

    //pwr_det.disable_irq();
    tr_debug("Placed magnet\r\n");

    LED.off();

    /* wait a bit to prevent arcs */
    wait(0.5);

    /* don't enter if modem is already suspending */
    if (!suspendRequired)
    {
      suspendRequired = true;

      if(modemCanSuspend)
      {
        suspendSystem();
      }
    }
    //pwr_det.enable_irq();
  }
}

void pwr_magnet_place_dummy(void)
{
}

void pwr_magnet_detach (void)
{
  //pwr_det.disable_irq();
  tr_debug("Magnet removed\r\n");

  /* wait a bit to prevent arcs */
  wait(0.5);

  if (suspendRequired)
  {
    suspendInterrupted = true;

    LED.off();
    LED.blink(RED, 1.0);

    tr_debug("Error! Magnet is removed while the modem is suspending");
  }
  else
  {
    tr_debug("Restarting");
  }
  //pwr_det.enable_irq();
}

void pwr_magnet_detach_dummy(void)
{
}

string get_reset_reason(uint8_t reason_rst)
{
  string result;

  switch(reason_rst)
  {
    case RESET_REASON_POWER_ON:
      result = "POWER_ON";
      break;
    case RESET_REASON_PIN_RESET:
      result = "PIN_RESET";
      break;
    case RESET_REASON_BROWN_OUT:
      result = "BROWN_OUT";
      break;
    case RESET_REASON_SOFTWARE:
      result = "SOFT_RESET";
      break;
    case RESET_REASON_WATCHDOG:
      result = "WATCHDOG";
      break;
    case RESET_REASON_LOCKUP:
      result = "LOCK_UP";
      break;
    case RESET_REASON_WAKE_LOW_POWER:
      result = "WAKE_UP_LP";
      break;
    case RESET_REASON_ACCESS_ERROR:
      result = "ACCES_ERROR";
      break;
    case RESET_REASON_BOOT_ERROR:
      result = "BOOT_ERROR";
      break;
    case RESET_REASON_MULTIPLE:
      result = "MULTIPLE_CAUSE";
      break;
    case RESET_REASON_PLATFORM:
      result = "PLATFORM_RESET";
      break;
    default:
      result = "RESET_REASON_UNKNOWN";
      break;
  }

  return result;
}

//New PCB VC2 New moisture board
int main(void)
{
  lowPower.powerUp();

  mbed_trace_init();
  mbed_trace_config_set(TRACE_ACTIVE_LEVEL_ALL);
  // Limit tracing to SPIIO GROUP
  mbed_trace_include_filters_set((char *)"SPIIO");

  // Use internal pullup for power detect pin : TODO evluate withoug internal pull up
  pwr_det.mode(PullUp);
  reset_reason_t reset = ResetReason::get();

  interface = new UbloxCellularHttp(MDMTXD, MDMRXD, MBED_CONF_APP_CELL_BAUD_RATE, MBED_CONF_APP_MODEM_TRACE);
  SPIIO::Device theDevice;

  // Delay for initial pullup to take effect
  wait(.01);

  // Attach the address of the interrupt handler routine for pushbutton
  pwr_det.rise(&pwr_magnet_place);
  pwr_det.fall(&pwr_magnet_detach);

  tr_debug("Spiio Sensor version: %s, model: %s, fwv: %s\r\n", VERSION, MODEL, FW_VER);

  lowPower.attach(&rtc_it_handle);
  lowPower.exitDebugMode();

  // createt a seeprate thread to schedule event based unit shutdown
  // SuspendTask.start(suspendSystem);

#if 1
  // start the watchdog ticker
  tr_debug("RESET reason is %i", reset);

  // Start wathdog with 28 sec timeout
  watchdog_status_t wdgStatus = watchdog.start(28000);
  tr_debug("Watchdog status: %i max timeout: %lu", wdgStatus, (int)watchdog.max_timeout());
  wdogTicker.attach(&kick_handle, 20.0);
#endif 

  //Normal work mode
  {
    /*********************************************************************************
    * 
    * Sensor production firmware section - initialization
    * 
    **********************************************************************************/

    //bool moisture_calibration = false;
    // LED ON BLINK WHITE
    if (reset != RESET_REASON_WATCHDOG)
    {
      LED.blink(WHITE, 0.2);
    }
 


    // This will write a secret to EEPROM if you enable it ...
    // Remember to disable again after running once on target
    if (0)
    {
      tr_debug("WRITING SECRET TO EEPROM !!");
      e2prom.write_secret((char *)"SP-110-LTE.a5ucpchkix0yjsy");
      wait(2);
    }
    if ((interface->init() && (e2prom.read_secret() != NULL)))
    {
      watchdog.kick();

      interface->set_functionality_mode(UbloxCellularBase::FUNC_FULL);

      interface->clear_sim_FPLMN();

      // If its first run or magnet is just removed
      if ((reset == RESET_REASON_POWER_ON) || (reset == RESET_REASON_PIN_RESET))
      {
        // call disconnect to appy MNO settings and run full scan
        interface->disconnect();
      }
      // Set the MNO profile
      // 0:   SW default
      // 1:   SIM ICCID select
      // 2:   ATT
      // 3:   Verizon
      // 4:   Telstra
      // 5:   TMO
      // 6:   China Telecom
      // 8:   Sprint
      // 19:  Vodafone
      // 21:  TELUS
      // 31:  DT
      // 100: Standard Europe
      //
      // R410M-02B-01 : We choose Sprint MNO for USA and apply a bandmask after
      //
      // R412M-02B-01 : allowed MNO profiles are:
      // 0, 1, 2, 5, 19, 31, 100.
      // MNO profile 4 for Telstra
      theDevice.set_mno_profile(interface, 100);

      // Set the selected RAT(s)
      // The RAT parameter(s) are specified in set_rat()
      // function as several combinations are possible
      theDevice.set_rat(interface);

      // Set the band mask for US to LTE-M bands 2, 4, 5, 12, 13 and 25 (16783386)
      // This should cover Sprint, Verizon and AT&T bands
      // Bands have the value 2 ^(band - 1), so ie. band 4 is 2^3
      // Also add bands 3, 8 and 20 (for Europe) (524420)
      // adding band 7, 28 for NZ and Telstra (67,108,928)
      // 508 18DE   (84,416,734)
      interface->set_bandmask(0,(uint32_t)151525598);
  
      // Disable Power Saving Mode (PSM)
      if (interface->set_PSM(0))
      {
        tr_debug("Power Save Mode was disabled");
      }
      else
      {
        tr_debug("Error - Power Save Mode was not disabled");
      }

      // Disable eDRX Mode (PSM)
      if (interface->set_eDRX(0))
      {
        tr_debug("eDRX was disabled");
      }
      else
      {
        tr_debug("Error - eDRX was not disabled");
      }

      watchdog.kick();
    
      // Reboot the modem for RAT, bandmask and PSM to take effect
      interface->reboot_modem();

      tr_debug("Rebooting modem ..");

      // Wait for modem to finish reboot
      wait(10.0);

      watchdog.kick();

      tr_debug("Read IMEI from Ublox, set the Device ID");
      tr_debug("Read Secret from EEPROM, set the Device Secret");
      SPIIO::Device *thisDevice = (SPIIO::Device *)&theDevice;
      thisDevice->id(interface->imei());
      thisDevice->secret(e2prom.read_secret());
      tr_debug("id = %s, secret = %s", thisDevice->id().c_str(), thisDevice->secret().c_str());
      tr_debug("MANUFACTURER = %s", interface->manufacturer());
      tr_debug("MODEL = %s", interface->model());
      tr_debug("FIRMWARE VERSION = %s", interface->fwversion());
      tr_debug("ICCID = %s", interface->iccid());
    }
    else
    {
      // archive reset reason
      char result[10];
      string rst_reason = get_reset_reason(reset);
      memcpy(result, &rst_reason[0], 10);
      tr_debug("Data to be archived is %s \r\n", result);

      if(!e2prom.read_reset_reason(result))
      {
        if(!e2prom.store_reset_reason(result))
          tr_debug("EEPROM not able to archive properly \r\n");
      }

      /* check waiting suspend */
      if(suspendRequired)
      {
        suspendSystem();
      }
      modemCanSuspend = true;
      watchdog.kick();
      interface->deinit();
      tr_debug("Get IMEI/Secret/MoistureCalibration failed!");
      // LED ON BLINK RED FOREVER
      if ((reset == RESET_REASON_POWER_ON) || (reset == RESET_REASON_PIN_RESET))
      {
        watchdog.kick();
        LED.off();
        LED.on(RED);
        wait(10.0);
        LED.off();

        handle_led_reset();
        tr_debug("Suspend and reboot");
        suspend_and_boot();

      }
      else
      {
        // Failed to start after Watchdog reboot. Wait for some time and try a new boot.
        tr_debug("Suspend and reboot");
	      wdogTicker.detach();
        suspend_and_boot();
      }
    }

    SPIIO::SpiioNetwork network(interface, theDevice, MBED_CONF_APP_DEFAULT_PIN, MBED_CONF_APP_APN, MBED_CONF_APP_USERNAME, MBED_CONF_APP_PASSWORD);

    // Create Spiio service API client with:
    // - Device information to identify the sensor to the spiio service.
    SPIIO::SpiioClient spiioClient(theDevice);

    bool connected = network.connect(connection_timeout);
    int timeResult;

    if (connected)
    {
      // Get internet time GMT
      // string reason = reset == RESET_REASON_WATCHDOG ? "WATCHDOG" : "RESET";
      string reason = get_reset_reason(reset);

      char dbg_reason[10] = {0};
      if(e2prom.read_reset_reason(dbg_reason))
        tr_debug("reset reson archive in eeprom is %s \r\n",dbg_reason);
      timeResult = spiioClient.time(network, reason, dbg_reason);
    }

    watchdog.kick();
    if (!connected || timeResult == SPIIO_FAILURE)
    {
      if (connected)
      {
        network.disconnect();
      }
      interface->deinit();
      // LED ON BLINK RED FOREVER
      tr_debug("Connection failure. Please restart device");

      /* check waiting suspend */
      if(suspendRequired)
      {
        suspendSystem();
      }
      modemCanSuspend = true;

      // TURN LIGHT OFF
      if ((reset == RESET_REASON_POWER_ON) || (reset == RESET_REASON_PIN_RESET))
      {
        // archive reset reason
        char result[10];
        string rst_reason = get_reset_reason(reset);
        memcpy(result, &rst_reason[0], 10);
        tr_debug("Data to be archived is %s \r\n", result);

        if(!e2prom.read_reset_reason(result))
        {
          if(!e2prom.store_reset_reason(result))
            tr_debug("EEPROM not able to archive properly \r\n");
        }

        modemCanSuspend = true;
	      watchdog.kick();
        LED.off();
        LED.on(RED);
        wait(10.0);
        LED.off();

        handle_led_reset();
        tr_debug("Suspend and reboot");
        suspend_and_boot();

      }
      else
      {
        // Failed to start after Watchdog reboot. Wait for some time and try a new boot.
        tr_debug("Suspend and reboot");
	      wdogTicker.detach();
        suspend_and_boot();
      }
    }

    // New firmware downloaded
    if (theDevice.needsUpdate())
    {
      tr_debug("GET new FW version V%s", theDevice.updateFW().c_str());
      string domain = SPIIO::Config::ftp::url;
      string path = "/" + string(MODEL) + "/" + string(VERSION) + "/V" + theDevice.updateFW();
      int res = network.getOtaFile(domain, path);
      if (res == SPIIO_SUCCESS)
      {
        tr_debug("New firmware received. Restarting device ....");
        if (e2prom.write_ota_info(UPDATE_FW_FILE_NAME))
        {
          network.disconnect();
          interface->deinit();
          wdogTicker.detach();
          watchdog_restart();
        }
        else
        {
          tr_debug("OTA eeprom write error!");
        }
      }
      else
      {
        tr_debug("FAILED to find new firmware version. ");
      }
    }

    /* check waiting suspend */
    if(suspendRequired)
    {
      suspendSystem();
    }
    modemCanSuspend = true;

    // Get the RSSI and calculate # of flashes
    uint8_t flashes = 0;
    int rssi_level = interface->rssi();

    if (-113 <= rssi_level && rssi_level <= -107) flashes = 1;
    if (-105 <= rssi_level && rssi_level <= -99) flashes = 2;
    if (-97 <= rssi_level && rssi_level <= -91) flashes = 3;
    if (-89 <= rssi_level && rssi_level <= -83) flashes = 4;
    if (-81 <= rssi_level && rssi_level <= -75) flashes = 5;
    if (-73 <= rssi_level && rssi_level <= -67) flashes = 6;
    if (-65 <= rssi_level && rssi_level <= -59) flashes = 7;
    if (-57 <= rssi_level && rssi_level <= -51) flashes = 8;

    tr_debug("RSSI is %d giving %d flashes", rssi_level, flashes);

    // Shutdown network connection and U-Blox modem
    network.disconnect();
    interface->deinit();

    // TURN LIGHT GREEN AND FLASH THE RSSI (1-10)
    if (reset != RESET_REASON_WATCHDOG)
    {
      watchdog.kick();
      LED.off();
      LED.on(GREEN);
      wait(3.0);
      LED.off();
      wait(2.0);

      for (int count = 0; count < flashes; count++)
      {
        LED.on(GREEN);
        wait(0.5);
        LED.off();
        wait(0.5);
      }
    }

    SPIIO::StoreConfig storeConfig;

    // Create board to handle MCU and sensor interface
    SPIIO::SpiioBoard board;

    // Create message store to hold meassurements
    SPIIO::MessageStore store(interface, storeConfig);

    int sleepInterval;

    /*********************************************************************************
    * 
    * Initialization finished - start Watchdog and enter main loop 
    * 
    **********************************************************************************/
#if 0
    // start the watchdog ticker
    Ticker wdogTicker;

    tr_debug("RESET reason is %i", reset);
    // Start wathdog with 28 sec timeout
    watchdog_status_t wdgStatus = watchdog.start(28000);
    tr_debug("Watchdog status: %i max timeout: %i", wdgStatus, watchdog.max_timeout());
    wdogTicker.attach(&kick_handle, 20.0);
#endif

    time_t ts_first = 0;
    time_t ts_now = 0;

    char reading[64] = {0};

    uint8_t sched_cnt = 0;
    uint8_t static rep_iter = sleep_schedule[sched_cnt][SCHED_ITEM_SIZE-1];

    // updating connection interval to 120 secs for main loop
    connection_timeout = 120;

    while (true)
    {
      uint8_t msg = 0x01;
      wdogTicker.detach();
      wdogTicker.attach(&kick_handle, 20.0);

      /* get timestamp before sample and publish */
      ts_first = WakeUp::get_ts_now();
      
      // Get mesasurements from the Board.
      memset(reading, 0x00, sizeof(reading));
      board.getMeasurement(reading);
		
      watchdog.kick();

      /* prevent suspending */
      modemCanSuspend = false;

      // Add the measurement to the message store.
      interface->init();
      watchdog.kick();

      /* check waiting suspend */
      if(suspendRequired)
      {
        suspendSystem();
      }

      store.add(reading);
      if (store.DoPostReadings())
      {
        watchdog.kick();
        if (network.connect(connection_timeout))
        {
          watchdog.kick();
          ////////////////expected start/////////////////////////////////////
          if(count<threshold)
          {
           int result = spiioClient.publish(network, store);
           ++count;
           ThisThread::sleep_for(TIMEOUT_MS / 10);
          }
          else
          {
           NVIC_SystemReset();
           count=0;
          }
        ////////////////////expected end//////////////////////////////////
          watchdog.kick();
		  
          // Detect new firmware version - restart sensor to apply new firmware
          if (theDevice.needsUpdate())
          {
            tr_debug("Trigger restart to perform FW update.");
            network.disconnect();
            interface->deinit();
            wdogTicker.detach();
            watchdog_restart();
          }

          if (result == SPIIO_FAILURE)
          {
            tr_debug("Posting readings failed.");
          }
          network.disconnect();
        }
        else
        {
          overrided_sleep_logic = true;
          connection_timeout = 180;
        }
      }

      /* check waiting suspend */
      if(suspendRequired)
      {
        suspendSystem();
      }
      modemCanSuspend = true;

      interface->deinit();
      watchdog.kick();

      // Set the board in sleep mode. The Store may have an updated collection interval.
      // sleepInterval = store.GetCollectionInterval();
      if(!overrided_sleep_logic)
      {
        sleepInterval = store.GetCollectionInterval();
      }
      else
      {
        tr_info("Schedule is %d \r\n", sched_cnt);
        tr_info("rep iteration is %d \r\n", rep_iter);
        if (rep_iter >= 0)
        {
          sleepInterval = sleep_schedule[sched_cnt][SCHED_ITEM_SIZE-2];
          sleepInterval = sleepInterval * (3600); //6*(3600*1000)

          watchdog.kick();
          if(sched_cnt < NUMBER_OF_SCHEDULER)
          {
            if(rep_iter == 0)
            {
              tr_info("Moving to next sleep cycle");
              sched_cnt++;
              if(sched_cnt < NUMBER_OF_SCHEDULER)
                rep_iter = sleep_schedule[sched_cnt][SCHED_ITEM_SIZE-1];
            }
            else
            {
              rep_iter--;
            }
          }
        }

        // If it's end of schedule
        if(sched_cnt == NUMBER_OF_SCHEDULER)
        {
          tr_info("Suspend System");
          sched_cnt = 0;
          suspend_and_boot();
        }
      }

      /* get timestamp after sample and publish */
      ts_now = WakeUp::get_ts_now();

      //tr_debug("Now: %d First: %d", (int)ts_now, (int)ts_first);

      int timeDiff = 0;

      /* exclude the time passed while sampling */
      timeDiff = (ts_now - ts_first);
      if (timeDiff < 0 || timeDiff > sleepInterval)
        timeDiff = 0;

      sleepInterval -= timeDiff;
      if (sleepInterval < 0)
        sleepInterval = 0;

      tr_debug("Sleep for ... %i secs.", sleepInterval);

      int partialInterval = sleepInterval % (SLEEP_PERIOD / 1000);
      if (partialInterval > 0)
      {
        sleepInterval -= partialInterval;
        if (sleepInterval < 0)
          sleepInterval = 0;
      }

      /* in case of an error (i.e. wrong response from RTC)  set interval to default */
      if ((sleepInterval < 0 || sleepInterval > store.GetCollectionInterval()) && (!overrided_sleep_logic))
      {
        sleepInterval = store.GetCollectionInterval();
      }

      int maxInterval = abs(sleepInterval * 1000 / SLEEP_PERIOD);      

      tr_debug("Going to sleep %i times.", maxInterval + (partialInterval ? 1 : 0));
      wait_ms(50);
      wdogTicker.detach();

      watchdog.kick();

      if (partialInterval)
      {
        lowPower.enterStop(partialInterval * 1000 - 10);  // exclude 10 ms since it is consumed in this function
        watchdog.kick();
      }

      /* assign interrupt callbacks as empty functions */
      pwr_det.rise(&pwr_magnet_place_dummy);
      pwr_det.fall(&pwr_magnet_detach_dummy);
      pwr_det.enable_irq();

      // lowPower.powerDown();
      for (int count = 0; count < maxInterval; count++)
      {
        watchdog.kick();
        wait_ms(10);
        // pwr_det.enable_irq();
        lowPower.enterStop(SLEEP_PERIOD - 20);  // exclude extra 10 ms since it is consumed in this function

        if (pwr_det.read())
        {
          // Magnet placed
          tr_debug("Magnet HIGH");

          // detatch ticker to avoid watchdog reset while in reset sequence
          wdogTicker.detach();

          modemCanSuspend = true;
          pwr_magnet_place();
        }
      }
      watchdog.kick();

      /* assign interrupt callbacks back to normal functions */
      pwr_det.rise(&pwr_magnet_place);
      pwr_det.fall(&pwr_magnet_detach);
    }
  }
}
