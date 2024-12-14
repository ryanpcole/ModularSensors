/** =========================================================================
 * @file CS500_logging.ino
 * @brief A simple data logging example for connecting the CS500 temp/rH sensor
 *
 * @author Ryan Cole
 * @copyright Stroud Water Research Center
 * This example is published under the BSD-3 license.
 *
 * Build Environment: Visual Studio Code with PlatformIO
 * Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
 *
 * DISCLAIMER:
 * THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
 * ======================================================================= */

// ==========================================================================
//  Defines for TinyGSM
// ==========================================================================
/** Start [defines] */
#ifndef TINY_GSM_RX_BUFFER
#define TINY_GSM_RX_BUFFER 64
#endif
#ifndef TINY_GSM_YIELD_MS
#define TINY_GSM_YIELD_MS 2
#endif
#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 240
#endif
/** End [defines] */


// ==========================================================================
//  Include the libraries required for any data logger
// ==========================================================================
/** Start [includes] */
// The Arduino library is needed for every Arduino program.
#include <Arduino.h>

// EnableInterrupt is used by ModularSensors for external and pin change
// interrupts and must be explicitly included in the main program.
#include <EnableInterrupt.h>

// Include the main header for ModularSensors
#include <ModularSensors.h>
/** End [includes] */


// ==========================================================================
//  Data Logging Options
// ==========================================================================
/** Start [logging_options] */
// The name of this program file
const char* sketchName = "CS500_logging.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char* LoggerID = "testlogger";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 1;
// Your logger's timezone.
const int8_t timeZone = -8;  // Pacific Standard Time
// NOTE:  Daylight savings time will not be applied!  Please use standard time!

// Set the input and output pins for the logger
// NOTE:  Use -1 for pins that do not apply
const int32_t serialBaud = 115200;  // Baud rate for debugging
const int8_t  greenLED   = 8;       // Pin for the green LED
const int8_t  redLED     = 9;       // Pin for the red LED
const int8_t  buttonPin  = 21;      // Pin for debugging mode (ie, button pin)
const int8_t  wakePin    = 31;  // MCU interrupt/alarm pin to wake from sleep
// Mayfly 0.x D31 = A7
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin   = -1;  // MCU SD card power pin
const int8_t sdCardSSPin    = 12;  // SD card chip select/slave select pin
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power
/** End [logging_options] */

// ==========================================================================
//  Wifi/Cellular Modem Options
//    NOTE:  DON'T USE MORE THAN ONE MODEM OBJECT!
//           Delete the sections you are not using!
// ==========================================================================

#if defined BUILD_MODEM_ESPRESSIF_ESP32
/** Start [espressif_esp32] */
// For almost anything based on the Espressif ESP8266 using the
// AT command firmware
#include <modems/EspressifESP32.h>

// NOTE: Extra hardware and software serial ports are created in the "Settings
// for Additional Serial Ports" section
// Create a reference to the serial port for the modem
HardwareSerial& modemSerial = Serial1;  // Use hardware serial if possible
const int32_t modemBaud = 57600;  // Communication speed of the modem
// NOTE:  This baud rate too fast for an 8MHz board, like the Mayfly!  The
// module should be programmed to a slower baud rate or set to auto-baud using
// the AT+UART_CUR or AT+UART_DEF command.

// Modem Pins - Describe the physical pin connection of your modem to your board
// NOTE:  Use -1 for pins that do not apply
// Example pins here are for a EnviroDIY ESP32 Bluetooth/Wifi Bee with
// Mayfly 1.1
const int8_t modemVccPin   = 18;      // MCU pin controlling modem power
const int8_t modemResetPin = -1;      // MCU pin connected to modem reset pin
const int8_t modemLEDPin   = redLED;  // MCU pin connected an LED to show modem
                                      // status

// Network connection information
const char* wifiId  = "XXXX";  // WiFi access point name
const char* wifiPwd = "XXXX";  // WiFi password (WPA2)

// Create the modem object
EspressifESP32 modemESP(&modemSerial, modemVccPin, modemResetPin, wifiId,
                        wifiPwd);
// Create an extra reference to the modem by a generic name
EspressifESP32 modem = modemESP;
/** End [espressif_esp32] */
// ==========================================================================
#endif

// ==========================================================================
//  Using the Processor as a Sensor
// ==========================================================================
/** Start [processor_sensor] */
#include <sensors/ProcessorStats.h>

// Create the main processor chip "sensor" - for general metadata
const char*    mcuBoardVersion = "v1.1";
ProcessorStats mcuBoard(mcuBoardVersion);

// Create sample number, battery voltage, and free RAM variable pointers for the
// processor
Variable* mcuBoardBatt = new ProcessorStats_Battery(
    &mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");
Variable* mcuBoardAvailableRAM = new ProcessorStats_FreeRam(
    &mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");
Variable* mcuBoardSampNo = new ProcessorStats_SampleNumber(
    &mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");

/** End [processor_sensor] */


// ==========================================================================
//  Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
/** Start [ds3231] */
#include <sensors/MaximDS3231.h>  // Includes wrapper functions for Maxim DS3231 RTC

// Create a DS3231 sensor object, using this constructor function:
MaximDS3231 ds3231(1);

// Create a temperature variable pointer for the DS3231
Variable* ds3231Temp =
    new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-ef00-1234567890ab");
/** End [ds3231] */


// ==========================================================================
//    Settings for Additional Sensors
// ==========================================================================
// Additional sensors can setup here, similar to the RTC, but only if
//   they have been supported with ModularSensors wrapper functions. See:
//   https://github.com/EnviroDIY/ModularSensors/wiki#just-getting-started
// Syntax for the include statement and constructor function for each sensor is
// at
//   https://github.com/EnviroDIY/ModularSensors/wiki#these-sensors-are-currently-supported
//   or can be copied from the `menu_a_la_carte.ino` example

// ==========================================================================
//  Campbell OBS 3 / OBS 3+ Analog Turbidity Sensor
// ==========================================================================
/** Start [campbell_obs3] */
#include <CS500tempRH.h>

// NOTE: Use -1 for any pins that don't apply or aren't being used.
const int8_t  CS500Power          = sensorPowerPin;  // Power pin
const uint8_t CS500NumberReadings = 10;
const uint8_t CS500ADSi2c_addr    = 0x48;  // The I2C address of the ADS1115 ADC

const int8_t CS500TempADSChannel = 0;  // ADS channel for temperature sensor
const int8_t CS500RHADSChannel = 1;  // ADS channel for humidity sensor

// Create a CS500 Sensor object
CS500tempRH cs500(CS500Power, 
                    CS500TempADSChannel, CS500RHADSChannel, 
                        CS500ADSi2c_addr, CS500NumberReadings);

// Create turbidity and voltage variable pointers for the low range  of the OBS3
Variable* cs500tempdegC = new CS500tempRH_Temp(
    &cs500, "12345678-abcd-1234-ef00-1234567890ab", "TempdegC");
Variable* cs500RHpct = new CS500tempRH_rH(
    &cs500, "12345678-abcd-1234-ef00-1234567890ab", "rHpct");
/** End [campbell_obs3] */



// ==========================================================================
//  Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================

/** Start [variables_pre_named] */
// Version 3: Fill array with already created and named variable pointers
Variable* variableList[] = {
    mcuBoardSampNo,
    mcuBoardBatt,
    ds3231Temp,
    cs500tempdegC,
    cs500RHpct
};
// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);
// Create the VariableArray object
VariableArray varArray(variableCount, variableList);
/** End [variables_pre_named] */



// ==========================================================================
//  The Logger Object[s]
// ==========================================================================
/** Start [loggers] */
// Create a new logger instance
Logger dataLogger(LoggerID, loggingInterval, &varArray);
/** End [loggers] */


// ==========================================================================
//  Working Functions
// ==========================================================================
/** Start [working_functions] */
// Flashes the LED's on the primary board
void greenredflash(uint8_t numFlash = 4, uint8_t rate = 75) {
    for (uint8_t i = 0; i < numFlash; i++) {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
        delay(rate);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
        delay(rate);
    }
    digitalWrite(redLED, LOW);
}

// Uses the processor sensor object to read the battery voltage
// NOTE: This will actually return the battery level from the previous update!
float getBatteryVoltage() {
    if (mcuBoard.sensorValues[PROCESSOR_BATTERY_VAR_NUM] == -9999) mcuBoard.update();
    return mcuBoard.sensorValues[PROCESSOR_BATTERY_VAR_NUM];
}

// Set the SW12v power pin high so it is constantly on for testing on a breadboard
// Power the sensors;
void power12v(int8_t sensorPowerPin) {
    if (sensorPowerPin > 0) {
    //Serial.println("Powering up sensors...");
    // pinMode(sensorPowerPin, OUTPUT);
    digitalWrite(sensorPowerPin, HIGH);
    //delay(200);
  }
}

/** End [working_functions] */


// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
/** Start [setup] */
void setup() {
    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Print a start-up note to the first serial port
    Serial.print(F("Now running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);

    #if (defined BUILD_MODEM_ESPRESSIF_ESP8266 || \
     defined BUILD_MODEM_ESPRESSIF_ESP32) &&  \
    F_CPU == 8000000L
    /** Start [setup_esp] */
    if (modemBaud > 57600) {
        modem.modemWake();  // NOTE:  This will also set up the modem
        modemSerial.begin(modemBaud);
        modem.gsmModem.sendAT(GF("+UART_DEF=9600,8,1,0,0"));
        modem.gsmModem.waitResponse();
        modemSerial.end();
        modemSerial.begin(9600);
    }
    /** End [setup_esp] */
    #endif

    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();

    pinMode(20, OUTPUT);  // for proper operation of the onboard flash memory
                          // chip's ChipSelect (Mayfly v1.0 and later)

    /* Only use this if sensor power pin is set to -1 because you want the
    switched power always on (useful for breadboard testing)
    if (sensorPowerPin < 0) {
        Serial.println("Powering up sensors...");
        pinMode(22, OUTPUT);
        digitalWrite(22, HIGH);
    }
    */


    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

    // Set information pins
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             greenLED);

    // Begin the logger
    dataLogger.begin();

    // Note:  Please change these battery voltages to match your battery
    // Set up the sensors, except at lowest battery level
    if (getBatteryVoltage() > 3.4) {
        Serial.println(F("Setting up sensors..."));
        varArray.setupSensors();
    }

    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct
    // Writing to the SD card can be power intensive, so if we're skipping
    // the sensor setup we'll skip this too.
    if (getBatteryVoltage() > 3.4) {
        Serial.println(F("Setting up file on SD card"));
        dataLogger.turnOnSDcard(true);
        // true = wait for card to settle after power up
        dataLogger.createLogFile(true);  // true = write a new header
        dataLogger.turnOffSDcard(true);
        // true = wait for internal housekeeping after write
    }
    // Call the processor sleep
    Serial.println(F("Putting processor to sleep\n"));
    // dataLogger.systemSleep();
}
/** End [setup] */


// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
/** Start [loop] */
void loop() {

    dataLogger.logData();

}
/** End [loop] */
