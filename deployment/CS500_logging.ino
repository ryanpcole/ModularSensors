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
//  Include the libraries required for any data logger
// ==========================================================================
/** Start [includes] */
// The Arduino library is needed for every Arduino program.
#include <Arduino.h>

// EnableInterrupt is used by ModularSensors for external and pin change
// interrupts and must be explicitly included in the main program.
// #include <EnableInterrupt.h>

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
const char* samplingFeature = "12345678-abcd-1234-ef00-1234567890ab";
const uint8_t loggingInterval = 1;
// Your logger's timezone.
const int8_t timeZone = -8;  // Pacific Standard Time
// NOTE:  Daylight savings time will not be applied!  Please use standard time!

// Set the input and output pins for the logger
// NOTE:  Use -1 for pins that do not apply
const int32_t serialBaud    = 115200;  // Baud rate for debugging
const int8_t  greenLED      = 8;       // Pin for the green LED
const int8_t  redLED        = 9;       // Pin for the red LED
const int8_t  buttonPin     = 21;  // Pin for debugging mode (i.e., button pin)
uint8_t       buttonPinMode = INPUT;  // mode for debugging pin
// NOTE: On the Mayfly (and Stonefly), pin 21 is tied to a button that pulls the
// pin HIGH when pressed and an external pull-down that keeps the pin LOW when
// the button is not pressed. We want the pin mode to be INPUT - i.e., floating
// internally and pulled down externally until the button is pressed.  AVR
// processors like the 1284P on the Mayfly do not have internal pull-down
// resistors - they do not have an INPUT_PULLDOWN mode like SAMD processors.
const int8_t wakePin     = 31;  // MCU interrupt/alarm pin to wake from sleep
uint8_t      wakePinMode = INPUT_PULLUP;  // mode for wake pin
// Mayfly 0.x, 1.x D31 = A7
// NOTE: On the Mayfly, pin D31=A7 is tied directly to the RTC INT/SQW pin
// on the onboard DS3231 RTC.  The interrupt from the DS3231 will pull the pin
// DOWN, so we want the pin mode to be INPUT_PULLUP - i.e., pulled up until the
// RTC pulls it down.
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in RTC, set the wakePin to 1.
const int8_t sdCardPwrPin   = -1;  // MCU SD card power pin
const int8_t sdCardSSPin    = 12;  // SD card chip select/slave select pin
const int8_t flashSSPin     = 20;  // onboard flash chip select/slave select pin
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power
const int8_t sdi12DataPin   = 7;
const int8_t relayPowerPin = A3;  // MCU pin controlling an optional power relay
/** End [logging_options] */

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
//  Campbell CS500 Temp and RH sensor
// ==========================================================================
/** Start [campbell_cs500] */
#include <sensors/CS500tempRH.h>

// NOTE: Use -1 for any pins that don't apply or aren't being used.
const int8_t  CS500Power          = sensorPowerPin;  // Power pin
const uint8_t CS500NumberReadings = 10;

const int8_t CS500TempADSChannel = 0;  // ADS channel for temperature sensor
const int8_t CS500RHADSChannel = 1;  // ADS channel for humidity sensor

// Create a CS500 Sensor object
CS500tempRH cs500(CS500Power, 
                  CS500TempADSChannel,
                  CS500RHADSChannel, 
                  CS500NumberReadings);

// Create temp and rH variable pointers 
Variable* cs500tempdegC = new CS500tempRH_Temp(
    &cs500, "12345678-abcd-1234-ef00-1234567890ab", "TempdegC");
Variable* cs500RHpct = new CS500tempRH_rH(
    &cs500, "12345678-abcd-1234-ef00-1234567890ab", "rHpct");
/** End [campbell_cs500] */

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
    cs500RHpct};
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

#if defined(PIN_NEOPIXEL)
#include <Adafruit_NeoPixel.h>
// Declare our NeoPixel strip object:
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);
// Flashes the LED's on the primary board
void greenRedFlash(uint8_t numFlash = 4, uint8_t rate = 75) {
#if defined(PIN_NEOPIXEL_POWER)
    pinMode(PIN_NEOPIXEL_POWER, OUTPUT);
    digitalWrite(PIN_NEOPIXEL_POWER, HIGH);
#endif
    for (uint8_t i = 0; i < numFlash; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 255, 0));  // set to green
        pixels.show();  // Send the updated pixel colors to the hardware.
        delay(rate);
        pixels.setPixelColor(i, pixels.Color(255, 0, 0));  // set to red
        pixels.show();  // Send the updated pixel colors to the hardware.
        delay(rate);
    }
    pixels.clear();  // Set all pixel colors to 'off'
#if defined(PIN_NEOPIXEL_POWER)
    digitalWrite(PIN_NEOPIXEL_POWER, LOW);
#endif
}
#else
// Flashes the LED's on the primary board
void greenRedFlash(uint8_t numFlash = 4, uint8_t rate = 75) {
    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    // Flash the lights
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
#endif

// Uses the processor sensor object to read the battery voltage
// NOTE: This will actually return the battery level from the previous update!
float getBatteryVoltage() {
    if (mcuBoard.sensorValues[PROCESSOR_BATTERY_VAR_NUM] == MS_INVALID_VALUE ||
        mcuBoard.sensorValues[PROCESSOR_BATTERY_VAR_NUM] == 0) {
        mcuBoard.update();
    }
    return mcuBoard.sensorValues[PROCESSOR_BATTERY_VAR_NUM];
}
/** End [working_functions] */


// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
/** Start [setup] */
void setup() {
    /** Start [setup_flashing_led] */
    // Blink the LEDs to show the board is on and starting up
    greenRedFlash(3, 100);
    /** End [setup_flashing_led] */

    // IMMEDIATELY set up the watchdog timer for 5 minutes
    // The watchdog interval will be reset in the data logger's begin()
    // function.
    extendedWatchDog::setupWatchDog(static_cast<uint32_t>(5 * 60));

/** Start [setup_wait] */
// Wait for USB connection to be established by PC
// NOTE:  Only use this when debugging - if not connected to a PC, this adds an
// unnecessary startup delay
#if defined(SERIAL_PORT_USBVIRTUAL)
    while (!SERIAL_PORT_USBVIRTUAL && (millis() < 10000L)) { delay(10); }
    greenRedFlash(3, 10);
#endif
    /** End [setup_wait] */

    /** Start [setup_prints] */
    // Start the primary serial connection
    Serial.begin(serialBaud);
#if defined(MS_2ND_OUTPUT)
    MS_2ND_OUTPUT.begin(serialBaud);
#endif
    greenRedFlash(5, 50);

    // Print a start-up note to the first serial port
    PRINTOUT("\n\n\n=============================");
    PRINTOUT("=============================");
    PRINTOUT("=============================");
    PRINTOUT(F("\n\nNow running"), sketchName, F("on Logger"), LoggerID, '\n');

    PRINTOUT(F("Using ModularSensors Library version"),
             MODULAR_SENSORS_VERSION);
#if !defined(BUILD_MODEM_NO_MODEM) && defined(BUILD_HAS_MODEM)
    PRINTOUT(F("TinyGSM Library version"), TINYGSM_VERSION, '\n');
#endif
    PRINTOUT(F("Processor:"), mcuBoard.getSensorLocation());
    PRINTOUT(F("The most recent reset cause was"), mcuBoard.getLastResetCode(),
             '(', mcuBoard.getLastResetCause(), ")\n");
    /** End [setup_prints] */

/** Start [setup_softserial] */
// Allow interrupts for software serial
#if defined(BUILD_TEST_SOFTSERIAL)
    PRINTOUT(F("Enabling interrupts for SoftwareSerial"));
    enableInterrupt(softSerialRx, SoftwareSerial_ExtInts::handle_interrupt,
                    CHANGE);
#endif
#if defined(BUILD_TEST_NEOSWSERIAL)
    PRINTOUT(F("Enabling interrupts for NeoSoftSerial"));
    enableInterrupt(neoSSerial1Rx, neoSSerial1ISR, CHANGE);
#endif
/** End [setup_softserial] */

/** Start [setup_serial_begins] */
// Start the serial connection with the modem
#if !defined(BUILD_MODEM_NO_MODEM) && defined(BUILD_HAS_MODEM)
    PRINTOUT(F("Starting modem connection on"), STR(modemSerial), F("at"),
             modemBaud, F(" baud"));
    modemSerial.begin(modemBaud);
#endif

#if defined(BUILD_MODBUS_SENSOR)
    // Start the stream for the modbus sensors;
    // modbusSerial.begin(57600);  // 57600 is the default for ANB pH sensors
    modbusSerial.begin(9600);  // All other modbus sensors use 9600 baud
#endif

#if defined(BUILD_SENSOR_MAX_BOTIX_SONAR)
    // Start the stream for the sonar; it will always be at 9600 baud
    sonarSerial.begin(9600);
#endif

#if defined(BUILD_SENSOR_GEOLUX_HYDRO_CAM)
    // Start the stream for the camera; it will always be at 115200 baud
    cameraSerial.begin(115200);
#endif
/** End [setup_serial_begins] */

// Assign pins SERCOM functionality for SAMD boards
// NOTE:  This must happen *after* the various serial.begin statements
/** Start [setup_samd_pins] */
#if defined(ARDUINO_SAMD_FEATHER_M0)
    PRINTOUT(F("Setting SAMD21 SERCOM pin peripherals"));
    // Serial2
    pinPeripheral(10, PIO_SERCOM);  // Serial2 Tx/Dout = SERCOM1 Pad #2
    pinPeripheral(11, PIO_SERCOM);  // Serial2 Rx/Din = SERCOM1 Pad #0
    // Serial 3
    pinPeripheral(2, PIO_SERCOM);  // Serial3 Tx/Dout = SERCOM2 Pad #2
    pinPeripheral(5, PIO_SERCOM);  // Serial3 Rx/Din = SERCOM2 Pad #3
#endif
    /** End [setup_samd_pins] */

    // Start the SPI library
    PRINTOUT(F("Starting SPI"));
    SPI.begin();

#if defined(EXTERNAL_FLASH_DEVICES)
    PRINTOUT(F("Setting onboard flash pin modes"));
    pinMode(flashSSPin,
            OUTPUT);  // for proper operation of the onboard flash memory
#endif

    PRINTOUT(F("Starting I2C (Wire)"));
    Wire.begin();

    /** Start [setup_logger] */

    // set the logger ID
    PRINTOUT(F("Setting logger id to"), LoggerID);
    dataLogger.setLoggerID(LoggerID);
    PRINTOUT(F("Setting the sampling feature UUID to"), LoggerID);
    dataLogger.setSamplingFeatureUUID(samplingFeature);
    // set the logging interval
    PRINTOUT(F("Setting logging interval to"), loggingInterval, F("minutes"));
    dataLogger.setLoggingInterval(loggingInterval);
    PRINTOUT(F("Setting number of initial 1 minute intervals to 10"));
    dataLogger.setStartupMeasurements(10);
    // Attach the variable array to the logger
    PRINTOUT(F("Attaching the variable array"));
    dataLogger.setVariableArray(&varArray);
    // set logger pins
    PRINTOUT(F("Setting logger pins"));
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             greenLED, wakePinMode, buttonPinMode);

    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    PRINTOUT(F("Setting logger time zone"));
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    PRINTOUT(F("Setting RTC time zone"));
    loggerClock::setRTCOffset(0);

#if !defined(BUILD_MODEM_NO_MODEM) && defined(BUILD_HAS_MODEM)
    // Attach the modem and information pins to the logger
    PRINTOUT(F("Attaching the modem"));
    dataLogger.attachModem(modem);
    PRINTOUT(F("Setting modem LEDs"));
    modem.setModemLED(modemLEDPin);
#endif

#if defined(BUILD_PUB_THING_SPEAK_PUBLISHER) && \
    (!defined(BUILD_MODEM_NO_MODEM) && defined(BUILD_HAS_MODEM))
    // Set the ThingSpeak MQTT client name
    TsMqtt.setRESTAPIKey(thingSpeakAPIKey);
#endif

#if defined(BUILD_PUB_S3_PRESIGNED_PUBLISHER)
    // Set the S3 host and certificate authority name
    s3pub.setHost(s3Host);
    s3pub.setCACertName(caCertName);
    // Attach to the logger
    s3pub.attachToLogger(dataLogger);
#endif

#if defined(BUILD_PUB_AWS_IO_T_PUBLISHER) &&     \
    defined(BUILD_PUB_S3_PRESIGNED_PUBLISHER) && \
    (!defined(BUILD_MODEM_NO_MODEM) && defined(BUILD_HAS_MODEM))
    // Set the callback function for the AWS IoT Core MQTT connection
    awsIoTPub.setCallback(IoTCallback);
    awsIoTPub.addSubTopic(s3URLSubTopic.c_str());
    awsIoTPub.addPublishRequest(s3URLPubTopic.c_str(), s3URLMsgGetter);
#endif

    // Begin the logger
    PRINTOUT(F("Beginning the logger"));
    dataLogger.begin();
    /** End [setup_logger] */

    /** Start [setup_sensors] */
    // Note:  Please change these battery voltages to match your battery
    // Set up the sensors, except at lowest battery level
    if (getBatteryVoltage() > 3.4) {
        PRINTOUT(F("Setting up sensors..."));
        varArray.sensorsPowerUp();  // only needed if you have sensors that need
                                    // power for setups
        varArray.setupSensors();
        varArray.sensorsPowerDown();  // only needed if you have sensors that
                                      // need power for setups
    }
    /** End [setup_sensors] */

#if (defined BUILD_MODEM_ESPRESSIF_ESP8266 || \
     defined BUILD_MODEM_ESPRESSIF_ESP32)
    /** Start [setup_esp] */
    PRINTOUT(F("Waking the modem.."));
    PRINTOUT(F("Attempting to begin modem communication at"), modemBaud,
             F("baud.  This will fail if the baud is mismatched.."));
    modemSerial.begin(modemBaud);
    modem.modemWake();  // NOTE:  This will also set up the modem
    // WARNING: PLEASE REMOVE AUTOBAUDING FOR PRODUCTION CODE!
    if (!modem.gsmModem.testAT()) {
        PRINTOUT(F("Attempting to force the modem baud rate."));
        modem.gsmModem.forceModemBaud(modemSerial,
                                      static_cast<uint32_t>(modemBaud));
    }
/** End [setup_esp] */
#endif

#if defined(BUILD_TEST_SKYWIRE)
    /** Start [setup_skywire] */
    modem.setModemStatusLevel(LOW);  // If using CTS, LOW
    modem.setModemWakeLevel(HIGH);   // Skywire dev board inverts the signal
    modem.setModemResetLevel(HIGH);  // Skywire dev board inverts the signal
    /** End [setup_skywire] */
#endif

#if defined(BUILD_MODEM_SIM_COM_SIM7080)
    /** Start [setup_sim7080] */
    modem.setModemWakeLevel(HIGH);   // ModuleFun Bee inverts the signal
    modem.setModemResetLevel(HIGH);  // ModuleFun Bee inverts the signal
    PRINTOUT(F("Waking modem and setting Cellular Carrier Options..."));
    modem.modemWake();  // NOTE:  This will also set up the modem
    // WARNING: PLEASE REMOVE AUTOBAUDING FOR PRODUCTION CODE!
    if (!modem.gsmModem.testAT()) {
        PRINTOUT(F("Attempting to force the modem baud rate."));
        modem.gsmModem.forceModemBaud(modemSerial,
                                      static_cast<uint32_t>(modemBaud));
    }
    modem.gsmModem.setNetworkMode(38);   // set to LTE only
                                         // 2 Automatic
                                         // 13 GSM only
                                         // 38 LTE only
                                         // 51 GSM and LTE only
    modem.gsmModem.setPreferredMode(1);  // set to CAT-M
                                         // 1 CAT-M
                                         // 2 NB-IoT
                                         // 3 CAT-M and NB-IoT
    /** End [setup_sim7080] */
#endif

#if defined(BUILD_MODEM_DIGI_XBEE_CELLULAR_TRANSPARENT)
    /** Start [setup_xbeec_carrier] */
    // Extra modem set-up
    PRINTOUT(F("Waking modem and setting Cellular Carrier Options..."));
    modem.modemWake();  // NOTE:  This will also set up the modem
    // Go back to command mode to set carrier options
    modem.gsmModem.commandMode();
    // Carrier Profile - 0 = Automatic selection
    //                 - 1 = No profile/SIM ICCID selected
    //                 - 2 = AT&T
    //                 - 3 = Verizon
    // NOTE:  To select T-Mobile, you must enter bypass mode!
    modem.gsmModem.sendAT(GF("CP"), 2);
    modem.gsmModem.waitResponse();
    // Cellular network technology - 0 = LTE-M with NB-IoT fallback
    //                             - 1 = NB-IoT with LTE-M fallback
    //                             - 2 = LTE-M only
    //                             - 3 = NB-IoT only
    // NOTE:  As of 2020 in the USA, AT&T and Verizon only use LTE-M
    // T-Mobile uses NB-IOT
    modem.gsmModem.sendAT(GF("N#"), 2);
    modem.gsmModem.waitResponse();
    // Write changes to flash and apply them
    PRINTOUT(F("Wait while applying changes..."));
    // Write changes to flash
    modem.gsmModem.writeChanges();
    // Reset the cellular component to ensure network settings are changed
    modem.gsmModem.sendAT(GF("!R"));
    modem.gsmModem.waitResponse(30000L);
    // Force reset of the Digi component as well
    // This effectively exits command mode
    modem.gsmModem.sendAT(GF("FR"));
    modem.gsmModem.waitResponse(5000L);
/** End [setup_xbeec_carrier] */
#endif

#if defined(BUILD_MODEM_DIGI_XBEE_LTE_BYPASS)
    /** Start [setup_r4_carrier] */
    // Extra modem set-up
    PRINTOUT(F("Waking modem and setting Cellular Carrier Options..."));
    modem.modemWake();  // NOTE:  This will also set up the modem
    // Turn off the cellular radio while making network changes
    modem.gsmModem.sendAT(GF("+CFUN=0"));
    modem.gsmModem.waitResponse();
    // Mobile Network Operator Profile - 0 = SW default
    //                                 - 1 = SIM ICCID selected
    //                                 - 2: ATT
    //                                 - 6: China Telecom
    //                                 - 100: Standard Europe
    //                                 - 4: Telstra
    //                                 - 5: T-Mobile US
    //                                 - 19: Vodafone
    //                                 - 3: Verizon
    //                                 - 31: Deutsche Telekom
    modem.gsmModem.sendAT(GF("+UMNOPROF="), 2);
    modem.gsmModem.waitResponse();
    // Selected network technology - 7: LTE Cat.M1
    //                             - 8: LTE Cat.NB1
    // Fallback network technology - 7: LTE Cat.M1
    //                              - 8: LTE Cat.NB1
    // NOTE:  As of 2020 in the USA, AT&T and Verizon only use LTE-M
    // T-Mobile uses NB-IOT
    modem.gsmModem.sendAT(GF("+URAT="), 7, ',', 8);
    modem.gsmModem.waitResponse();
    // Restart the module to apply changes
    modem.gsmModem.sendAT(GF("+CFUN=1,1"));
    modem.gsmModem.waitResponse(10000L);
/** End [setup_r4_carrier] */
#endif

    /** Start [setup_clock] */
    // Sync the clock if it isn't valid or we have battery to spare
    if (getBatteryVoltage() > 3.55 || !loggerClock::isRTCSane()) {
        // Set up the modem, synchronize the RTC with NIST, and publish
        // configuration information to publishers that support it.
        dataLogger.makeInitialConnections();
    }
    /** End [setup_clock] */

    /** Start [setup_file] */
    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct.
    // Writing to the SD card can be power intensive, so if we're skipping the
    // sensor setup we'll skip this too.
    if (getBatteryVoltage() > 3.4) {
        PRINTOUT(F("Setting up file on SD card"));
        dataLogger.turnOnSDcard(true);
        // true = wait for card to settle after power up
        dataLogger.createLogFile(true);  // true = write a new header
        dataLogger.turnOffSDcard(true);
        // true = wait for internal housekeeping after write
    }
    /** End [setup_file] */

    /** Start [setup_sleep] */
    // Call the processor sleep
    PRINTOUT(F("Putting processor to sleep\n"));
    dataLogger.systemSleep();
    /** End [setup_sleep] */
}
/** End [setup] */


// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
// Use this short loop for simple data logging and sending
/** Start [simple_loop] */
void loop() {
    // Note:  Please change these battery voltages to match your battery
    // At very low battery, just go back to sleep
    if (getBatteryVoltage() < 3.4) {
        PRINTOUT(F("Battery too low, ("),
                 mcuBoard.sensorValues[PROCESSOR_BATTERY_VAR_NUM],
                 F("V) going back to sleep."));
        dataLogger.systemSleep();
    } else {
        // If the battery is good enough to log, log the data but we have no
        // modem so we can't publish
        PRINTOUT(F("Battery at"),
                 mcuBoard.sensorValues[PROCESSOR_BATTERY_VAR_NUM],
                 F("V; high enough to log data"));
        dataLogger.logData();
    }
}

/** End [simple_loop] */
