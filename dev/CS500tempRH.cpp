/**
 * @file CS500tempRH.cpp
 * @copyright Stroud Water Research Center and Neil Hancock
 * Part of the EnviroDIY ModularSensors library
 * This library is published under the BSD-3 license.
 * @author Written By: Ryan Cole <ryan.cole@oregonstate.edu>
 *
 * @brief This encapsulates an old school CS500 temperature and rH sensor that needs to be
 * shielded as part of a met station. It has two analog outputs (temp and rH) and needs 12V power

 */

#include "CS500tempRH.h"
#include <Adafruit_ADS1015.h>


// For Mayfly version; the battery resistor depends on it
CS500tempRH::CS500tempRH(int8_t powerPin, 
                         uint8_t adsChannelTemp, 
                         uint8_t adsChannelRH, 
                         uint8_t i2cAddress, 
                         uint8_t measurementsToAverage)
    : Sensor("CS500tempRH", 
             CS500_NUM_VARIABLES,         // total returned values
             CS500_WARM_UP_TIME_MS,       // warm up time
             CS500_STABILIZATION_TIME_MS, // stabilization time
             CS500_MEASUREMENT_TIME_MS,   // measurement time
             powerPin,                    // power pin
             -1,                          // data pin (-1 for there isn' one)
             measurementsToAverage,       // measurements to average
             CS500_INC_CALC_VARIABLES),   // number of included calculated variables
      _adsChannelTemp(adsChannelTemp),
      _adsChannelRH(adsChannelRH),
      _i2cAddress(i2cAddress) {}

// Destructor
CS500tempRH::~CS500tempRH() {}

String CS500tempRH::getSensorLocation(void) {
    String sensorLocation = F("ADS1115_0x");
    sensorLocation += String(_i2cAddress, HEX);
    sensorLocation += F("_Channel");
    sensorLocation += String(_adsChannelTemp);
    sensorLocation += String(_adsChannelRH);
    return sensorLocation;
}


bool CS500tempRH::addSingleMeasurementResult(void) {
    // Variables to store the results in
    float temp_mV  = -9999;
    float rH_mV = -9999;
    float Temp_degC = -9999;
    float rH_pct = -9999;

    // Check a measurement was *successfully* started (status bit 6 set)
    // Only go on to get a result if it was
    if (bitRead(_sensorStatus, 6)) {
        MS_DBG(getSensorNameAndLocation(), F("is reporting:"));

// Create an Auxillary ADD object
// We create and set up the ADC object here so that each sensor using
// the ADC may set the gain appropriately without effecting others.
#ifndef MS_USE_ADS1015
        Adafruit_ADS1115 ads(_i2cAddress);  // Use this for the 16-bit version
#else
        Adafruit_ADS1015 ads(_i2cAddress);  // Use this for the 12-bit version
#endif
        // ADS Library default settings:
        //  - TI1115 (16 bit)
        //    - single-shot mode (powers down between conversions)
        //    - 128 samples per second (8ms conversion time)
        //    - 2/3 gain +/- 6.144V range (limited to VDD +0.3V max)
        //  - TI1015 (12 bit)
        //    - single-shot mode (powers down between conversions)
        //    - 1600 samples per second (625µs conversion time)
        //    - 2/3 gain +/- 6.144V range (limited to VDD +0.3V max)

        // Bump the gain up to 1x = +/- 4.096V range
        // Sensor return range is 0-2.5V, but the next gain option is 2x which
        // only allows up to 2.048V
        ads.setGain(GAIN_ONE);
        // Begin ADC
        ads.begin();

        // TEMP SENSOR
        // Read Analog to Digital Converter (ADC)
        // Taking this reading includes the 8ms conversion delay.
        // We're allowing the ADS1115 library to do the bit-to-volts conversion
        // for us
        temp_mV =
            ads.readADC_SingleEnded_V(_adsChannelTemp) * 1000;  // Getting the reading (in mV)
        MS_DBG(F("  ads.readADC_SingleEnded_V("), _adsChannelTemp, F("):"),
               temp_mV);
        if (temp_mV < 1000 && temp_mV > -1) {
            // Skip results out of range
            // Apply the unique calibration curve for the given sensor
            Temp_degC = (0.1 * temp_mV) - 40 ;
            MS_DBG(F("  Temp degC:"), Temp_degC);
        } else {  // set invalid voltages back to -9999
            temp_mV = -9999;
        }

        // RH SENSOR
        // Read Analog to Digital Converter (ADC)
        // Taking this reading includes the 8ms conversion delay.
        // We're allowing the ADS1115 library to do the bit-to-volts conversion
        // for us
        rH_mV =
            ads.readADC_SingleEnded_V(_adsChannelRH) / 1000;  // Getting the reading
        MS_DBG(F("  ads.readADC_SingleEnded_V("), _adsChannelRH, F("):"),
               rH_mV);

        if (rH_mV < 1000 && rH_mV > -1) {
            // Skip results out of range
            // Apply the unique calibration curve for the given sensor
            rH_pct = rH_mV * 0.1;
            MS_DBG(F("  rH %:"), rH_pct);
        } else {  // set invalid voltages back to -9999
            rH_mV = -9999;
        }

    } else {
        MS_DBG(getSensorNameAndLocation(), F("is not currently measuring!"));
    }

    // Add Temperature measurement and voltage
    verifyAndAddMeasurementResult(TEMP_DEGC_VAR_NUM, Temp_degC);
    verifyAndAddMeasurementResult(TEMP_VOLTAGE_VAR_NUM, temp_mV);

    // Add Relative Humidity measurement and voltage
    verifyAndAddMeasurementResult(RH_PERCENT_VAR_NUM, rH_pct);
    verifyAndAddMeasurementResult(RH_VOLTAGE_VAR_NUM, rH_mV);


    // Unset the time stamp for the beginning of this measurement
    _millisMeasurementRequested = 0;
    // Unset the status bits for a measurement request (bits 5 & 6)
    _sensorStatus &= 0b10011111;

    if (temp_mV < 1000 && temp_mV > -1) {
        return true;
    } else {
        return false;
    }
}
