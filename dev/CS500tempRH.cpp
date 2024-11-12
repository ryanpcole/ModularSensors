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

// For Mayfly version; the battery resistor depends on it
CS500tempRH::CS500tempRH(int8_t powerPin, int8_t tempPin, int8_t rHPin,
                                uint8_t i2cAddress,
                                uint8_t measurementsToAverage)
    : Sensor("CS500tempRH", CS500_NUM_VARIABLES,
             CS500_WARM_UP_TIME_MS,
             CS500_STABILIZATION_TIME_MS,
             CS500_MEASUREMENT_TIME_MS, powerPin, tempPin, rHPin,
             measurementsToAverage, CS500_INC_CALC_VARIABLES),
      _PowerPin(powerPin),
      _tempAdcPin(tempPin),
      _rhAdcPin(rHPin) {}
// Destructor
CS500tempRH::~CS500tempRH() {}

String CS500tempRH::getSensorLocation(void) {
    String sensorLocation = F("anlgEc Proc Data/Pwr");
    sensorLocation += String(_tempAdcPin) + String(_rhAdcPin);
    return sensorLocation;
}

// Case when no argument given to readTemp - assume _tempAdcPin
float CS500tempRH::readTemp() {
    return readTemp(_tempAdcPin);
}

// To read the Temp sensor
float CS500tempRH::readTemp(uint8_t analogPinNum) {
    uint32_t sensorTemp_adc;
    float Temp_degC = -9999; // Set to -9999 as nodata  default

    // Set the resolution for the processor ADC, only applies to SAMD boards.
#if !defined ARDUINO_ARCH_AVR
    analogReadResolution(ADC_RESOLUTION);
#endif  // ARDUINO_ARCH_AVR
    // Set the analog reference mode for the voltage measurement.
    // If possible, to get the best results, an external reference should be
    // used.
    analogReference(ADC_REFERENCE_MODE);

    // First measure the analog voltage.
    // The return value from analogRead() is IN BITS NOT IN VOLTS!!
    // Take a priming reading.
    // First reading will be low - discard
    analogRead(analogPinNum);
    // Take the reading we'll keep
    sensorTemp_adc = analogRead(analogPinNum);
    MS_DEEP_DBG("adc bits=", sensorTemp_adc);

    if (0 == sensorTemp_adc) {
        // Prevent underflow, can never be ADC_RANGE
        sensorTemp_adc = 1;
    }

    // Convert to Temp Degrees C
    Temp_degC = sensorTemp_adc * 0.1 - 40;
    MS_DEEP_DBG("temp=", Temp_degC);

    return Temp_degC;
}


// Case when no argument given to readRH - assume _rhAdcPin
float CS500tempRH::readRH() {
    return readRH(_rhAdcPin);
}

// To read the RH sensor
float CS500tempRH::readRH(uint8_t analogPinNum) {
    uint32_t sensorRH_adc;
    float RH_pct = -9999; // Set to -9999 as nodata  default

    // Set the resolution for the processor ADC, only applies to SAMD boards.
#if !defined ARDUINO_ARCH_AVR
    analogReadResolution(ADC_RESOLUTION);
#endif  // ARDUINO_ARCH_AVR
    // Set the analog reference mode for the voltage measurement.
    // If possible, to get the best results, an external reference should be
    // used.
    analogReference(ADC_REFERENCE_MODE);

    // First measure the analog voltage.
    // The return value from analogRead() is IN BITS NOT IN VOLTS!!
    // Take a priming reading.
    // First reading will be low - discard
    analogRead(analogPinNum);
    // Take the reading we'll keep
    sensorRH_adc = analogRead(analogPinNum);
    MS_DEEP_DBG("adc bits=", sensorTemp_adc);

    if (0 == sensorRH_adc) {
        // Prevent underflow, can never be ADC_RANGE
        sensorRH_adc = 1;
    }

    // Convert to Temp Degrees C
    RH_pct = sensorRH_adc * 0.1;
    MS_DEEP_DBG("RH=", RH_pct);

    return RH_pct;
}

// I think this is what actually takes the measurement and adds it
// to the sensor. It also can debug, but I'm not sure how.
bool CS500tempRH::addSingleMeasurementResult(void) {
    float sensorTemp_degC = -9999;
    float sensorRH_pct = -9999;


    if (bitRead(_sensorStatus, 6)) {
        MS_DBG(getSensorNameAndLocation(), F("is reporting:"));

        sensorTemp_degC = readTemp(_tempAdcPin);
        sensorRH_pct = readRH(_rhAdcPin);

        MS_DBG(F("Air Temp (C) "), sensorTemp_degC, 
               F("\nAir RH (%) "), sensorRH_pct);
    } else {
        MS_DBG(getSensorNameAndLocation(), F("is not currently measuring!"));
    }

    // Verify and add the two measurements to the sensor
    verifyAndAddMeasurementResult(TEMP_DEGC_VAR_NUM,
                                  sensorTemp_degC);
    verifyAndAddMeasurementResult(RH_PERCENT_VAR_NUM,
                                  sensorRH_pct);


    // Unset the time stamp for the beginning of this measurement
    _millisMeasurementRequested = 0;
    // Unset the status bits for a measurement request (bits 5 & 6)
    _sensorStatus &= 0b10011111;

    // Return true when finished
    return true;
}
