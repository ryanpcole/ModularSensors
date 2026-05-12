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
#include "TIADS1x15.h"


// For Mayfly version; the battery resistor depends on it
CS500tempRH::CS500tempRH(int8_t powerPin, 
                         int8_t analogChannelTemp, 
                         int8_t analogChannelRH, 
                         uint8_t measurementsToAverage,
                        AnalogVoltageReader* analogVoltageReader)
    : Sensor("CS500tempRH", 
             CS500_NUM_VARIABLES,         // total returned values
             CS500_WARM_UP_TIME_MS,       // warm up time
             CS500_STABILIZATION_TIME_MS, // stabilization time
             CS500_MEASUREMENT_TIME_MS,   // measurement time
             powerPin,                    // power pin
//             analogChannelTemp,
  //           analogChannelRH,                 
             measurementsToAverage,       // measurements to average
             CS500_INC_CALC_VARIABLES),   // number of included calculated variables
      // If no analog voltage reader was provided, create a default one
      _analogVoltageReader(analogVoltageReader == nullptr
                               ? new TIADS1x15Reader()
                               : analogVoltageReader),
      _ownsAnalogVoltageReader(analogVoltageReader == nullptr),
      // Define the analog channels here since we need two
      _analogChannelTemp(analogChannelTemp),
      _analogChannelRH(analogChannelRH) {}

// Destructor
CS500tempRH::~CS500tempRH() {
    // Clean up the analog voltage reader if we created it
    if (_ownsAnalogVoltageReader && _analogVoltageReader != nullptr) {
        delete _analogVoltageReader;
    } 
}

String CS500tempRH::getSensorLocation() {
        if (_analogVoltageReader != nullptr) {
        // Set the reference channel to -1 for a single-ended sensor
        return _analogVoltageReader->getAnalogLocation(_dataPin, -1);
    } else {
        return String(F("Unknown_AnalogVoltageReader"));
    }
}

bool CS500tempRH::setup() {
    bool sensorSetupSuccess         = Sensor::setup();
    bool analogVoltageReaderSuccess = false;

    if (_analogVoltageReader != nullptr) {
        analogVoltageReaderSuccess = _analogVoltageReader->begin();
        if (!analogVoltageReaderSuccess) {
            MS_DBG(getSensorNameAndLocation(),
                   F("Analog voltage reader initialization failed"));
        }
    } else {
        MS_DBG(getSensorNameAndLocation(),
               F("No analog voltage reader to initialize"));
    }

    return sensorSetupSuccess && analogVoltageReaderSuccess;
}


bool CS500tempRH::addSingleMeasurementResult() {
    // Perform common initialization checks
    if (!initializeMeasurementResult()) { return false; }

    // Check if we have a valid analog voltage reader
    if (_analogVoltageReader == nullptr) {
        MS_DBG(getSensorNameAndLocation(),
               F("No analog voltage reader available"));
        return finalizeMeasurementAttempt(false);
    }

    // TODO - how do I make this work for a sensor with multiple pins?
    // TEMP SENSOR
    // Read Analog to Digital Converter (ADC)
    // Taking this reading includes the 8ms conversion delay.
    // We're allowing the ADS1115 library to do the bit-to-volts conversion
    // for us
    float Temp_degC = MS_INVALID_VALUE;
    float temp_V =   MS_INVALID_VALUE;
    MS_DBG(getSensorNameAndLocation(), F("is reporting:"));

    bool tempSuccess = _analogVoltageReader->readVoltageSingleEnded(_analogChannelTemp,
                                                                    temp_V);

    if (tempSuccess) {
        float temp_mV = temp_V * 1000;
        Temp_degC = (0.1 * temp_mV) - 40;
        MS_DBG(F(" Temp_degC:"), Temp_degC)
        // Add Temperature measurement and voltage
        verifyAndAddMeasurementResult(TEMP_DEGC_VAR_NUM, Temp_degC);
        verifyAndAddMeasurementResult(TEMP_VOLTAGE_VAR_NUM, temp_mV);
    }

    // RH SENSOR
    // Read Analog to Digital Converter (ADC)
    // Taking this reading includes the 8ms conversion delay.
    // We're allowing the ADS1115 library to do the bit-to-volts conversion
    // for us
    float rH_pct = MS_INVALID_VALUE;
    float rH_V  = MS_INVALID_VALUE;
    MS_DBG(getSensorNameAndLocation(), F("is reporting:"));

    bool rHSuccess = _analogVoltageReader->readVoltageSingleEnded(_analogChannelRH,
                                                                  rH_V);

    if (rHSuccess) {
        float rH_mV = rH_V * 1000;
        rH_pct = rH_mV * 0.1;
        MS_DBG(F(" rH %:"), rH_pct)
        // Add Relative Humidity measurement and voltage
        verifyAndAddMeasurementResult(RH_PERCENT_VAR_NUM, rH_pct);
        verifyAndAddMeasurementResult(RH_VOLTAGE_VAR_NUM, rH_mV);    
    }

    // Return success value when finished
    return finalizeMeasurementAttempt(tempSuccess);

}
