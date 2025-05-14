/**
 * @file CS500.h
 * @author Written By: Ryan Cole
 *
 * @brief Measures temperature and relative humidity using two analog inputs and
 *  onboard ADC and ADC ref.
 */
/* clang-format off */
/**
 * @defgroup sensor_cs500 Temperature and relative humidity via a Campbell Sci
 * CS500 sensor
 *
 * @ingroup the_sensors
 *
 * @tableofcontents
 * @m_footernavigation
 *
 * @section sensor_cs500_notes Introduction
 * This is for an old school CS500 temperature and rH sensor that needs to be
 * shielded as part of a met station.
 * 
 * Temp sensor: 1000 ohm PRT, DIN 43760B
 * Temp Measurement Range: -40 degrees C to +60 degrees C
 * Temp Output Signal Range: 0 to 1.0 VDC
 * 
 * RH Sensor: Intercap
 * RH Measurement Range: 0 to 100% non-condensing
 * RH Output Signal Range: 0 to 1.0 VDC
 * 
 *  *
 * @section sensor_cs500_calcs Calculating the Temperature and Relative Humidity
 * First, we need to convert the bit reading of the ADC into volts based on the
 * range of the ADC (1 bit more than the resolution):
 *
 * `meas_voltage = (analog_ref_voltage * raw_adc_bits) / ADC_RANGE`
 *
 * Assuming the voltage of the ADC reference is the same as that used to power
 * the EC resistor circuit we can replace the reference voltage with the sensor
 * power voltage:
 *
 * `meas_voltage = (sensor_power_voltage * raw_adc_bits) / ADC_RANGE`
 * 
 * @note  * Before applying any temp or rH calibration, the analog output 
 * must be converted into a high resolution digital signal.  See the
 * [ADS1115 page](@ref analog_group) for details on the conversion.
 *
 *
 * @note The analog reference of the Mayfly is not broken out (and is tied to
 * ground).  If using a Mayfly, you have no choice by to use the internal analog
 * reference.
 *
 * Now we can calculate the air temperature:
 *
 * `temp_degreesC = (meas_voltage_mV * 0.1) - 40`
 * 
 * And the relative humidity:
 * `rH_pct = (meas_voltage_mV * 0.1)
 *
 * @section sensor_cs500_ref References
 * - For the CS500 sensor manual:
 * https://s.campbellsci.com/documents/us/manuals/cs500.pdf
 *
 * @section sensor_analog_temprh_flags Build flags
 * 
 * @section sensor_cs500_ctor Sensor Constructor
 * {{ @ref CS500::CS500 }}
 *
 * ___
 */
/* clang-format on */

// Header Guards
#ifndef SRC_SENSORS_CS500_H_
#define SRC_SENSORS_CS500_H_

#ifdef MS_CS500_DEBUG
#define MS_DEBUGGING_STD "CS500tempRH"
#endif
#ifdef MS_CS500_DEBUG_DEEP
#define MS_DEBUGGING_DEEP "CS500tempRH"
#endif
// Included Dependencies
#include "ModSensorDebugger.h"
#undef MS_DEBUGGING_STD
#undef MS_DEBUGGING_DEEP
#include "SensorBase.h"
#include "VariableBase.h"
#include "math.h"

/** @ingroup sensor_cs500 */
/**@{*/

// Sensor Specific Defines
/// @brief Sensor::_numReturnedValues; we get one value from the temp sensor
/// and one from the rH sensor
#define CS500_NUM_VARIABLES 4
/// @brief Sensor::_incCalcValues; we calculate actual temperature and rH from
/// voltages read
#define CS500_INC_CALC_VARIABLES 2


/**
 * @anchor sensor_cs500_parts_timing
 * @name Sensor Timing
 * The timing for analog conductivity via resistance.
 */
/**@{*/
/// @brief Sensor::_warmUpTime_ms; giving 2ms for warm-up.
#define CS500_WARM_UP_TIME_MS 2
/// @brief Sensor::_stabilizationTime_ms; we give 1 second delay for
/// stabilization.
#define CS500_STABILIZATION_TIME_MS 1000
/**
 * @brief Sensor::_measurementTime_ms; we assume the analog voltage is measured
 * instantly.
 *
 * It's not really *quite* instantly, but it is very fast and the time to
 * measure is included in the read function.
 * On ATmega based boards (UNO, Nano, Mini, Mega), it takes about 100
 * microseconds (0.0001 s) to read an analog input, so the maximum reading rate
 * is about 10,000 times a second.
 */
#define CS500_MEASUREMENT_TIME_MS 0
/**@}*/


/**
 * @anchor sensor_cs500_tempV
 * @name Raw voltage from air temperature sensor
 * - Range: 0-1.0 V
 *
 * {{ @ref CS500_tempV::CS500_tempV }}
 * 
 *  */
/**@{*/
/**
 * @brief Decimals places in string representation; Temp and rH should both have 1
 *
 * Range of 0-5V5 with 16bit ADC - resolution of 0.0008 mV then converted to temp or rH
 */
#define TEMP_VOLTAGE_RESOLUTION 4
/// @brief Sensor vensor variable number; tempV is stored in sensorValues[0].
#define TEMP_VOLTAGE_VAR_NUM 0
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "Voltage"
#define TEMP_VOLTAGE_VAR_NAME "Voltage"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/);
/// "millivolts"
#define TEMP_VOLTAGE_UNIT_NAME "millivolts"
/// @brief Default variable short code; "mV"
#define TEMP_VOLTAGE_DEFAULT_CODE "mV"
/**@}*/

//  Make a section for rH voltage
/**
 * @anchor sensor_cs500_rHV
 * @name Raw voltage from relative humidity sensor
 * - Range: 0-1.0 V
 *
 * {{ @ref CS500_rHV::CS500_rHV }}
 * 
 *  */
/**@{*/
/**
 * @brief Decimals places in string representation; Temp and rH should both have 1
 *
 * Range of 0-5V5 with 16bit ADC - resolution of 0.0008 mV then converted to temp or rH
 */
#define RH_VOLTAGE_RESOLUTION 4
/// @brief Sensor vensor variable number; tempV is stored in sensorValues[0].
#define RH_VOLTAGE_VAR_NUM 1
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "Voltage"
#define RH_VOLTAGE_VAR_NAME "Voltage"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/);
/// "millivolts"
#define RH_VOLTAGE_UNIT_NAME "millivolts"
/// @brief Default variable short code; "mV"
#define RH_VOLTAGE_DEFAULT_CODE "mV"
/**@}*/

// Make a section for calculated temp C
/**
 * @anchor sensor_cs500_tempC
 * @name Air temperature calculated from raw voltage 
 *
 * {{ @ref CS500_tempC::CS500_tempC }}
 * 
 *  */
/**@{*/
/**
 * @brief Using equation from manual to calculate temperature in degrees Celsius 
 * from the supplied voltage. 
 * 
 * Equation is:
 * degC = mV * 0.1 - 40
 * 
 */
#define TEMP_DEGC_RESOLUTION 1
/// @brief Sensor vensor variable number; tempV is stored in sensorValues[0].
#define TEMP_DEGC_VAR_NUM 2
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "Temperature"
#define TEMP_DEGC_VAR_NAME "Temperature"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/);
/// "Degree Celsius"
#define TEMP_DEGC_UNIT_NAME "Degree Celsius"
/// @brief Default variable short code; "degC"
#define TEMP_DEGC_DEFAULT_CODE "degC"
/**@}*/

// calcualted rH %
/**
 * @anchor sensor_cs500_rH
 * @name Relative humidity calculated from raw voltage 
 *
 * {{ @ref CS500_rH::CS500_rH }}
 * 
 *  */
/**@{*/
/**
 * @brief Using equation from manual to calculate relative humidity in percent
 * from the supplied voltage. 
 * 
 * Equation is:
 * rH = mV * 0.1
 * 
 */
#define RH_PERCENT_RESOLUTION 1
/// @brief Sensor vensor variable number; tempV is stored in sensorValues[0].
#define RH_PERCENT_VAR_NUM 3
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "Relative Humidity"
#define RH_PERCENT_VAR_NAME "Relative Humidity"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/);
/// "percent"
#define RH_PERCENT_UNIT_NAME "percent"
/// @brief Default variable short code; "rH
#define RH_PERCENT_DEFAULT_CODE "rH%"
/**@}*/

/// @brief The assumed address of the ADS1115, 1001 000 (ADDR = GND)
#define ADS1115_ADDRESS 0x48

/**
 * @brief Class for the analog Temperature and Relative Humidity monitor
 *
 * @ingroup sensor_analog_temprh
 */
class CS500tempRH : public Sensor {
 public:
    /**
     * @brief Construct a new CS500tempRH object.
     *     
     * @note ModularSensors only supports connecting the ADS1x15 to the primary
     * hardware I2C instance defined in the Arduino core.  Connecting the ADS to
     * a secondary hardware or software I2C instance is *not* supported!

     * @param powerPin The port pin providing power to the temp/rH probe.
     * Needs to be 12 V switched power pin (pin XX)
     * 
     * - The ADS1x15 requires an input voltage of 2.0-5.5V, but this library
     * assumes the ADS is powered with 3.3V.

     * @param adsChannelTemp The analog data channel _on the TI ADS1115_ that the
     * temp sensor is connected to (0-3).
     * @param adsChannel The analog data channel _on the TI ADS1115_ that the
     * rH sensor is connected to (0-3).
     * @param i2cAddress The I2C address of the ADS 1x15, default is 0x48 (ADDR
     * = GND)
     * @param measurementsToAverage The number of measurements to average;
     * optional with default value of 1.
     */
    CS500tempRH(int8_t powerPin,
                        uint8_t adsChannelTemp,
                        uint8_t adsChannelRH,
                        uint8_t i2cAddress            = ADS1115_ADDRESS,  
                        uint8_t measurementsToAverage = 1);

    /**
     * @brief Destroy the CS500tempRH object - no action needed.
     */
    ~CS500tempRH();

    /**
     * @brief Report the sensor info.
     *
     * @return **String** Text describing how the sensor is attached to the mcu.
     */
    String getSensorLocation(void) override;

    /**
     * @copydoc Sensor::addSingleMeasurementResult()
     */
    bool addSingleMeasurementResult(void) override;

 private:
    uint8_t _adsChannelTemp;
    uint8_t _adsChannelRH;
    uint8_t _i2cAddress;

};

/**
 * @brief The variable class used for Temperature and Relative Humidity measured 
 * using analog pins connected to CS500 sensor
 *
 * @ingroup sensor_analog_temprh
 *
 */
class CS500tempRH_Temp : public Variable {
 public:
    /**
     * @brief Construct a new  CS500tempRH_Temp object.
     *
     * @param parentSense The parent CS500tempRH providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "degC".
     */
    CS500tempRH_Temp(
        CS500tempRH* parentSense, const char* uuid = "",
        const char* varCode = TEMP_DEGC_DEFAULT_CODE)
        : Variable(parentSense,
                   (const uint8_t)TEMP_DEGC_VAR_NUM,
                   (uint8_t)TEMP_DEGC_RESOLUTION,
                   TEMP_DEGC_VAR_NAME,
                   TEMP_DEGC_UNIT_NAME, varCode, uuid) {}

    /**
     * @brief Construct a new CS500tempRH_Temp object.
     *
     * @note This must be tied with a parent CS500tempRH before it
     * can be used.
     */
    CS500tempRH_Temp()
        : Variable((const uint8_t)TEMP_DEGC_VAR_NUM,
                   (uint8_t)TEMP_DEGC_RESOLUTION,
                   TEMP_DEGC_VAR_NAME,
                   TEMP_DEGC_UNIT_NAME,
                   TEMP_DEGC_DEFAULT_CODE) {}
    /**
     * @brief Destroy the CS500tempRH_Temp object - no action needed.
     */
    ~CS500tempRH_Temp() {}
};

/**
 * @brief The variable class used for Temperature and Relative Humidity measured 
 * using analog pins connected to CS500 sensor
 *
 * @ingroup sensor_analog_temprh
 *
 */
class CS500tempRH_rH : public Variable {
 public:
    /**
     * @brief Construct a new  CS500tempRH_Temp object.
     *
     * @param parentSense The parent CS500tempRH providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "degC".
     */
    CS500tempRH_rH(
        CS500tempRH* parentSense, const char* uuid = "",
        const char* varCode = RH_PERCENT_DEFAULT_CODE)
        : Variable(parentSense,
                   (const uint8_t)RH_PERCENT_VAR_NUM,
                   (uint8_t)RH_PERCENT_RESOLUTION,
                   RH_PERCENT_VAR_NAME,
                   RH_PERCENT_UNIT_NAME, varCode, uuid) {}

    /**
     * @brief Construct a new CS500tempRH_Temp object.
     *
     * @note This must be tied with a parent CS500tempRH before it
     * can be used.
     */
    CS500tempRH_rH()
        : Variable((const uint8_t)RH_PERCENT_VAR_NUM,
                   (uint8_t)RH_PERCENT_RESOLUTION,
                   RH_PERCENT_VAR_NAME,
                   RH_PERCENT_UNIT_NAME,
                   RH_PERCENT_DEFAULT_CODE) {}
    /**
     * @brief Destroy the CS500tempRH_Temp object - no action needed.
     */
    ~CS500tempRH_rH() {}
};

/**@}*/
#endif  // SRC_SENSORS_CS500_H_
