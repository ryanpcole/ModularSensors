/**
 * @file CS10Xtemp.h
 * @author Written By: Ryan Cole
 *
 * @brief Measures temperature using analog input and
 *  onboard ADC and ADC ref.
 */
/* clang-format off */
/**
 * @defgroup sensor_cs10X Temperature and relative humidity via a Campbell Sci
 * CS10X sensor
 *
 * @ingroup the_sensors
 *
 * @tableofcontents
 * @m_footernavigation
 *
 * @section sensor_cs10X_notes Introduction
 * This is for an old school CS10X temperature sensor that needs to be
 * shielded as part of a met station.
 * 
 * Temp sensor: 1000 ohm PRT, DIN 43760B
 * Temp Measurement Range: -40 degrees C to +60 degrees C
 * Temp Output Signal Range: 0 to 1.0 VDC
 * 
 *  *
 * @section sensor_cs10X_calcs Calculating the Temperature and Relative Humidity
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
 * @note  * Before applying any temp calibration, the analog output 
 * must be converted into a high resolution digital signal.  See the
 * [ADS1115 page](@ref analog_group) for details on the conversion.
 *
 *
 * @note The analog reference of the Mayfly is not broken out (and is tied to
 * ground).  If using a Mayfly, you have no choice by to use the internal analog
 * reference.
 *
 * Now we can calculate the temperature:
 *
 * `temp_degreesC = (meas_voltage_mV * 0.1) - 40`
 *  *
 * @section sensor_cs10X_ref References
 * - For the CS10X sensor manual:
 * http://s.campbellsci.com/documents/us/manuals/108.pdf
 *
 * @section sensor_cs10X_temp_flags Build flags
 * 
 * @section sensor_cs10X_ctor Sensor Constructor
 * {{ @ref CS10X::CS10X }}
 *
 * ___
 */
/* clang-format on */

// Header Guards
#ifndef SRC_SENSORS_CS10X_H_
#define SRC_SENSORS_CS10X_H_

#ifdef MS_CS10X_DEBUG
#define MS_DEBUGGING_STD "CS10Xtemp"
#endif
#ifdef MS_CS10X_DEBUG_DEEP
#define MS_DEBUGGING_DEEP "CS10Xtemp"
#endif
// Included Dependencies
#include "ModSensorDebugger.h"
#undef MS_DEBUGGING_STD
#undef MS_DEBUGGING_DEEP
#include "SensorBase.h"
#include "VariableBase.h"
#include "math.h"

/** @ingroup sensor_cs10X */
/**@{*/

// Sensor Specific Defines
/// @brief Sensor::_numReturnedValues; we get one value (Voltage) from the temp sensor
#define CS10X_NUM_VARIABLES 2
/// @brief Sensor::_incCalcValues; we calculate actual temperature from voltage
#define CS10X_INC_CALC_VARIABLES 1


/**
 * @anchor sensor_cs10X_parts_timing
 * @name Sensor Timing
 * The timing for analog conductivity via resistance.
 */
/**@{*/
/// @brief Sensor::_warmUpTime_ms; giving 2ms for warm-up.
#define CS10X_WARM_UP_TIME_MS 2
/// @brief Sensor::_stabilizationTime_ms; we give 1 second delay for
/// stabilization.
#define CS10X_STABILIZATION_TIME_MS 1000
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
#define CS10X_MEASUREMENT_TIME_MS 0
/**@}*/


/**
 * @anchor sensor_cs10X_tempV
 * @name Raw voltage from air temperature sensor
 * - Range: 0-3.3 V
 *
 * {{ @ref CS10X_tempV::CS10X_tempV }}
 * 
 *  */
/**@{*/
/**
 * @brief Decimals places in string representation; Temp and should both have 1
 *
 * Range of 0-5V5 with 16bit ADC - resolution of 0.0008 mV then converted to temp
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
/// "Volts"
#define TEMP_VOLTAGE_UNIT_NAME "Volts"
/// @brief Default variable short code; "V"
#define TEMP_VOLTAGE_DEFAULT_CODE "V"
/**@}*/


// Make a section for calculated temp C
/**
 * @anchor sensor_cs10X_tempC
 * @name Air temperature calculated from raw voltage 
 *
 * {{ @ref CS10X_tempC::CS10X_tempC }}
 * 
 *  */
/**@{*/
/**
 * @brief Using equation from manual to calculate temperature in degrees Celsius 
 * from the supplied voltage. 
 * 
 * Relevant equations:
 * Vmeasure / Vexcited = (1k Ohms) / (Rs + 40 kOhms + 1k Ohms) 
 * 
 * RS = 1 kOhm * (Vexcited / Vmeasure) - 41k Ohms
 * 
 * Tc = 1 / (A + B * ln(Rs) + C * (ln(Rs))^3 )  - 273.15 
 * 
 * 
 */
#define TEMP_DEGC_RESOLUTION 1
/// @brief Sensor vensor variable number; tempV is stored in sensorValues[0].
#define TEMP_DEGC_VAR_NUM 1
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


/// @brief The assumed address of the ADS1115, 1001 000 (ADDR = GND)
#define ADS1115_ADDRESS 0x48

/**
 * @brief Class for the CS10X Temperature Sensor
 *
 * @ingroup sensor_cs10X_temp
 */
class CS10Xtemp : public Sensor {
 public:
    /**
     * @brief Construct a new CS10Xtemp object.
     *     
     * @note ModularSensors only supports connecting the ADS1x15 to the primary
     * hardware I2C instance defined in the Arduino core.  Connecting the ADS to
     * a secondary hardware or software I2C instance is *not* supported!

     * @param powerPin The port pin providing power to the temp probe.
     * We use 3v3 voltage for this sensor's calibration
     * - The ADS1x15 requires an input voltage of 2.0-5.5V, but this library
     * assumes the ADS is powered with 3.3V.
     * 
     * @param adsChannelTemp The analog data channel _on the TI ADS1115_ that the
     * temp sensor is connected to (0-3).
     * @param coeff_A The (A) coefficient for the calibration _in volts_
     * @param coeff_B The (B) coefficient for the calibration _in volts_
     * @param coeff_C The (C) coefficient for the calibration _in volts_
     * @param i2cAddress The I2C address of the ADS 1x15, default is 0x48 (ADDR
     * = GND)
     * @param measurementsToAverage The number of measurements to average;
     * optional with default value of 1.
     */
    CS10Xtemp(int8_t powerPin,
                        uint8_t adsChannelTemp,
                        float coeff_A,
                        float coeff_B,
                        float coeff_C,
                        uint8_t i2cAddress            = ADS1115_ADDRESS,  
                        uint8_t measurementsToAverage = 1);

    /**
     * @brief Destroy the CS10Xtemp object - no action needed.
     */
    ~CS10Xtemp();

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
    float _coeff_A;
    float _coeff_B;
    float _coeff_C;
    uint8_t _i2cAddress;

};

/**
 * @brief The variable class used for Temperature and Relative Humidity measured 
 * using analog pins connected to CS10X sensor
 *
 * @ingroup sensor_cs10X_temp
 *
 */
class CS10Xtemp_Temp : public Variable {
 public:
    /**
     * @brief Construct a new  CS10Xtemp_Temp object.
     *
     * @param parentSense The parent CS10Xtemp providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "degC".
     */
    CS10Xtemp_Temp(
        CS10Xtemp* parentSense, const char* uuid = "",
        const char* varCode = TEMP_DEGC_DEFAULT_CODE)
        : Variable(parentSense,
                   (const uint8_t)TEMP_DEGC_VAR_NUM,
                   (uint8_t)TEMP_DEGC_RESOLUTION,
                   TEMP_DEGC_VAR_NAME,
                   TEMP_DEGC_UNIT_NAME, varCode, uuid) {}

    /**
     * @brief Construct a new CS10Xtemp_Temp object.
     *
     * @note This must be tied with a parent CS10Xtemp before it
     * can be used.
     */
    CS10Xtemp_Temp()
        : Variable((const uint8_t)TEMP_DEGC_VAR_NUM,
                   (uint8_t)TEMP_DEGC_RESOLUTION,
                   TEMP_DEGC_VAR_NAME,
                   TEMP_DEGC_UNIT_NAME,
                   TEMP_DEGC_DEFAULT_CODE) {}
    /**
     * @brief Destroy the CS10Xtemp_Temp object - no action needed.
     */
    ~CS10Xtemp_Temp() {}
};

/**@}*/
#endif  // SRC_SENSORS_CS10X_H_
