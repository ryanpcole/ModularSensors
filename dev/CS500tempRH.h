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
 * `meas_voltage = (analog_ref_voltage * raw_adc_bits) / ANALOG_CS500_ADC_RANGE`
 *
 * Assuming the voltage of the ADC reference is the same as that used to power
 * the EC resistor circuit we can replace the reference voltage with the sensor
 * power voltage:
 *
 * `meas_voltage = (sensor_power_voltage * raw_adc_bits) / ANALOG_CS500_ADC_RANGE`
 *
 * @note The Vcc going to the circuit (~12V) can and will vary, as battery
 * level gets low.  If possible, you should use setup the processor to use an
 * external reference (`-D ANALOG_EC_ADC_REFERENCE_MODE=EXTEERNAL`) and tie
 * the Aref pin to the sensor power pin.
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
 * - `-D CS500_ADC_RESOLUTION=##`
 *      - used to set the resolution of the processor ADC
 *      - @see #CS500_ADC_RESOLUTION
 * - `-D CS500_ADC_REFERENCE_MODE=xxx`
 *      - used to set the processor ADC value reference mode
 *      - @see #CS500_ADC_REFERENCE_MODE
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
#define MS_DEBUGGING_STD "AnalogCS500"
#endif
#ifdef MS_CS500_DEBUG_DEEP
#define MS_DEBUGGING_DEEP "AnalogCS500"
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
#define TEMP_VOLTAGE_RESOLUTION 3
/// @brief Sensor vensor variable number; tempV is stored in sensorValues[0].
#define TEMP_VOLTAGE_VAR_NUM 0
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "Voltage"
#define TEMP_VOLTAGE_VAR_NAME "Voltage"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/);
/// "volts"
#define TEMP_VOLTAGE_UNIT_NAME "volts"
/// @brief Default variable short code; "V"
#define TEMP_VOLTAGE_DEFAULT_CODE "V"
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
#define RH_VOLTAGE_RESOLUTION 3
/// @brief Sensor vensor variable number; tempV is stored in sensorValues[0].
#define RH_VOLTAGE_VAR_NUM 1
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "Voltage"
#define RH_VOLTAGE_VAR_NAME "Voltage"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/);
/// "volts"
#define RH_VOLTAGE_UNIT_NAME "volts"
/// @brief Default variable short code; "V"
#define RH_VOLTAGE_DEFAULT_CODE "V"
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







#if !defined ANALOG_ADC_RESOLUTION
/**
 * @brief Default resolution (in bits) of the voltage measurement
 *
 * The default for all boards is 10, use a build flag to change this, if
 * necessary.
 */
#define ANALOG_ADC_RESOLUTION 10
#endif  // ANALOG_ADC_RESOLUTION
/// @brief The maximum possible value of the ADC - one less than the resolution
/// shifted up one bit.
#define ANALOG_ADC_MAX ((1 << ANALOG_ADC_RESOLUTION) - 1)
/// @brief The maximum possible range of the ADC - the resolution shifted up one
/// bit.
#define ANALOG_ADC_RANGE (1 << ANALOG_ADC_RESOLUTION)

/* clang-format off */
#if !defined ANALOG_ADC_REFERENCE_MODE
#if defined (ARDUINO_ARCH_AVR) || defined (DOXYGEN)
/**
 * @brief The voltage reference mode for the processor's ADC.
 *
 * For an AVR board, this must be one of:
 * - `DEFAULT`: the default built-in analog reference of 5 volts (on 5V Arduino
 * boards) or 3.3 volts (on 3.3V Arduino boards)
 * - `INTERNAL`: a built-in reference, equal to 1.1 volts on the ATmega168 or
 * ATmega328P and 2.56 volts on the ATmega32U4 and ATmega8 (not available on the
 * Arduino Mega)
 * - `INTERNAL1V1`: a built-in 1.1V reference (Arduino Mega only)
 * - `INTERNAL2V56`: a built-in 2.56V reference (Arduino Mega only)
 * - `EXTERNAL`: the voltage applied to the AREF pin (0 to 5V only) is used as the
 * reference.
 *
 * If not set on an AVR board `DEFAULT` is used.
 *
 * For the best accuracy, use an `EXTERNAL` reference with the AREF pin
 * connected to the power supply for the EC sensor.
 */
#define ANALOG_ADC_REFERENCE_MODE DEFAULT
#endif
#if defined (ARDUINO_ARCH_SAMD) || defined (DOXYGEN)
/**
 * @brief The voltage reference mode for the processor's ADC.
 *
 * For a SAMD board, this must be one of:
 * - `AR_DEFAULT`: the default built-in analog reference of 3.3V
 * - `AR_INTERNAL`: a built-in 2.23V reference
 * - `AR_INTERNAL1V0`: a built-in 1.0V reference
 * - `AR_INTERNAL1V65`: a built-in 1.65V reference
 * - `AR_INTERNAL2V23`: a built-in 2.23V reference
 * - `AR_EXTERNAL`: the voltage applied to the AREF pin is used as the reference
 *
 * If not set on an SAMD board `AR_DEFAULT` is used.
 *
 * For the best accuracy, use an `EXTERNAL` reference with the AREF pin
 * connected to the power supply for the EC sensor.
 *
 * @see https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
 */
#define ANALOG_ADC_REFERENCE_MODE AR_DEFAULT
#endif
#if !defined ANALOG_ADC_REFERENCE_MODE
#error The processor ADC reference type must be defined!
#endif  // ANALOG_ADC_REFERENCE_MODE
#endif  // ARDUINO_ARCH_SAMD
/* clang-format on */


/**
 * @brief Class for the analog Temperature and Relative Humidity monitor
 *
 * @ingroup sensor_analog_temprh
 */
class AnalogCS500 : public Sensor {
 public:
    /**
     * @brief Construct a new AnalogCS500 object.
     *
     * @param powerPin The port pin providing power to the temp/rH probe.
     * Needs to be 12 V switched power pin (pin )
     * @param tempPin The processor ADC port pin to read the voltage from the temp sensor.
     * Not all processor pins can be used as analog pins.  Those usable
     * as analog pins generally are numbered with an "A" in front of the number
     * - ie, A1.
     * @param rHPin The processor ADC port pin to read the voltage from the rH sensor.
     * Not all processor pins can be used as analog pins.  Those usable
     * as analog pins generally are numbered with an "A" in front of the number
     * - ie, A1.
     * @param measurementsToAverage The number of measurements to average;
     * optional with default value of 1.
     */
    AnalogCS500(int8_t powerPin, int8_t tempPin, int8_t rHPin,
                           uint8_t measurementsToAverage = 1);

    /**
     * @brief Destroy the AnalogCS500 object - no action needed.
     */
    ~AnalogCS500();

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

    /**
     * @brief reads the calculated Temperature from an analog pin using the analog pin
     * number set in the constructor.
     *
     * @return The temperature in degress Celsius
     */
    float readTemp(void);
    /**
     * @brief reads the calculated Temperature from an analog pin using the analog pin
     * number set in the constructor.
     *
     * @return The temperature in degress Celsius
    */
    float readTemp(uint8_t analogPinNum);

    /**
     * @brief reads the calculated rH from an analog pin using the analog pin
     * number set in the constructor.
     *
     * @return The relative humidity in %
     */
    float readRH(void);
    /**
     * @brief reads the calculated rH from an analog pin using the analog pin
     * number set in the constructor.
     *
     * @return The relative humidity in %
     */
    float readRH(uint8_t analogPinNum);


 private:
    int8_t _PowerPin;
    int8_t _tempAdcPin;
    int8_t _rhAdcPin;

};

/**
 * @brief The variable class used for Temperature and Relative Humidity measured 
 * using analog pins connected to CS500 sensor
 *
 * @ingroup sensor_analog_temprh
 *
 */
class AnalogCS500_Temp : public Variable {
 public:
    /**
     * @brief Construct a new  AnalogCS500_Temp object.
     *
     * @param parentSense The parent AnalogCS500 providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "degC".
     */
    AnalogCS500_Temp(
        AnalogCS500* parentSense, const char* uuid = "",
        const char* varCode = TEMP_DEGC_DEFAULT_CODE)
        : Variable(parentSense,
                   (const uint8_t)TEMP_DEGC_VAR_NUM,
                   (uint8_t)TEMP_DEGC_RESOLUTION,
                   TEMP_DEGC_VAR_NAME,
                   TEMP_DEGC_UNIT_NAME, varCode, uuid) {}

    /**
     * @brief Construct a new AnalogCS500_Temp object.
     *
     * @note This must be tied with a parent AnalogCS500 before it
     * can be used.
     */
    AnalogCS500_Temp()
        : Variable((const uint8_t)TEMP_DEGC_VAR_NUM,
                   (uint8_t)TEMP_DEGC_RESOLUTION,
                   TEMP_DEGC_VAR_NAME,
                   TEMP_DEGC_UNIT_NAME,
                   TEMP_DEGC_DEFAULT_CODE) {}
    /**
     * @brief Destroy the AnalogCS500_Temp object - no action needed.
     */
    ~AnalogCS500_Temp() {}
};

/**
 * @brief The variable class used for Temperature and Relative Humidity measured 
 * using analog pins connected to CS500 sensor
 *
 * @ingroup sensor_analog_temprh
 *
 */
class AnalogCS500_rH : public Variable {
 public:
    /**
     * @brief Construct a new  AnalogCS500_Temp object.
     *
     * @param parentSense The parent AnalogCS500 providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "degC".
     */
    AnalogCS500_rH(
        AnalogCS500* parentSense, const char* uuid = "",
        const char* varCode = RH_PERCENT_DEFAULT_CODE)
        : Variable(parentSense,
                   (const uint8_t)RH_PERCENT_VAR_NUM,
                   (uint8_t)RH_PERCENT_RESOLUTION,
                   RH_PERCENT_VAR_NAME,
                   RH_PERCENT_UNIT_NAME, varCode, uuid) {}

    /**
     * @brief Construct a new AnalogCS500_Temp object.
     *
     * @note This must be tied with a parent AnalogCS500 before it
     * can be used.
     */
    AnalogCS500_rH()
        : Variable((const uint8_t)RH_PERCENT_VAR_NUM,
                   (uint8_t)RH_PERCENT_RESOLUTION,
                   RH_PERCENT_VAR_NAME,
                   RH_PERCENT_UNIT_NAME,
                   RH_PERCENT_DEFAULT_CODE) {}
    /**
     * @brief Destroy the AnalogCS500_Temp object - no action needed.
     */
    ~AnalogCS500_rH() {}
};

/**@}*/
#endif  // SRC_SENSORS_CS500_H_
