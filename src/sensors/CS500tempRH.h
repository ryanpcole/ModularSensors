/**
 * @file CS500tempRH.h
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
 * @ingroup analog_group
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

// Include the library config before anything else
#include "ModSensorConfig.h"

// Include the debugging config
#include "ModSensorDebugConfig.h"

// Define the print label[s] for the debugger
#ifdef MS_CS500_DEBUG
#define MS_DEBUGGING_STD "CS500tempRH"
#endif
#ifdef MS_CS500_DEBUG_DEEP
#define MS_DEBUGGING_DEEP "CS500tempRH"
#endif

// Include the debugger
#include "ModSensorDebugger.h"
#undef MS_DEBUGGING_STD
#undef MS_DEBUGGING_DEEP

// Include other in-library and external dependencies
#include "SensorBase.h"
#include "VariableBase.h"
#include "math.h" // TODO do I need this?

// Forward declaration
class AnalogVoltageReader;

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
 * @brief Decimals places in string representation; Temp and rH should both have 4
 *
 * Range of 0-5V5 with 16bit ADC - resolution of 0.0008 mV then converted to temp or rH
 */

/// @brief Minimum temperature voltage (mV)
#define TEMP_VOLTAGE_MIN 0
/// @brief Maximum temperature voltage (mV)
#define TEMP_VOLTAGE_MAX 1000
/// @brief Sensor vensor variable number; tempV is stored in sensorValues[0].
#define TEMP_VOLTAGE_VAR_NUM 0
/// @brief Decimal places in string representation
#ifdef MS_US_ADS1015
#define TEMP_VOLTAGE_RESOLUTION 1
#else
// @brief Decimal places in string representation
#define TEMP_VOLTAGE_RESOLUTION 4
#endif
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
 * @brief Decimals places in string representation; Temp and rH should both have 4
 *
 * Range of 0-5V5 with 16bit ADC - resolution of 0.0008 mV then converted to temp or rH
 */
/// @brief Minimum temperature voltage (mV)
#define RH_VOLTAGE_MIN 0
/// @brief Maximum temperature voltage (mV)
#define RH_VOLTAGE_MAX 1000
/// @brief Sensor vensor variable number; tempV is stored in sensorValues[0].
#define RH_VOLTAGE_VAR_NUM 1
/// @brief Decimal places in string representation
#ifdef MS_US_ADS1015
#define RH_VOLTAGE_RESOLUTION 1
#else
// @brief Decimal places in string representation
#define RH_VOLTAGE_RESOLUTION 4
#endif
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
/// @brief Minimum temperature (Degrees C)
#define TEMP_DEGC_MIN -40
/// @brief Maximum temperature (Degrees C)
#define TEMP_DEGC_MAX 60
/// @brief Decimal places in string representation
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

 /// @brief Minimum relative humidity (%)
#define RH_PERCENT_MIN 0
/// @brief Maximum relative humidity (%)
#define RH_PERCENT_MAX 100

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


/**
 * @brief Class for the analog Temperature and Relative Humidity monitor
 *
 * @ingroup sensor_analog_temprh
 */
class CS500tempRH : public Sensor {
 public:
    /**
     * @brief Construct a new CS500tempRH object. Need the power pin, the
     * analog data channel, and the calibration info.
     *
     * By default, this constructor will internally create a default
     * AnalogVoltageReader implementation for voltage readings, but a pointer to
     * a custom AnalogVoltageReader object can be passed in if desired.

     * @param powerPin The port pin providing power to the temp/rH probe.
     * Needs to be 12 V switched power pin (pin XX)

      @param analogChannelTemp The analog data channel or processor pin for voltage
     * measurements of the Temp sensor. The significance of the channel number depends on the
     * specific AnalogVoltageReader implementation used for voltage readings.
     * For example, with the default TI ADS1x15, this would be the ADC channel
     * (0-3) that the sensor is connected to.  Negative or invalid channel
     * numbers are not clamped and will cause the reading to fail and emit a
     * warning.
     * @param analogChannelRH The analog data channel or processor pin for voltage
     * measurements on the RH sensor. The significance of the channel number depends on the
     * specific AnalogVoltageReader implementation used for voltage readings.
     * For example, with the default TI ADS1x15, this would be the ADC channel
     * (0-3) that the sensor is connected to.  Negative or invalid channel
     * numbers are not clamped and will cause the reading to fail and emit a
     * warning.
     * @param measurementsToAverage The number of measurements to average;
     * optional with default value of 1.
     * @param analogVoltageReader Pointer to an AnalogVoltageReader object for
     * voltage measurements.  Pass nullptr (the default) to have the constructor
     * internally create and own an analog voltage reader.  For backward
     * compatibility, the default reader uses a TI ADS1115 or ADS1015.  If a
     * non-null pointer is supplied, the caller retains ownership and must
     * ensure its lifetime exceeds that of this object.

     */
    CS500tempRH(int8_t powerPin,
                        int8_t analogChannelTemp,
                        int8_t analogChannelRH,
                        uint8_t measurementsToAverage = 1,
                        AnalogVoltageReader* analogVoltageReader = nullptr);

    /**
     * @brief Destroy the CS500tempRH object - no action needed.
     */
    ~CS500tempRH() override;

    // Delete copy constructor and copy assignment operator to prevent shallow
    // copies
    CS500tempRH(const CS500tempRH&)            = delete;
    CS500tempRH& operator=(const CS500tempRH&) = delete;

    // Delete move constructor and move assignment operator
    CS500tempRH(CS500tempRH&&)            = delete;
    CS500tempRH& operator=(CS500tempRH&&) = delete;

    String getSensorLocation() override;

    bool setup() override;

    bool addSingleMeasurementResult() override;

 private:
    /// @brief Pointer to analog voltage reader
    AnalogVoltageReader* _analogVoltageReader = nullptr;
    /// @brief Flag to track if this object owns the analog voltage reader and
    /// should delete it in the destructor
    bool _ownsAnalogVoltageReader = false;
    // Also define the analog channels
    int8_t _analogChannelTemp = -1;
    int8_t _analogChannelRH   = -1;
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
    explicit CS500tempRH_Temp(
        CS500tempRH* parentSense, const char* uuid = "",
        const char* varCode = TEMP_DEGC_DEFAULT_CODE)
        : Variable(parentSense,
                   TEMP_DEGC_VAR_NUM,
                   TEMP_DEGC_RESOLUTION,
                   TEMP_DEGC_VAR_NAME,
                   TEMP_DEGC_UNIT_NAME, 
                   varCode, 
                   uuid) {}

    /**
     * @brief Destroy the CS500tempRH_Temp object - no action needed.
     */
    ~CS500tempRH_Temp() override = default;
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
    explicit CS500tempRH_rH(
        CS500tempRH* parentSense, const char* uuid = "",
        const char* varCode = RH_PERCENT_DEFAULT_CODE)
        : Variable(parentSense,
                   RH_PERCENT_VAR_NUM,
                   RH_PERCENT_RESOLUTION,
                   RH_PERCENT_VAR_NAME,
                   RH_PERCENT_UNIT_NAME, varCode, uuid) {}

    
    /**
     * @brief Destroy the CS500tempRH_Temp object - no action needed.
     */
    ~CS500tempRH_rH() override = default;
};

/**@}*/
#endif  // SRC_SENSORS_CS500_H_
