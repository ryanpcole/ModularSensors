/**
 * @file mqttPublisher.h
 * @copyright Stroud Water Research Center
 * Part of the EnviroDIY ModularSensors library for Arduino.
 * This library is published under the BSD-3 license.
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 *
 * @brief Contains the mqttPublisher subclass of dataPublisher for
 * publishing data to ThingSpeak using the MQTT protocol.
 * 
 * TODO: need to remove the following as they are only needed for thingspeak:
 * - channelID
 * - channelKey
 * - MQTTKey
 */

// Header Guards
#ifndef SRC_PUBLISHERS_MQTTPUBLISHER_H_
#define SRC_PUBLISHERS_MQTTPUBLISHER_H_


// Debugging Statement
#define MS_MQTTPUBLISHER_DEBUG

#ifdef MS_MQTTPUBLISHER_DEBUG
#define MS_DEBUGGING_STD "mqttPublisher"
#endif

/**
 * @brief The MQTT User Name
 *
 */
#ifndef MQTT_USER_NAME
#define MQTT_USER_NAME ""
#endif

/**
 * @brief MQTT password that goes with the username
 *  
 */
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD ""
#endif

/**
 * @brief The MQTT Client Name
 *
 */
#ifndef MQTT_CLIENT_NAME
#define MQTT_CLIENT_NAME "MS"
#endif

/**
 * @brief MQTT topic string to publish to
 * 
 */
#ifndef MQTT_TOPIC_STRING
#define MQTT_TOPIC_STRING "topic"
#endif

// Included Dependencies
#include "ModSensorDebugger.h"
#undef MS_DEBUGGING_STD
#include "dataPublisherBase.h"
#include "PubSubClient.h"


// ============================================================================
//  Functions for the EnviroDIY data portal receivers.
// ============================================================================
/**
 * @brief The mqttPublisher subclass of dataPublisher for publishing data
 * to ThingSpeak using the MQTT protocol.
 *
 * When sending data to an MQTT broker the order of the variables in the variable
 * array attached to your logger is __crucial__.  The results from the variables
 * in the VariableArray will be sent to ThingSpeak in the order they are in the
 * array; that is, the first variable in the array will be sent as Field1, the
 * second as Field2, etc.  Any UUID's or custom variable codes are ignored for
 * ThingSpeak.  They will only appear in the header of your file on the SD card.
 * Giving a variable a custom variable code like "Field3" will **NOT** make that
 * variable field 3 on ThingSpeak.  The third variable in the array will always
 * be "Field3".  Any text names you have given to your fields in ThingSpeak are
 * also irrelevant.
 *
 * @ingroup the_publishers
 */
class mqttPublisher : public dataPublisher {
 public:
    // Constructors
    /**
     * @brief Construct a new MQTT Publisher object with no members
     * initialized.
     */
    mqttPublisher();
    /**
     * @brief Construct a new Publisher object
     *
     * @note If a client is never specified, the publisher will attempt to
     * create and use a client on a LoggerModem instance tied to the attached
     * logger.
     *
     * @param baseLogger The logger supplying the data to be published
     * @param sendEveryX Interval (in units of the logging interval) between
     * attempted data transmissions. NOTE: not implemented by this publisher!
     */
    explicit mqttPublisher(Logger& baseLogger, int sendEveryX = 1);
    /**
     * @brief Construct a new Publisher object
     *
     * @param baseLogger The logger supplying the data to be published
     * @param inClient An Arduino client instance to use to print data to.
     * Allows the use of any type of client and multiple clients tied to a
     * single TinyGSM modem instance
     * @param sendEveryX Interval (in units of the logging interval) between
     * attempted data transmissions. NOTE: not implemented by this publisher!
     */
    mqttPublisher(Logger& baseLogger, Client* inClient,
                        int sendEveryX = 1);

   /**
     * @brief Construct a new mqtt Publisher object
     *
     * @param baseLogger The logger supplying the data to be published
     * @param MQTTtopic the topic string for organizing the MQTT message
     * @param sendEveryX Interval (in units of the logging interval) between
     * attempted data transmissions. NOTE: not implemented by this publisher!
     */
    mqttPublisher(Logger& baseLogger, 
      const char* MQTTtopic,
      int sendEveryX = 1);

   /**
     * @brief Construct a new mqtt Publisher object
     *
     * @param baseLogger The logger supplying the data to be published
     * @param inClient An Arduino client instance to use to print data to.
     * Allows the use of any type of client and multiple clients tied to a
     * single TinyGSM modem instance
     * @param MQTTtopic the topic string for organizing the MQTT message
     * @param sendEveryX Interval (in units of the logging interval) between
     * attempted data transmissions. NOTE: not implemented by this publisher!
     */
    mqttPublisher(Logger& baseLogger, Client* inClient,
      const char* MQTTtopic,
      int sendEveryX = 1);


   /**
     * @brief Construct a new mqtt Publisher object
     *
     * @param baseLogger The logger supplying the data to be published
     * @param MQTTtopic the topic string for organizing the MQTT message
     * @param MQTTuser the MQTT username (if required by broker)
     * @param MQTTpw Your MQTT password (if required by broker)
     * @param sendEveryX Interval (in units of the logging interval) between
     * attempted data transmissions. NOTE: not implemented by this publisher!
     */
   mqttPublisher(Logger& baseLogger, 
      const char* MQTTtopic,
      const char* MQTTuser,
      const char* MQTTpw,
      int sendEveryX = 1);
   /**
    * @brief Construct a new mqtt Publisher object
    *
    * @param baseLogger The logger supplying the data to be published
    * @param inClient An Arduino client instance to use to print data to.
    * Allows the use of any type of client and multiple clients tied to a
    * single TinyGSM modem instance
    * @param MQTTtopic the topic string for organizing the MQTT message
    * @param MQTTuser the MQTT username (if required by broker)
    * @param MQTTpw Your MQTT password (if required by broker)
    * @param sendEveryX Interval (in units of the logging interval) between
    * attempted data transmissions. NOTE: not implemented by this publisher!
    */
   mqttPublisher(Logger& baseLogger, Client* inClient,
      const char* MQTTtopic,
      const char* MQTTuser,
      const char* MQTTpw,
      int sendEveryX = 1);

    /**
     * @brief Destroy the MQTT Publisher object
     */
    virtual ~mqttPublisher();

    // Returns the data destination
    String getEndpoint(void) override {
        return String(mqttServer);
    }

    /**
     * @brief Set the MQTT topic
     *
     * @param MQTTtopic Your MQTT topic string
     */
    void setMQTTtopic(const char* MQTTtopic);

    /**
     * @brief Set the MQTT user (if required by broker)
     *
     * @param MQTTuser Your MQTT username
     */
    void setMQTTuser(const char* MQTTuser);

    /**
     * @brief Set the MQTT password (if required by broker)
     *
     * @param MQTTpw Your MQTT password.
     */
    void setMQTTpw(const char* MQTTpw);

    // A way to begin with everything (credentials/client) already set
    /**
     * @copydoc dataPublisher::begin(Logger& baseLogger, Client* inClient)
     * @param MQTTtopic Your MQTT topic string
     */
    void begin(Logger& baseLogger, Client* inClient,
      const char* MQTTtopic);
    /**
     * @copydoc dataPublisher::begin(Logger& baseLogger)
     * @param MQTTtopic Your MQTT topic string
     */
    void begin(Logger& baseLogger, const char* MQTTtopic);

    // A way to begin with everything (credentials/client) already set
    /**
     * @copydoc dataPublisher::begin(Logger& baseLogger, Client* inClient)
     * @param MQTTtopic Your MQTT topic string
     * @param MQTTuser Your MQTT username
     * @param MQTTpw Your MQTT password
     */
    void begin(Logger& baseLogger, Client* inClient,
      const char* MQTTtopic,
      const char* MQTTuser,
      const char* MQTTpw);
    /**
     * @copydoc dataPublisher::begin(Logger& baseLogger)
     * @param MQTTtopic Your MQTT topic string
     * @param MQTTuser Your MQTT username
     * @param MQTTpw Your MQTT password
     */
    void begin(Logger& baseLogger, 
      const char* MQTTtopic,
      const char* MQTTuser,
      const char* MQTTpw);

    // This sends the data to MQTT
    // bool mqttMQTT(void);
    int16_t publishData(Client* outClient) override;

 protected:
    /**
     * @anchor mqtt_vars
     * @name Portions of the MQTT data publication
     *
     * @{
     */
    static const char* mqttServer;      ///< The MQTT server
    static const int   mqttPort;        ///< The MQTT port
    static const char* mqttClientName;  ///< The MQTT client name
    static const char* mqttTopic;       ///< The MQTT topic string
                                        /**@}*/

 private:
    // for MQTT
    const char* _MQTTuser = nullptr;
    const char* _MQTTpw = nullptr;
    const char* _MQTTclient = nullptr;
    PubSubClient _mqttClient;
};

#endif  // SRC_PUBLISHERS_MQTTPUBLISHER_H_
