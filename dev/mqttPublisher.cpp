/**
 * @file mqttPublisher.cpp
 * @copyright Stroud Water Research Center
 * Part of the EnviroDIY ModularSensors library for Arduino.
 * This library is published under the BSD-3 license.
 * @author Ryan Cole
 *
 * @brief Implements the mqttPublisher class.
 */

#include "mqttPublisher.h"


// ============================================================================
//  Functions for the EnviroDIY data portal receivers.
// ============================================================================

// Constant values for MQTT publish
// I want to refer to these more than once while ensuring there is only one copy
// in memory
const char* mqttPublisher::mqttServer      = "madiebookpro.local";
const int   mqttPublisher::mqttPort        = 1883;
const char* mqttPublisher::mqttClientName  = MQTT_CLIENT_NAME;
const char* mqttPublisher::mqttTopic       = MQTT_TOPIC_STRING;

//const char* mqttPublisher::mqttUser        = MQTT_USER_NAME;
//const char* mqttPublisher::mqttPassword    = MQTT_PASSWORD;



// Constructors
mqttPublisher::mqttPublisher() : dataPublisher() {}

mqttPublisher::mqttPublisher(Logger& baseLogger, int sendEveryX)
    : dataPublisher(baseLogger, sendEveryX) {}

mqttPublisher::mqttPublisher(Logger& baseLogger, Client* inClient,
                                         int sendEveryX)
    : dataPublisher(baseLogger, inClient, sendEveryX) {}

mqttPublisher::mqttPublisher(Logger& baseLogger, 
    const char* MQTTtopic,
    int sendEveryX)
    : dataPublisher(baseLogger, sendEveryX) {
        setMQTTtopic(MQTTtopic);
    }

mqttPublisher::mqttPublisher(Logger& baseLogger, Client* inClient,
    const char* MQTTtopic,
    int sendEveryX)
    : dataPublisher(baseLogger, inClient, sendEveryX) {
        setMQTTtopic(MQTTtopic);
    }


mqttPublisher::mqttPublisher(Logger& baseLogger,
    const char* MQTTtopic,
    const char* MQTTuser, 
    const char* MQTTpw,
    int         sendEveryX)
    : dataPublisher(baseLogger, sendEveryX) {
        setMQTTtopic(MQTTtopic);
        setMQTTuser(MQTTuser);
        setMQTTpw(MQTTpw);
}

mqttPublisher::mqttPublisher(Logger& baseLogger, Client* inClient,
    const char* MQTTtopic,
    const char* MQTTuser,
    const char* MQTTpw,
    int         sendEveryX)
    : dataPublisher(baseLogger, inClient, sendEveryX) {
        setMQTTtopic(MQTTtopic);
        setMQTTuser(MQTTuser);
        setMQTTpw(MQTTpw);
}


// Destructor
mqttPublisher::~mqttPublisher() {}

// Setter for MQTT topic
void mqttPublisher::setMQTTtopic(const char* MQTTtopic) {
    mqttTopic = MQTTtopic;
}
// Setter for MQTT user
void mqttPublisher::setMQTTuser(const char* MQTTuser) {
    _MQTTuser = MQTTuser;
}
// Setter for MQTT password
void mqttPublisher::setMQTTpw(const char* MQTTpw) {
    _MQTTpw = MQTTpw;
}

// Begin with only topic set
void mqttPublisher::begin(Logger& baseLogger, Client* inClient,
    const char* MQTTtopic) {
    setMQTTtopic(MQTTtopic);
    dataPublisher::begin(baseLogger, inClient);
}

void mqttPublisher::begin(Logger& baseLogger,
    const char* MQTTtopic) {
    setMQTTtopic(MQTTtopic);
    dataPublisher::begin(baseLogger);
}


// Begin with everything set
void mqttPublisher::begin(Logger& baseLogger, Client* inClient,
    const char* MQTTtopic,
    const char* MQTTuser,
    const char* MQTTpw) {
        setMQTTtopic(MQTTtopic);
        setMQTTuser(MQTTuser);
        setMQTTpw(MQTTpw);
    dataPublisher::begin(baseLogger, inClient);
}

void mqttPublisher::begin(Logger& baseLogger,
    const char* MQTTtopic,
    const char* MQTTuser,
    const char* MQTTpw) {
        setMQTTtopic(MQTTtopic);
        setMQTTuser(MQTTuser);
        setMQTTpw(MQTTpw);
    dataPublisher::begin(baseLogger);
}



// This sends the data via MQTT
int16_t mqttPublisher::publishData(Client* outClient) {
    bool retVal = false;

    // Make sure we don't have too many fields
    // A channel can have a max of 8 fields
    if (_baseLogger->getArrayVarCount() > 8) {
        MS_DBG(F("No more than 8 fields of data can be sent to a single "
                 "MQTT channel!"));
        MS_DBG(F("Only the first 8 fields worth of data will be sent."));
    }
    uint8_t numChannels = min(_baseLogger->getArrayVarCount(), 8);
    MS_DBG(numChannels, F("fields will be sent via MQTT"));

    // Create a buffer for the portions of the request and response
    char tempBuffer[26] = "";

/*
    char topicBuffer[42] = ;
    MS_DBG(F("Topic ["), strlen(topicBuffer), F("]:"), String(topicBuffer));
*/

    // buffer is used only locally, it does not transmit
    txBufferInit(nullptr);

    txBufferAppend("created_at=");
    txBufferAppend(
        Logger::formatDateTime_ISO8601(Logger::markedLocalEpochTime).c_str());

    for (uint8_t i = 0; i < numChannels; i++) {
        txBufferAppend("&field");
        itoa(i + 1, tempBuffer, 10);  // BASE 10
        txBufferAppend(tempBuffer);
        txBufferAppend('=');
        txBufferAppend(_baseLogger->getValueStringAtI(i).c_str());
    }
    MS_DBG(F("Message ["), strlen(txBuffer), F("]:"), String(txBuffer));


    // Definitely need the below since in makes connection to client and sends data

    // Set the client connection parameters
    _mqttClient.setClient(*outClient);
    _mqttClient.setServer(mqttServer, mqttPort);

    // Make sure any previous TCP connections are closed
    // NOTE:  The PubSubClient library used for MQTT connect assumes that as
    // long as the client is connected, it must be connected to the right place.
    // Closing any stray client sockets here ensures that a new client socket
    // is opened to the right place.
    // client is connected when a different socket is open
    if (outClient->connected()) { outClient->stop(); }

    // Make the MQTT connection
    MS_DBG(F("Opening MQTT Connection"));
    MS_START_DEBUG_TIMER;
    if (_mqttClient.connect(mqttClientName, _MQTTuser, _MQTTpw)) {
        MS_DBG(F("MQTT connected after"), MS_PRINT_DEBUG_TIMER, F("ms"));

        if (_mqttClient.publish(mqttTopic, txBuffer)) {
            PRINTOUT(F("MQTT topic published!  Current state:"),
                     parseMQTTState(_mqttClient.state()));
            retVal = true;
        } else {
            PRINTOUT(F("MQTT publish failed with state:"),
                     parseMQTTState(_mqttClient.state()));
            retVal = false;
        }
    } else {
        PRINTOUT(F("MQTT connection failed with state:"),
                 parseMQTTState(_mqttClient.state()));
        delay(1000);
        retVal = false;
    }

    // Disconnect from MQTT
    MS_DBG(F("Disconnecting from MQTT"));
    MS_RESET_DEBUG_TIMER
    _mqttClient.disconnect();
    MS_DBG(F("Disconnected after"), MS_PRINT_DEBUG_TIMER, F("ms"));
    return retVal;
}
