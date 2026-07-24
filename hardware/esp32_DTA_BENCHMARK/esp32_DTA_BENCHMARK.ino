/*
 * Copyright 2024
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ESP32 MQTT/UART bridge for the Pololu DTA benchmark.
 *
 * This keeps the same topic/frame style as the older low-comm-search bridge:
 *   MQTT topic:  <robot_id><topic_digit>, for example 001 or 023
 *   UART frame:  <sender_id><topic_digit>.<payload>-
 *
 * Topic digits:
 *   1 = position/state
 *   2 = next-step collision intent
 *   3 = primary allocator-specific payload / goal-style payload
 *   4 = clue reports
 *   5 = target alerts
 *   6 = secondary allocator-specific payload / sync-style payload
 *   7 = hub command forwarded to robot from hub/command
 *
 * The bridge is payload-agnostic. DMCHBA can simply ignore allocator-specific
 * payload topics. ACBBA/CBAA/PI/HIPC can use topic 3 and/or 6 without changing
 * the ESP32 bridge.
 */

#include <WiFi.h>
#include "ESP32MQTTClient.h"
#include <vector>
#include <string>
#include <assert.h>

// Wi-Fi network credentials
const char *ssid = "USDresearch";
const char *pass = "USDresearch";

// MQTT broker configuration
const char *server = "mqtt://192.168.1.10:1883"; // MQTT server URI

// Robot-specific MQTT topics
String clientID = "00";                            // unique robot ID; edit per robot
String pubpositiontopic = clientID + "1";          // current position/state
String pubintenttopic = clientID + "2";            // next intended cell
String pubgoaltopic = clientID + "3";              // primary allocator-specific payload
String pubcluetopic = clientID + "4";              // clue reports
String pubtargettopic = clientID + "5";            // target alerts
String pubsyncstatetopic = clientID + "6";         // secondary allocator-specific payload
const char *hubCommandTopic = "hub/command";       // shared hub command topic
std::vector<String> otherIDs = {"01", "03", "02"}; // edit per robot
const char *lastWillMessage = "disconnected";      // Last Will message

ESP32MQTTClient mqttClient; // MQTT client object

// UART configuration
#define RXD2 16  // UART RX pin
#define TXD2 17  // UART TX pin
HardwareSerial robotSerial(2); // UART2 for communication with the Pololu

// Track reconnection attempts to throttle retries
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 4000; // check every 4 seconds

void frameToRobot(char topicDigit, const String &senderID, const String &payload);
void handlemsg(String line);
void sendtoMQTT(String topic, String msg);

std::string toStdString(const String &value)
{
    return std::string(value.c_str());
}

void publishString(const String &topic, const String &payload)
{
    mqttClient.publish(toStdString(topic), toStdString(payload), 0, false);
}

void subscribePeerTopic(const String &peer, char topicDigit)
{
    String topic = peer + topicDigit;

    // ESP32MQTTClient.h in this install expects std::string callbacks, not Arduino String callbacks.
    mqttClient.subscribe(toStdString(topic), [topicDigit, peer](const std::string &payload) {
        frameToRobot(topicDigit, peer, String(payload.c_str()));
    });
}

void subscribeAllTopics()
{
    // Subscribe to MQTT topics for each peer
    for (const String &peer : otherIDs)
    {
        for (char topicDigit = '1'; topicDigit <= '6'; ++topicDigit)
        {
            subscribePeerTopic(peer, topicDigit);
        }
    }

    // Subscribe to hub command topic
    mqttClient.subscribe(std::string(hubCommandTopic), [](const std::string &payload) {
        String hub = "99";
        frameToRobot('7', hub, String(payload.c_str()));
    });
}

void onMqttConnect(esp_mqtt_client_handle_t client)
{
    if (mqttClient.isMyTurn(client))
    {
        mqttClient.publish(toStdString(pubpositiontopic), std::string("connected"), 0, false);
        subscribeAllTopics();
    }
}

void setup()
{
    // Initialize serial ports for robot communication
    Serial.begin(115200);
    robotSerial.setRxBufferSize(1024);  // safe headroom for allocator payload frames
    robotSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

    // Connect to Wi-Fi
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }

    // Configure the MQTT client
    mqttClient.setURI(server);
    mqttClient.enableLastWillMessage(pubpositiontopic.c_str(), lastWillMessage); // set Last Will message
    mqttClient.setKeepAlive(3);                                                  // 3-second keep-alive timeout

    // Start the MQTT loop
    mqttClient.loopStart();
}

void loop()
{
    // Ensure Wi-Fi is connected
    if (WiFi.status() != WL_CONNECTED)
    {
        WiFi.begin(ssid, pass);
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(3000);
        }
    }

    // Ensure MQTT connection
    if (!mqttClient.isConnected())
    {
        unsigned long currentMillis = millis();
        if (currentMillis - lastReconnectAttempt > reconnectInterval)
        {
            lastReconnectAttempt = currentMillis;
            mqttClient.publish(toStdString(pubpositiontopic), std::string("Reconnected"), 0, false);
            subscribeAllTopics();
        }
    }

    static String serialBuffer = "";

    // Check for responses from the Pololu robot
    while (robotSerial.available())
    {
        char c = robotSerial.read();
        serialBuffer += c;

        if (c == '-')
        {
            // Full message received
            serialBuffer.trim(); // remove any unwanted whitespace

            // Remove trailing '-' and process
            String full_msg = serialBuffer.substring(0, serialBuffer.length() - 1);
            serialBuffer = "";
            handlemsg(full_msg); // publish message to proper topic
        }
    }

    delay(1); // Short delay to prevent busy looping
}

void handlemsg(String line)
{
    int divider = line.indexOf('.'); // position of the divider in the string
    assert(divider != -1);

    String topic = line.substring(0, divider);
    String message = line.substring(divider + 1);
    sendtoMQTT(topic, message);
}

void frameToRobot(char topicDigit, const String &senderID, const String &payload)
{
    // Keep a single '-' terminator end-to-end; avoid adding a second one.
    bool hasTerminator = payload.length() && payload.charAt(payload.length() - 1) == '-';

    robotSerial.print(senderID);   // "00", "01", ... or "99" for hub
    robotSerial.print(topicDigit); // '1'..'7'
    robotSerial.print('.');
    robotSerial.print(payload);    // payload may already end with '-'
    if (!hasTerminator)
    {
        robotSerial.print('-');
    }
}

void sendtoMQTT(String topic, String msg)
{
    if (topic == "5")
    {
        publishString(pubtargettopic, msg);
    }
    else if (topic == "1")
    {
        publishString(pubpositiontopic, msg);
    }
    else if (topic == "4")
    {
        publishString(pubcluetopic, msg);
    }
    else if (topic == "2")
    {
        publishString(pubintenttopic, msg);
    }
    else if (topic == "3")
    {
        publishString(pubgoaltopic, msg);
    }
    else if (topic == "6")
    {
        publishString(pubsyncstatetopic, msg);
    }
}

void handleMQTT(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    auto *event = static_cast<esp_mqtt_event_handle_t>(event_data);
    mqttClient.onEventCallback(event); // Pass events to the client
}
