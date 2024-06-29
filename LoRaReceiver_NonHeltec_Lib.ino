/*===============================================*/
/*========SELECT BOARD: HELTEC WIFI LORA V3======*/
/*========FROM UNOFFICIAL HELTEC LIBRARIES=======*/
/*===============================================*/
#include <heltec_unofficial.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "";
const char* password = "";
const char* mqtt_server = "";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";

WiFiClient espClient;
PubSubClient client(espClient);

#define BUFFER_SIZE 128 
char rxpacket[BUFFER_SIZE];
bool newPacketAvailable = false;

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    client.setServer(mqtt_server, mqtt_port);

    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("Initializing SX1262 failed, code "));
        Serial.println(state);
        while (true); // halt on error
    }

    radio.setFrequency(915.0);
    radio.setOutputPower(22); 
    radio.setSpreadingFactor(8);
    radio.setBandwidth(125.0);
    radio.setCodingRate(6);
    //radio.setCRC(1);  // Enable CRC
    Serial.println("LoRa initialized successfully!");
    radio.startReceive(); 
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

if (radio.available()) {
    int state = radio.readData(reinterpret_cast<uint8_t*>(rxpacket), BUFFER_SIZE);
    if (state == RADIOLIB_ERR_NONE) {
        rxpacket[radio.getPacketLength()] = '\0'; // Properly terminate the string
        newPacketAvailable = true;
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println("CRC error in received packet");
    }
    radio.startReceive(); // Reset the receiver
}

    if (newPacketAvailable) {
        processIncomingPacket(rxpacket, strlen(rxpacket)); // Process packet based on its actual length
        newPacketAvailable = false;
        memset(rxpacket, 0, BUFFER_SIZE); // Clear the buffer after processing
    }
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
//update Device Name to your device, if using multiple receivers, use unique names
        if (client.connect("Device Name", mqtt_user, mqtt_password)) {
            Serial.println("Connected to MQTT broker");
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" trying again in 5 seconds");
            delay(5000);
        }
    }
}

void processIncomingPacket(char* packet, int length) {
    Serial.printf("Received LoRa packet: \"%s\" Length: %d\n", packet, length);
    String message(packet);
    int delimiterIndex = message.indexOf(',');
    if (delimiterIndex != -1) {
        String sensorName = message.substring(0, delimiterIndex);
        sensorName.trim();
        sensorName.toLowerCase();
//update this to your topic
        String topic = "example/topic" + sensorName;
        String payload = message.substring(delimiterIndex + 1);
        payload += ", RSSI: " + String(radio.getRSSI());
        if (client.publish(topic.c_str(), payload.c_str())) {
            Serial.print("Published to MQTT topic ");
            Serial.println(topic);
        } else {
            Serial.println("MQTT publish failed");
        }
    } else {
        Serial.println("Sensor name delimiter not found in message");
    }
}
