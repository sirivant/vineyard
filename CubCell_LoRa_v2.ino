/*===============================================*/
/*======SELECT BOARD: CUBECELL HTCC-AB01-V2======*/
/*==========FROM CUBECELL DEV FRAMEWORK==========*/
/*===============================================*/
#include "LoRaWan_APP.h"
#include "Arduino.h"

// LoRa parameters
#define RF_FREQUENCY 915000000  // 915 MHz
#define TX_OUTPUT_POWER 14      // dBm
#define LORA_BANDWIDTH 0        // [0: 125 kHz]
#define LORA_SPREADING_FACTOR 7 // [SF7]
#define LORA_CODINGRATE 5       // [4/8]
#define LORA_PREAMBLE_LENGTH 8  // Preamble length
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

unsigned long messageCount = 0;  // Message counter as a unique identifier
unsigned long lastSendTime = 0;  // Last send time
unsigned long sendInterval = 180000;  // Send interval in milliseconds (3 minutes)

void setup() {
  boardInitMcu();
  Serial.begin(115200);
  
  // Initialize LoRa
  Radio.Init(NULL);
  Radio.SetChannel(RF_FREQUENCY);

  // Configure LoRa transmitter
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  // Set the LoRa sync word
  Radio.SetSyncWord(0x12);

  Serial.println("LoRa radio is initialized.");

  // Send a boot status message
  String bootMessage = "Device:vineyard-east,Status:Online,ID:" + String(messageCount++);
  Radio.Send((uint8_t *)bootMessage.c_str(), bootMessage.length());
  Serial.println("Boot status message sent: " + bootMessage);

  lastSendTime = millis();  // Initialize last send time
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if it's time to send a message
  if (currentMillis - lastSendTime >= sendInterval) {
    // Create a simple message with the device name, "Hello World", and an ID
    String message = "Device:vineyard-east,Message:Hello World,ID:" + String(messageCount++);

    // Send the message over LoRa
    Radio.Send((uint8_t *)message.c_str(), message.length());

    Serial.println("Message sent: " + message);

    lastSendTime = currentMillis;  // Update the last send time
  }

  // Other non-blocking code can be placed here
}
