#include <heltec_unofficial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

#define ONE_WIRE_BUS 7
#define MOISTURE_SENSOR_PIN 2
#define DHTPIN 45  // DHT data pin
#define DHTTYPE DHT22  // DHT 22

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  heltec_setup();  // Initialize the Heltec module
  Serial.begin(115200);

  // Enable internal pull-up resistor on the DS18B20 data line
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);

  sensors.begin();
  dht.begin();
  pinMode(MOISTURE_SENSOR_PIN, INPUT);

  // Initialize SX1262 with the specific settings
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
}

void loop(void) {
  heltec_loop();
  heltec_ve(true);  // Power on the sensors
  delay(2000);  // Allow some time for sensors to stabilize

  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  float tempF = tempC * 9.0 / 5.0 + 32.0;
  float humidity = dht.readHumidity();
  float tempDHT = dht.readTemperature(true);

  int rawMoistureLevel = analogRead(MOISTURE_SENSOR_PIN);
  // Serial.print("Raw Moisture Level: "); //Uncomment these two lines
  // Serial.println(rawMoistureLevel);  // Print the raw value to the serial monitor
  // Comment out everything past here except the last curly brace for moisture testing
  int moisturePercentage = map(rawMoistureLevel, 1891, 3600, 100, 0);
  moisturePercentage = constrain(moisturePercentage, 0, 100);

  heltec_ve(false);  // Power off the sensors to conserve battery

  float batteryVoltage = heltec_vbat();  // Get battery voltage using Heltec function
  int batteryPercent = heltec_battery_percent();  // Get battery percentage

  // Create the data packet with all sensor values
  String dataPacket = "vines_2," + String(tempF) + "," + String(tempDHT) + "," + String(humidity) + "," + String(moisturePercentage) + "," + String(batteryVoltage) + "," + String(batteryPercent);

  // Transmit the data packet
  int transmissionState = radio.transmit(dataPacket);
  if (transmissionState == RADIOLIB_ERR_NONE) {
    Serial.println("Success: Message sent!");
  } else {
    Serial.print("Error during transmission, code: ");
    Serial.println(transmissionState);
  }

  Serial.println(dataPacket);

  heltec_ve(false);
  // Turn off LED
  heltec_led(0);
  // Set all pins to input to save power
  pinMode(VBAT_CTRL, INPUT);
  pinMode(VBAT_ADC, INPUT);
  pinMode(DIO1, INPUT);
  pinMode(RST_LoRa, INPUT);
  pinMode(BUSY_LoRa, INPUT);
  pinMode(SS, INPUT);
  pinMode(MISO, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SDA_OLED, INPUT);
  pinMode(SCL_OLED, INPUT);
  pinMode(RST_OLED, INPUT);
  // Set button wakeup if applicable
    // Enter deep sleep for 30 minutes
  unsigned long sleepTime = 3600000; // 30 minutes in milliseconds
  esp_sleep_enable_timer_wakeup(sleepTime * 1000ULL); // Convert to microseconds
  esp_deep_sleep_start();
}