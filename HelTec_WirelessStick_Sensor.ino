/*===============================================*/
/*====SELECT BOARD: HELTEC WIRELESS STICK V3=====*/
/*========FROM UNOFFICIAL HELTEC LIBRARIES=======*/
/*===============================================*/
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RadioLib.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define ONE_WIRE_BUS 47
#define MOISTURE_SENSOR_PIN 19
#define BATTERY_PIN 1 // ADC1_CH0 Read battery voltage
#define DHTPIN 48  // DHT21 data pin
#define DHTTYPE DHT21  // DHT 21

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DHT dht(DHTPIN, DHTTYPE);

// Define the pins for the SX1262 module
#define NSS_PIN 8   // Chip Select
#define DIO1_PIN 14 // DIO1 pin used for both TX and RX done
#define RST_PIN 12  // Reset pin
#define BUSY_PIN 13 // Busy pin

SX1262 radio = new Module(NSS_PIN, DIO1_PIN, RST_PIN, BUSY_PIN);
// Initialize with the default I2C pins
Adafruit_ADS1115 ads; // Default I2C address

void setup() {
  Wire.begin(19, 20); // Custom SDA and SCL pins
  Serial.begin(115200);

  if (!ads.begin(0x48)) {  // Initialize with I2C address if it's not the default
    Serial.println("Failed to initialize ADS1115");
    while (1); // halt on error
  }
  Serial.println("ADS1115 initialized.");

  Serial.begin(115200);
  sensors.begin();
  dht.begin();
  pinMode(BATTERY_PIN, INPUT);

  // Optional: Set gain
  ads.setGain(GAIN_ONE);  // 1x gain +/- 4.096V  1 bit = 2mV (default)
  Serial.println("ADS1115 initialized.");

  // Initialize SX1262 with the specific settings
  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Initializing SX1262 failed, code "));
    Serial.println(state);
    while (true); // halt on error
  }

  radio.setFrequency(915.0);
  radio.setOutputPower(22);
  radio.setSpreadingFactor(7);
  radio.setBandwidth(125.0);
  radio.setCodingRate(5);
}

void loop(void) {
  // Request temperatures from DS18B20 sensor
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  float tempF = tempC * 9.0 / 5.0 + 32.0;  // Convert to Fahrenheit

  // Read temperature and humidity from DHT11 sensor
  float humidity = dht.readHumidity();
  float tempDHT = dht.readTemperature(true);  // Read temperature as Fahrenheit (true)

  // Delay for stabilization
  delay(1000);  // Delay of 1 second for moisture sensor stabilization

  // Read moisture level using ADS1115
  int16_t rawMoistureLevel = ads.readADC_SingleEnded(0);  // Assuming the sensor is connected to A0
  Serial.print("Raw Moisture Sensor Data from ADS1115: ");
  Serial.println(rawMoistureLevel);

  // You might want to map and scale this value as needed
  int moisturePercentage = map(rawMoistureLevel, 13138, 21290, 0, 100); // Adjust mapping as per your calibration
  moisturePercentage = constrain(moisturePercentage, 0, 100);

  // Read the battery voltage
  int batteryValue = analogRead(BATTERY_PIN);
  float VADC_IN1 = batteryValue * (3.3 / 4095.0); // Assuming 3.3V as ADC reference and 12-bit ADC resolution
  float batteryVoltage = (100.0 / (100.0 + 390.0)) * VADC_IN1; // Apply the voltage divider formula

  // Create the data packet with all sensor values
  String dataPacket = "vines_2, DS18B20 Temp: " + String(tempF) + ", DHT11 Temp: " + String(tempDHT) + ", Humidity: " + String(humidity) + ", Moisture: " + String(moisturePercentage) + ", Battery: " + String(batteryVoltage, 2);

  // Transmit the data packet
  int transmissionState = radio.transmit(dataPacket);
  if (transmissionState == RADIOLIB_ERR_NONE) {
    Serial.println("Success: Message sent!");
  } else {
    Serial.print("Error during transmission, code: ");
    Serial.println(transmissionState);
  }

  // Print the data packet to the serial console
  Serial.println(dataPacket);
 // Enter deep sleep for 30 seconds test
 // esp_sleep_enable_timer_wakeup(30000000);
 
   // Enter deep sleep for 10 minutes (600000000 microseconds)
  unsigned long sleepTime = 30000000; // 1 hour in milliseconds
  esp_sleep_enable_timer_wakeup(sleepTime); // Convert to microseconds
  esp_deep_sleep_start();
}
