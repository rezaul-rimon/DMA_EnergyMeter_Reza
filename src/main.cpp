#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>  // WiFiManager library
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <SD.h>
// #include <FastLED.h>

// Device Config
#define DEVICE_ID 1102002309180001
#define HB_INTERVAL 1*30*1000
#define DATA_INTERVAL 1*60*1000

unsigned long hbLastTime = 0, dataLastTime = 0;
boolean sd_status = false;

// RS485 Config
#define MAX485_DE_RE 27
#define RS485_RX 16
#define RS485_TX 17

// Modbus instance
ModbusMaster node;

void preTransmission() {
  digitalWrite(MAX485_DE_RE, HIGH); // Enable Transmit mode
}

void postTransmission() {
  digitalWrite(MAX485_DE_RE, LOW); // Enable Receive mode
}


// Function to read a specific Modbus register
int readModbusData(uint16_t reg_address) {
  uint8_t result = node.readHoldingRegisters(reg_address, 1);
  if (result == node.ku8MBSuccess) {
    return node.getResponseBuffer(0);
  } else {
    return -1; // Error value
  }
}


// Modbus register addresses
#define taeHigh_reg_addr 0x30
#define taeLow_reg_addr 0x31
#define activePower_reg_addr 0x1A
#define pAvolt_reg_addr 0x14
#define pBvolt_reg_addr 0x15
#define pCvolt_reg_addr 0x16
#define lABvolt_reg_addr 0x17
#define lBCvolt_reg_addr 0x18
#define lCAvolt_reg_addr 0x19
#define pAcurrent_reg_addr 0x10
#define pBcurrent_reg_addr 0x11
#define pCcurrent_reg_addr 0x12
#define frequency_reg_addr 0x1E
#define powerfactor_reg_addr 0x1D

// Variables for storing Modbus data
int taeHigh, taeLow, activePower;
int pAvolt, pBvolt, pCvolt;
int lABvolt, lBCvolt, lCAvolt;
int pAcurrent, pBcurrent, pCcurrent;
int frequency, powerFactor;

// SD Card Configuration
#define SD_CS_PIN 5
const char* filename = "/energy_data.csv";


// LED Conffig
// #define DATA_PIN 4
// #define NUM_LEDS 1
// CRGB leds[NUM_LEDS];

// MQTT Config
const char* mqtt_server = "broker2.dma-bd.com";
const char* mqtt_user = "broker2";
const char* mqtt_password = "Secret!@#$1234";

// WiFi and MQTT connection settings
#define WIFI_ATTEMPT_COUNT 30
#define WIFI_ATTEMPT_DELAY 1000
#define WIFI_WAIT_COUNT 60
#define WIFI_WAIT_DELAY 1000
#define MAX_WIFI_ATTEMPTS 2
#define MQTT_ATTEMPT_COUNT 6
#define MQTT_ATTEMPT_DELAY 5000

// WiFi and MQTT attempt counters
int wifiAttemptCount = WIFI_ATTEMPT_COUNT;
int wifiWaitCount = WIFI_WAIT_COUNT;
int maxWifiAttempts = MAX_WIFI_ATTEMPTS;
int mqttAttemptCount = MQTT_ATTEMPT_COUNT;

WiFiClient espClient;
PubSubClient client(espClient);

TaskHandle_t networkTaskHandle;
TaskHandle_t mainTaskHandle;
TaskHandle_t wifiResetTaskHandle;

// Debug mode setting
#define DEBUG_MODE true
#define DEBUG_PRINT(x)  if (DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x) if (DEBUG_MODE) { Serial.println(x); }

// Button setup for WiFi reset
#define WIFI_RESET_BUTTON_PIN 35  // GPIO pin for reset button
bool wifiResetFlag = false;

// Function to manage LED status
/*
void manageLEDStatus() {
    CRGB color = CRGB::Black;
    if (WiFi.status() != WL_CONNECTED) {
        color = CRGB::Red;
    } else if (!client.connected()) {
        color = CRGB::Yellow;
    }
    leds[0] = color;
    FastLED.show();
}
*/

// MQTT callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    DEBUG_PRINTLN("Message arrived: " + message);
}

// Function to reconnect to MQTT with a unique client ID
void reconnectMQTT() {
  if (!client.connected()) {
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);  // Generate a unique client ID using a random hexadecimal string

    if (mqttAttemptCount > 0) {
      DEBUG_PRINTLN("Attempting MQTT connection...");
      
      if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {  // Use the unique client ID
        DEBUG_PRINTLN("MQTT connected");
        DEBUG_PRINTLN("Client_ID: " + String(clientId));
        client.subscribe("YOUR_TOPIC");
      } else {
        DEBUG_PRINTLN("MQTT connection failed");
        DEBUG_PRINTLN("Remaining MQTT attempts: " + String(mqttAttemptCount));
        mqttAttemptCount--;
        vTaskDelay(MQTT_ATTEMPT_DELAY / portTICK_PERIOD_MS);
      }
    } else {
      DEBUG_PRINTLN("Max MQTT attempts exceeded, restarting...");
      ESP.restart();
    }
  }
}

// Function to reconnect to WiFi
void reconnectWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiAttemptCount > 0) {
            DEBUG_PRINTLN("Attempting WiFi connection...");
            WiFi.begin();  // Use saved credentials
            wifiAttemptCount--;
            DEBUG_PRINTLN("Remaining WiFi attempts: " + String(wifiAttemptCount));
            vTaskDelay(WIFI_ATTEMPT_DELAY / portTICK_PERIOD_MS);
        } else if (wifiWaitCount > 0) {
            wifiWaitCount--;
            DEBUG_PRINTLN("WiFi wait... retrying in a moment");
            DEBUG_PRINTLN("Remaining WiFi wait time: " + String(wifiWaitCount) + " seconds");
            vTaskDelay(WIFI_WAIT_DELAY / portTICK_PERIOD_MS);
        } else {
            wifiAttemptCount = WIFI_ATTEMPT_COUNT;
            wifiWaitCount = WIFI_WAIT_COUNT;
            maxWifiAttempts--;
            if (maxWifiAttempts <= 0) {
                DEBUG_PRINTLN("Max WiFi attempt cycles exceeded, restarting...");
                ESP.restart();
            }
        }
    }
}

// Task for network and MQTT management
void networkTask(void *param) {
    WiFi.mode(WIFI_STA);
    WiFi.begin();

    for (;;) {
      if (WiFi.status() != WL_CONNECTED) {
        reconnectWiFi();
        }
      if (WiFi.status() == WL_CONNECTED && !client.connected()) {
        reconnectMQTT();
      }
      client.loop();
      // manageLEDStatus();
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Task for WiFi reset and WiFiManager setup
void wifiResetTask(void *param) {
  for (;;) {
    if (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
      // Suspend other tasks to avoid conflict
      // vTaskDelay(500 / portTICK_PERIOD_MS);  // Debounce Delay
      DEBUG_PRINTLN("Reset Button Pressed....");
      vTaskSuspend(networkTaskHandle);
      vTaskSuspend(mainTaskHandle);

      DEBUG_PRINTLN("Starting WiFiManager for new WiFi setup...");
      WiFiManager wifiManager;
      wifiManager.resetSettings();  // Clear previous settings
      wifiManager.autoConnect("DMA_EM_WiFi_Setup"); // Start AP for new configuration

      DEBUG_PRINTLN("New WiFi credentials set, restarting...");
      ESP.restart();
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);  // Check every 100 ms
  }
}

/*********************************************************************/
/*                                  main                             */
/*********************************************************************/

// Task for main loop (e.g., performing your application logic)
void mainTask(void *param) {
  for (;;) {
    /*
    // Read data from Modbus
    taeHigh = readModbusData(taeHigh_reg_addr);
    taeLow = readModbusData(taeLow_reg_addr);
    activePower = readModbusData(activePower_reg_addr);
    pAvolt = readModbusData(pAvolt_reg_addr);
    pBvolt = readModbusData(pBvolt_reg_addr);
    pCvolt = readModbusData(pCvolt_reg_addr);
    lABvolt = readModbusData(lABvolt_reg_addr);
    lBCvolt = readModbusData(lBCvolt_reg_addr);
    lCAvolt = readModbusData(lCAvolt_reg_addr);
    pAcurrent = readModbusData(pAcurrent_reg_addr);
    pBcurrent = readModbusData(pBcurrent_reg_addr);
    pCcurrent = readModbusData(pCcurrent_reg_addr);
    frequency = readModbusData(frequency_reg_addr);
    powerFactor = readModbusData(powerfactor_reg_addr);

    float taeHigha = taeHigh/10.0;
    float taeLowa = taeLow/10.0;

    String em_data = String(DEVICE_ID) + ",";
    em_data += String(taeHigha) + ",";
    em_data += String(taeLowa) + ",";
    em_data += String(activePower) + ",";
    em_data += String(pAvolt) + ",";
    em_data += String(pBvolt) + ",";
    em_data += String(pCvolt) + ",";
    em_data += String(lABvolt) + ",";
    em_data += String(lBCvolt) + ",";
    em_data += String(lCAvolt) + ",";
    em_data += String(pAcurrent) + ",";
    em_data += String(pBcurrent) + ",";
    em_data += String(pCcurrent) + ",";
    em_data += String(frequency) + ",";
    em_data += String(powerFactor);

    // Print results to Serial Monitor
    /*
    Serial.print("taeHigh: "); Serial.println(taeHigh);
    Serial.print("taeLow: "); Serial.println(taeLow);
    Serial.print("Active Power: "); Serial.println(activePower);
    Serial.print("Phase A Voltage: "); Serial.println(pAvolt);
    Serial.print("Phase B Voltage: "); Serial.println(pBvolt);
    Serial.print("Phase C Voltage: "); Serial.println(pCvolt);
    Serial.print("Line AB Voltage: "); Serial.println(lABvolt);
    Serial.print("Line BC Voltage: "); Serial.println(lBCvolt);
    Serial.print("Line CA Voltage: "); Serial.println(lCAvolt);
    Serial.print("Phase A Current: "); Serial.println(pAcurrent);
    Serial.print("Phase B Current: "); Serial.println(pBcurrent);
    Serial.print("Phase C Current: "); Serial.println(pCcurrent);
    Serial.print("Frequency: "); Serial.println(frequency);
    Serial.print("Power Factor: "); Serial.println(powerFactor);
    */

   
    unsigned long hbNow = millis();
    if(hbNow - hbLastTime > HB_INTERVAL){
      hbLastTime = hbNow;
      if (client.connected()) { 
        String hb_data = String(DEVICE_ID)+","+"wifi_connected,"+"SD_Card:"+String(sd_status);
        client.publish("DMA/EnergyMeter/PUB", hb_data.c_str());
        DEBUG_PRINTLN(hb_data);
        DEBUG_PRINTLN("Heartbeat published data to mqtt");

      } else {
        DEBUG_PRINTLN("Failed to publish Heartbeat on MQTT");
      }
    }
    
    /*
    unsigned long dataNow = millis();
    if(dataNow - dataLastTime > DATA_INTERVAL){
      dataLastTime = dataNow;

      // Write data to SD card
      File file = SD.open(filename, FILE_APPEND);
      if (file) {
        file.print(taeHigh); file.print(",");
        file.print(taeLow); file.print(",");
        file.print(activePower); file.print(",");
        file.print(pAvolt); file.print(",");
        file.print(pBvolt); file.print(",");
        file.print(pCvolt); file.print(",");
        file.print(lABvolt); file.print(",");
        file.print(lBCvolt); file.print(",");
        file.print(lCAvolt); file.print(",");
        file.print(pAcurrent); file.print(",");
        file.print(pBcurrent); file.print(",");
        file.print(pCcurrent); file.print(",");
        file.print(frequency); file.print(",");
        file.println(powerFactor);
        file.close();
        DEBUG_PRINTLN("Data written success to SD card.");
        sd_status = true;
      } else {
        DEBUG_PRINTLN("Error opening file for writing.");
        sd_status = false;
      }
////////////////////////////////////

      if (client.connected()) {
        DEBUG_PRINTLN(em_data);
        client.publish("DMA/EnergyMeter/PUB", em_data.c_str());
        DEBUG_PRINTLN("Data published data to mqtt");

    } else {
      DEBUG_PRINTLN("Failed to publish data on MQTT.")
    }
}
*/
      DEBUG_PRINTLN("Hello");
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Print "Hello" every second
  }
}

// Setup function
void setup() {
    Serial.begin(115200);
    // FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

/////////////////////////////////////
    // Initialize RS485 pins
    pinMode(MAX485_DE_RE, OUTPUT);
    digitalWrite(MAX485_DE_RE, LOW);

    // Initialize Serial2 for RS485 communication
    Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);

    // Initialize ModbusMaster instance
    node.begin(1, Serial2); // Slave ID 1
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

    // Initialize SD card
    if (!SD.begin(SD_CS_PIN)) {
      DEBUG_PRINTLN("SD Card initializing...");
      // while (1); // Halt execution if SD card initialization fails
      for (int i=10; i>0; i--) {
        SD.begin(SD_CS_PIN);
        delay(500);
        DEBUG_PRINTLN("Trying to initial SD Card: " + String(i));
      }
    } else {
      DEBUG_PRINTLN("SD card initialized.");
    }
    

    // Create file header if the file does not exist
    if (!SD.exists(filename)) {
      File file = SD.open(filename, FILE_WRITE);
      sd_status = true;
      if (file) {
        file.println("taeHigh,taeLow,ActivePower,PhaseA_V,PhaseB_V,PhaseC_V,LineAB_V,LineBC_V,LineCA_V,PhaseA_C,PhaseB_C,PhaseC_C,Frequency,PowerFactor");
        file.close();
        DEBUG_PRINTLN("CSV header written.");
      } else {
        DEBUG_PRINTLN("Failed to create file header.");
      }
    }
    else {
      sd_status = false;
      DEBUG_PRINTLN("SD card Failed");
    }

    // Button setup
    pinMode(WIFI_RESET_BUTTON_PIN, INPUT_PULLUP);

    // Set up MQTT client
    client.setServer(mqtt_server, 1883);
    client.setCallback(mqttCallback);

    // Create tasks
    xTaskCreatePinnedToCore(networkTask, "Network Task", 10*1024, NULL, 1, &networkTaskHandle, 0);
    xTaskCreatePinnedToCore(mainTask, "Main Task", 32*1024, NULL, 1, &mainTaskHandle, 1);
    xTaskCreatePinnedToCore(wifiResetTask, "WiFi Reset Task", 4*1024, NULL, 1, &wifiResetTaskHandle, 1);
}

// Check for WiFi reset button press in loop
void loop() {
}