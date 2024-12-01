// Start Library Include section //
// ---------------------------- //
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>  // WiFiManager library
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <SD.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP32_FTPClient.h>
#include <FastLED.h>

// End Library Include section //
// ---------------------------- //


//Configuration Section Start
//-------------------------//

// Debug mode Config
#define DEBUG_MODE true
#define DEBUG_PRINT(x)  if (DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x) if (DEBUG_MODE) { Serial.println(x); }

// Device Config
#define DEVICE_ID "1191012412010001"
#define HB_INTERVAL 1*60*1000
#define DATA_INTERVAL 5*60*1000

// RS485 Config
#define MAX485_DE_RE 27
#define RS485_RX 16
#define RS485_TX 17

// WiFi and MQTT reconnection time config
#define WIFI_ATTEMPT_COUNT 30
#define WIFI_ATTEMPT_DELAY 1000
#define WIFI_WAIT_COUNT 60
#define WIFI_WAIT_DELAY 1000
#define MAX_WIFI_ATTEMPTS 2
#define MQTT_ATTEMPT_COUNT 10
#define MQTT_ATTEMPT_DELAY 5000

// WiFi and MQTT attempt counters
int wifiAttemptCount = WIFI_ATTEMPT_COUNT;
int wifiWaitCount = WIFI_WAIT_COUNT;
int maxWifiAttempts = MAX_WIFI_ATTEMPTS;
int mqttAttemptCount = MQTT_ATTEMPT_COUNT;

// MQTT Server Config
const char* mqtt_server = "broker2.dma-bd.com";
const char* mqtt_user = "broker2";
const char* mqtt_password = "Secret!@#$1234";
const char* mqtt_topic = "DMA/EnergyMeter/PUB";

//File Name for SD Card
const char* filename = "/energy_data.csv";

// FTP Server Config
#define FTP_SERVER "iot2.dma-bd.com"
#define FTP_USER "dmacam"
#define FTP_PASS "dmabd987!@#$"

// LED Conffig
#define DATA_PIN 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

//End Configuration Section//
//-------------------------//


//Start Making instance Section//
//-----------------------------//

//Wifi and MQTT Instance
WiFiClient espClient;
PubSubClient client(espClient);

//FreeRTOS Task instance
TaskHandle_t networkTaskHandle;
TaskHandle_t mainTaskHandle;
TaskHandle_t wifiResetTaskHandle;

// Modbus instance
ModbusMaster node;

//NTP instance
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 21600, 3600000);

//FTP instance
ESP32_FTPClient ftp(FTP_SERVER, FTP_USER, FTP_PASS, 5000, 2); 

//End Making instance Section//
//-----------------------------//


//Start Variable declaretion Section//
//----------------------------------//

// WiFi Reset Button
#define WIFI_RESET_BUTTON_PIN 35

//WiFi Reset Flag
bool wifiResetFlag = false;

//Heartbeat and Data send interval variable
unsigned long hbLastTime = 0, dataLastTime = 0;

//SD Card status
boolean sd_status = false;

// SD Card Chip Select Pin
#define SD_CS_PIN 5

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

// Modbus data storing Variables
int taeHigh, taeLow, activePower;
int pAvolt, pBvolt, pCvolt;
int lABvolt, lBCvolt, lCAvolt;
int pAcurrent, pBcurrent, pCcurrent;
int frequency, powerFactor;

// FInal Data Storing variable
char em_data[128];

//End Variable declaretion Section//
//--------------------------------//



// Start Function Section //
//-----------------------//

// Modbus Pre-transmission
void preTransmission() {
  digitalWrite(MAX485_DE_RE, HIGH); // Enable Transmit mode
}

// Modbus Post-transmission
void postTransmission() {
  digitalWrite(MAX485_DE_RE, LOW); // Enable Receive mode
}

// Function to read a specific Modbus register
// int readModbusData(uint16_t reg_address) {
//   uint8_t result = node.readHoldingRegisters(reg_address, 1);
//   if (result == node.ku8MBSuccess) {
//     return node.getResponseBuffer(0);
//   } else {
//     return -1; // Error value
//   }
// }

int readModbusData(uint16_t reg_address, uint8_t max_retries) {
  
  vTaskDelay(pdMS_TO_TICKS(30));

  int value = -1;
  while (max_retries > 0) {  // Continue while there are retries left
    uint8_t result = node.readHoldingRegisters(reg_address, 1);
    if (result == node.ku8MBSuccess) {
      value = node.getResponseBuffer(0);
      DEBUG_PRINT("Trying to get data from Modbus: ");
      DEBUG_PRINT(reg_address);
      DEBUG_PRINT(": ");
      DEBUG_PRINTLN(value);
      break; // Exit the loop if a valid value is read
    }
    max_retries--;  // Decrease the retry count
    vTaskDelay(pdMS_TO_TICKS(60)); // Small delay between retries (100ms)
  }

  return value; // Return the value, -1 if all retries failed
}


// Getting Modbus Data
void GetModbusData() {
  // Read Modbus data with specific retry counts for each field
  taeHigh = readModbusData(taeHigh_reg_addr, 3);       // Retry up to 3 times
  taeLow = readModbusData(taeLow_reg_addr, 3);         // Retry up to 3 times
  activePower = readModbusData(activePower_reg_addr, 2); // Retry up to 2 times
  pAvolt = readModbusData(pAvolt_reg_addr, 1);         // Retry up to 1 times
  pBvolt = readModbusData(pBvolt_reg_addr, 1);         // Retry up to 1 times
  pCvolt = readModbusData(pCvolt_reg_addr, 1);         // Retry up to 2 times
  lABvolt = readModbusData(lABvolt_reg_addr, 1);       // Retry up to 1 time
  lBCvolt = readModbusData(lBCvolt_reg_addr, 1);       // Retry up to 1 time
  lCAvolt = readModbusData(lCAvolt_reg_addr, 1);       // Retry up to 1 time
  pAcurrent = readModbusData(pAcurrent_reg_addr, 1);   // Retry up to 2 times
  pBcurrent = readModbusData(pBcurrent_reg_addr, 1);   // Retry up to 2 times
  pCcurrent = readModbusData(pCcurrent_reg_addr, 1);   // Retry up to 2 times
  frequency = readModbusData(frequency_reg_addr, 1);   // Retry up to 1 time
  powerFactor = readModbusData(powerfactor_reg_addr, 1); // Retry up to 1 time
}


// Parsing Modbus Data
void ParsingModbusData() {
  // Format the data into the buffer with DEVICE_ID at the beginning
  snprintf(em_data, sizeof(em_data), 
           "%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
           DEVICE_ID,  // Add DEVICE_ID to the formatted string
           taeHigh,
           taeLow,
           activePower,
           pAvolt,
           pBvolt,
           pCvolt,
           lABvolt,
           lBCvolt,
           lCAvolt,
           pAcurrent,
           pBcurrent,
           pCcurrent,
           frequency,
           powerFactor);
}

// Function to reconnect to WiFi
void reconnectWiFi() {
  leds[0] = CRGB::Red;
  FastLED.show();

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiAttemptCount > 0) {
      DEBUG_PRINTLN("Attempting WiFi connection...");
      WiFi.begin();  // Use saved credentials
      wifiAttemptCount--;
      DEBUG_PRINTLN("Remaining WiFi attempts: " + String(wifiAttemptCount));
      // vTaskDelay(WIFI_ATTEMPT_DELAY / portTICK_PERIOD_MS);
      vTaskDelay(pdMS_TO_TICKS(WIFI_ATTEMPT_DELAY));
    } else if (wifiWaitCount > 0) {
      wifiWaitCount--;
      DEBUG_PRINTLN("WiFi wait... retrying in a moment");
      DEBUG_PRINTLN("Remaining WiFi wait time: " + String(wifiWaitCount) + " seconds");
      vTaskDelay(pdMS_TO_TICKS(WIFI_WAIT_DELAY));
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

// Function to reconnect to MQTT with a unique client ID
void reconnectMQTT() {
  if (!client.connected()) {
    leds[0] = CRGB::Yellow;
    FastLED.show();

    char clientId[16];  // 1 byte for "dma_em_" + 8 bytes for random hex + null terminator
    snprintf(clientId, sizeof(clientId), "dma_em_%04X%04X", random(0xffff), random(0xffff));

    if (mqttAttemptCount > 0) {
      DEBUG_PRINTLN("Attempting MQTT connection...");
      
      if (client.connect(clientId, mqtt_user, mqtt_password)) {  // Use the unique client ID
        DEBUG_PRINTLN("MQTT connected");
        DEBUG_PRINT("Client_ID: ");
        DEBUG_PRINTLN(clientId);

        leds[0] = CRGB::Black;
        FastLED.show();
        
        char topic[48];
        snprintf(topic, sizeof(topic), "%s/%s", mqtt_topic, DEVICE_ID);
        client.subscribe(topic);
        
      } else {
        DEBUG_PRINTLN("MQTT connection failed");
        DEBUG_PRINTLN("Remaining MQTT attempts: " + String(mqttAttemptCount));
        mqttAttemptCount--;
        vTaskDelay(pdMS_TO_TICKS(MQTT_ATTEMPT_DELAY));
      }
    } else {
      DEBUG_PRINTLN("Max MQTT attempts exceeded, restarting...");
      ESP.restart();
    }
  }
}

// MQTT callback function
void sendToFtp();
void clearSDCard();
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Print the topic and message for debugging
  DEBUG_PRINTLN("Message arrived on topic: " + String(topic));
  DEBUG_PRINTLN("Message content: " + message);
  leds[0] = CRGB::Blue;
  FastLED.show();
  vTaskDelay(pdMS_TO_TICKS(500));
  leds[0] = CRGB::Black;
  FastLED.show();

  // Check if the message is "get_from_sd_card"
  if (message == "get_data_from_sd_card") {
    DEBUG_PRINTLN("Triggering sendToFtp()...");
    sendToFtp();
  }

  // Check if the message is "get_from_sd_card"
  if (message == "clear_sd_card") {
    DEBUG_PRINTLN("Triggering sendToFtp()...");
    clearSDCard();  // Call the function to upload the file
  }
}


// Send Data from SD Card to FTP Server
void sendToFtp(){
  // Convert String to const char*
  char ftpFileName[40];  // Buffer for file name
  snprintf(ftpFileName, sizeof(ftpFileName), "%s_em_data_%04X.csv", DEVICE_ID, random(0xFFFF));

  // Print the generated file name for debugging
  DEBUG_PRINTLN(ftpFileName);

  // Connect to FTP server
  ftp.OpenConnection();
  DEBUG_PRINTLN("Connected to FTP server");

  // Change working directory on the FTP server
  ftp.ChangeWorkDir("/home/dmacam/intrusion/LOS_FILE_TEST");
  DEBUG_PRINTLN("Directory changed");

  // Open the file from SD card
  File file = SD.open(filename);
  if (!file) {
    DEBUG_PRINTLN("Failed to open file on SD card");
    leds[0] = CRGB::Red;
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(1000));
    leds[0] = CRGB::Black;
    FastLED.show();
    ftp.CloseConnection();

    return;
  }

  ftp.InitFile("Type A");
  ftp.NewFile(ftpFileName);
  byte buffer[128];  //in this part it will read the file bit my bit then upload it
  while (file.available()) {
    int bytesRead = file.read(buffer, sizeof(buffer));
    ftp.WriteData(buffer, bytesRead);  // Write data to FTP server
  }
  DEBUG_PRINTLN("File uploaded successfully");
  leds[0] = CRGB::Green;
  FastLED.show();
  vTaskDelay(pdMS_TO_TICKS(1000));
  leds[0] = CRGB::Black;
  FastLED.show();

  ftp.CloseFile();  // Close the file transfer
  file.close();
  ftp.CloseConnection();
}

// Clear the SD Card Data
void clearSDCard(){
  //Clear SD Card
  if (SD.begin()) {
    File file = SD.open(filename, FILE_WRITE);
    if (file) {
        file.println("AOS,timeStamp,Device_ID,taeHigh,taeLow,ActivePower,PhaseA_V,PhaseB_V,PhaseC_V,LineAB_V,LineBC_V,LineCA_V,PhaseA_C,PhaseB_C,PhaseC_C,Frequency,PowerFactor");
      file.close();
      DEBUG_PRINTLN("CSV header written after FTP");
      leds[0] = CRGB::Green;
      FastLED.show();
      vTaskDelay(pdMS_TO_TICKS(1000));
      leds[0] = CRGB::Black;
      FastLED.show();
    } else {
      DEBUG_PRINTLN("Failed to create Header After FTP");
      leds[0] = CRGB::Red;
      FastLED.show();
      vTaskDelay(pdMS_TO_TICKS(500));
      leds[0] = CRGB::Black;
      FastLED.show();
    }
  }
}


// End Function Section //
//----------------------//


// Start FreeRTOS Task Section //
//----------------------//

// Network Task for WiFi and MQTT
void networkTask(void *param) {
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  // Initialize timeClient only once
  bool timeClientInitialized = false;

  for (;;) {
    // Check WiFi connection
    if (WiFi.status() == WL_CONNECTED) {
      // Initialize timeClient only once
      if (!timeClientInitialized) {
        timeClient.begin();  // Start timeClient after WiFi is connected
        vTaskDelay(pdMS_TO_TICKS(1000));
      // Update time from NTP server
        timeClient.update();
        timeClientInitialized = true;
      }


      // Check and reconnect MQTT if necessary
      if (!client.connected()) {
        reconnectMQTT();
      }
    } else {
      // Reconnect WiFi if disconnected
      reconnectWiFi();
    }

    // Loop MQTT client for processing incoming messages
    client.loop();

    // Delay for 100ms before next cycle
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


// Task for WiFi reset and WiFiManager setup
void wifiResetTask(void *param) {
  for (;;) {
    // Check if button is pressed (LOW state)
    if (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
      unsigned long pressStartTime = millis(); // Record the time when the button is pressed
      DEBUG_PRINTLN("Button Pressed....");

      // Wait for at least 5 seconds to confirm long press
      while (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
        // Check if button is still pressed after 5 seconds
        if (millis() - pressStartTime >= 5000) {
          DEBUG_PRINTLN("5 seconds holding time reached, starting WiFiManager...");

          leds[0] = CRGB::Green;
          FastLED.show();

          // Suspend other tasks to avoid conflict
          vTaskSuspend(networkTaskHandle);
          vTaskSuspend(mainTaskHandle);

          DEBUG_PRINTLN("Starting WiFiManager for new WiFi setup...");
          WiFiManager wifiManager;
          wifiManager.resetSettings();  // Clear previous settings
          wifiManager.autoConnect("DMA_EnergyMeter"); // Start AP for new configuration

          DEBUG_PRINTLN("New WiFi credentials set, restarting...");
          ESP.restart();  // Restart after WiFi configuration
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to avoid overwhelming the system
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100 ms
  }
}


/*********************************************************************/
/*                                  main                             */
/*********************************************************************/

void mainTask(void *param) {
  for (;;) {

    // Get the current time's epoch
    unsigned long hbNow = millis();
    if (hbNow - hbLastTime > HB_INTERVAL) {
        hbLastTime = hbNow;

        if (client.connected()) {
            char hb_data[50];  // Buffer for the heartbeat data

            // Format the heartbeat message into the buffer
            snprintf(hb_data, sizeof(hb_data), "%s,W:1,G:0,C:1,SD:%d", DEVICE_ID, sd_status);

            // Publish the heartbeat message
            client.publish(mqtt_topic, hb_data);
            DEBUG_PRINTLN("Heartbeat published data to mqtt");
            leds[0] = CRGB::Blue;
            FastLED.show();
            vTaskDelay(pdMS_TO_TICKS(1000));
            leds[0] = CRGB::Black;
            FastLED.show();
        } else {
            DEBUG_PRINTLN("Failed to publish Heartbeat on MQTT");
        }
    }

    // Get the current epoch time for timestamp
    unsigned long epochTime = timeClient.getEpochTime(); // Epoch time
    struct tm *ptm = gmtime((time_t *)&epochTime);       // Convert to tm structure

    // Format the timestamp into the provided buffer
    char timestamp[20];
    snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d",
            ptm->tm_year + 1900,
            ptm->tm_mon + 1,
            ptm->tm_mday,
            ptm->tm_hour,
            ptm->tm_min,
            ptm->tm_sec);


    unsigned long dataNow = millis();
    // Check if the current minute is 0 (top of the hour)
    if (ptm->tm_min == 0) {
        // Send data every hour (when minute == 0)
        if (dataNow - dataLastTime > 60000) {  // 1 minute interval for top of the hour
            dataLastTime = dataNow;

            // Process data
            if (WiFi.status() == WL_CONNECTED) {
                timeClient.update(); // Update time from NTP server
            }

            DEBUG_PRINTLN(timestamp); // Print the timestamp directly (no need to convert to String)

            GetModbusData();  // Get Modbus data
            ParsingModbusData();  // Parse Modbus data into em_data
            
            // Determine if WiFi and MQTT are connected (aos = 1 if both are connected)
            boolean aos = (WiFi.status() == WL_CONNECTED && client.connected()) ? true : false;

            // Write data to SD card, including aos status
            if (SD.begin()) {
                File file = SD.open(filename, FILE_APPEND);
                if (file) {
                    file.print(aos); // Write the aos status in the first column
                    file.print(","); 
                    file.print(timestamp); 
                    file.print(","); 
                    file.println(em_data);  // Use println() to end the line
                    file.close();
                    sd_status = true;
                    DEBUG_PRINTLN("Data written successfully to SD card.");
                } else {
                    sd_status = false;
                    DEBUG_PRINTLN("Error opening file for writing.");
                }
            } else {
                DEBUG_PRINTLN("SD card initialization failed.");
            }

            // Send data via MQTT
            if (client.connected()) {
                DEBUG_PRINTLN(em_data);
                client.publish(mqtt_topic, em_data);  // Use the global MQTT topic

                DEBUG_PRINTLN("Data published to mqtt");
                leds[0] = CRGB::Green;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(1000));
                leds[0] = CRGB::Black;
                FastLED.show();
            } else {
                DEBUG_PRINTLN("Failed to publish data on MQTT");
            }
        }
    } else {
        // For all other times, send data every DATA_INTERVAL
        if (dataNow - dataLastTime > DATA_INTERVAL) {
            dataLastTime = dataNow;

            // Process data
            if (WiFi.status() == WL_CONNECTED) {
                timeClient.update(); // Update time from NTP server
            }

            DEBUG_PRINTLN(timestamp); // Print the timestamp directly (no need to convert to String)

            GetModbusData();  // Get Modbus data
            ParsingModbusData();  // Parse Modbus data into em_data

            // Determine if WiFi and MQTT are connected (aos = 1 if both are connected)
            boolean aos = (WiFi.status() == WL_CONNECTED && client.connected()) ? true : false;

            // Write data to SD card, including aos status
            if (SD.begin()) {
                File file = SD.open(filename, FILE_APPEND);
                if (file) {
                    file.print(aos); // Write the aos status in the first column
                    file.print(","); 
                    file.print(timestamp); 
                    file.print(","); 
                    file.println(em_data);  // Use println() to end the line
                    file.close();
                    sd_status = true;
                    DEBUG_PRINTLN("Data written successfully to SD card.");
                } else {
                    sd_status = false;
                    DEBUG_PRINTLN("Error opening file for writing.");
                }
            } else {
                DEBUG_PRINTLN("SD card initialization failed.");
            }

            // Send data via MQTT
            if (client.connected()) {
                DEBUG_PRINTLN(em_data);
                client.publish(mqtt_topic, em_data);  // Use the global MQTT topic

                DEBUG_PRINTLN("Data published to mqtt");
                leds[0] = CRGB::Green;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(1000));
                leds[0] = CRGB::Black;
                FastLED.show();
            } else {
                DEBUG_PRINTLN("Failed to publish data on MQTT");
            }
        }
    }

    // Additional task (optional, e.g., print debug message every second)
    // DEBUG_PRINTLN(timestamp);
    // DEBUG_PRINTLN("Hello");
    vTaskDelay(pdMS_TO_TICKS(1000));  // Print "Hello" every second
  }
}


void setup() {
  // Serial Monitor buad rate
  Serial.begin(115200);

  Serial.print("Device ID: ");
  Serial.println(DEVICE_ID);
  delay(1000);

  // LED setup
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  // Initialize RS485 pins
  pinMode(MAX485_DE_RE, OUTPUT);
  digitalWrite(MAX485_DE_RE, LOW);

  // Initialize Serial2 for RS485 communication
  Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);

  // Initialize ModbusMaster instance
  node.begin(1, Serial2); // Slave ID 1
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // timeClient.begin();

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    DEBUG_PRINTLN("SD Card initializing...");
    // while (1); // Halt execution if SD card initialization fails
    for (int i=5; i>0; i--) {
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
      file.println("AOS,timeStamp,Device_ID,taeHigh,taeLow,ActivePower,PhaseA_V,PhaseB_V,PhaseC_V,LineAB_V,LineBC_V,LineCA_V,PhaseA_C,PhaseB_C,PhaseC_C,Frequency,PowerFactor");
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
  xTaskCreatePinnedToCore(networkTask, "Network Task", 8*1024, NULL, 1, &networkTaskHandle, 0);
  xTaskCreatePinnedToCore(mainTask, "Main Task", 16*1024, NULL, 1, &mainTaskHandle, 1);
  xTaskCreatePinnedToCore(wifiResetTask, "WiFi Reset Task", 4*1024, NULL, 1, &wifiResetTaskHandle, 1);
}

// Loop function
void loop() {
}




