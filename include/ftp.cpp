#include <WiFi.h>
#include <time.h>
#include <ESP32_FTPClient.h>
#include <SD.h>       // Include SD card library
#include <Arduino.h>

const int chipSelect = 5;  // SD card CS pin
int temptime=0;
bool connection = false;
String timeString = "15-25_09-11-2024.csv";
int CHARGING_STATE=0;
int metter = 588382; // Your integer value
// Convert String to const char*
#define DEVICE_SERIAL_ID                             "0087" //63
#define DEVICE_CODE_UPLOAD_DATE                      "240909"
#define WORK_PACKAGE                                 "114600"
String id =  String(WORK_PACKAGE) + String(DEVICE_CODE_UPLOAD_DATE) +  String(DEVICE_SERIAL_ID) ;


#define WIFI_SSID "DMA-Link3-2Gn"
#define WIFI_PASS "dmabd987"
#define FTP_SERVER "iot2.dma-bd.com"
#define FTP_USER "dmacam"
#define FTP_PASS "dmabd987!@#$"
#define SD_CS_PIN 5  // Chip select pin for SD card (adjust if necessary)

// Create FTP client object
ESP32_FTPClient ftp(FTP_SERVER, FTP_USER, FTP_PASS, 5000, 2); 



// Define NTP server and timezone settings for Dhaka, Bangladesh
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 6 * 3600;  // UTC+6 for Dhaka
const int daylightOffset_sec = 0;     // No daylight saving time

struct tm currentTime;                 // Structure to store the current time
unsigned long lastSyncTime = 0;        // Last sync time (in milliseconds)
const unsigned long syncInterval = 3600000; // 1-hour interval in milliseconds

// Function to check if a year is a leap year
bool isLeapYear(int year) {
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

// Function to get the number of days in a given month
int daysInMonth(int month, int year) {
  switch (month) {
    case 1:  // February
      return isLeapYear(year) ? 29 : 28;
    case 3: case 5: case 8: case 10:  // April, June, September, November
      return 30;
    default:
      return 31;  // All other months
  }
}

// Function to connect to Wi-Fi and synchronize time with NTP server
bool syncTimeWithNTP() {


  if (WiFi.status() == WL_CONNECTED) {
   
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    
    if (getLocalTime(&currentTime)) {
      Serial.println("Time synchronized with NTP");

      return true;
    } else {
      Serial.println("Failed to obtain time from NTP");
    }
  } else {
    Serial.println("WiFi connection failed");
  }

  return false;  // Return false if sync was unsuccessful
}

// Function to increment the current time by one second and adjust for date rollovers
void incrementTime() {
  currentTime.tm_sec++;  // Increment seconds

  if (currentTime.tm_sec >= 60) {
    currentTime.tm_sec = 0;
    currentTime.tm_min++;  // Increment minutes

    if (currentTime.tm_min >= 60) {
      currentTime.tm_min = 0;
      currentTime.tm_hour++;  // Increment hours

      if (currentTime.tm_hour >= 24) {
        currentTime.tm_hour = 0;
        currentTime.tm_mday++;  // Increment day

        int daysInCurrentMonth = daysInMonth(currentTime.tm_mon, currentTime.tm_year + 1900);
        
        if (currentTime.tm_mday > daysInCurrentMonth) {
          currentTime.tm_mday = 1;
          currentTime.tm_mon++;  // Increment month

          if (currentTime.tm_mon >= 12) {
            currentTime.tm_mon = 0;
            currentTime.tm_year++;  // Increment year
          }
        }
      }
    }
  }
}





// Function to print the current time
void printLocalTime() {
  Serial.printf("Current time: %04d-%02d-%02d %02d:%02d:%02d\n",
                currentTime.tm_year + 1900,
                currentTime.tm_mon + 1,
                currentTime.tm_mday,
                currentTime.tm_hour,
                currentTime.tm_min,
                currentTime.tm_sec);


  
}

void setup() {
  Serial.begin(115200);
   WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("connected wifiiiiii");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  else{
    Serial.println("SD card initialized");
    }

  if (syncTimeWithNTP()) {             // Initial sync with NTP if possible
    lastSyncTime = millis();           // Record the successful sync time
  } else {
    Serial.println("Starting without NTP sync");
  }
  CREAT();
}

void loop() {
  incrementTime();                     // Increment time locally each second
  printLocalTime();
  delay(1000);

  // Attempt re-sync with NTP every hour
  if (millis() - lastSyncTime >= syncInterval) {
    if (syncTimeWithNTP()) {
      lastSyncTime = millis();         // Update last sync time if successful
    }
  }

      

    if (temptime == 0) {
      temptime = millis();
    }
    if (millis() - temptime > 10000) {
      char payload[60];
      snprintf(payload, sizeof(payload), "%s,C:%d,S:0,TW:%d,F:100,connected", id.c_str(), CHARGING_STATE, metter);      
      write(payload);
      temptime = 0;
      sendftp();
    }





}


////////////////////////// this part of code is responcible to write inside the filesystem
void write(String payload) {
  String timeString = 
                    (currentTime.tm_hour < 10 ? "0" : "") + String(currentTime.tm_hour) + "-" +
                    (currentTime.tm_min < 10 ? "0" : "") + String(currentTime.tm_min) + "_" + 
                    (currentTime.tm_mday < 10 ? "0" : "") + String(currentTime.tm_mday) + "-" + 
                    (currentTime.tm_mon + 1 < 10 ? "0" : "") + String(currentTime.tm_mon + 1) + "-" + 
                    String(currentTime.tm_year + 1900);

// Append ".csv" to the time string
timeString += payload;

  
  // Write to /log.csv
  File file = SD.open("/log.csv", FILE_APPEND);
  if (file) {
   
   
    file.println(timeString); // Log the input from the serial monitor
    file.close();
    Serial.print("Logged in /log.csv: ");
  } else {
    Serial.println("Error opening /log.csv for writing.");
  }

  // If connection is false, also log in /los.csv
  if (connection == false) {
    File file = SD.open("/los.csv", FILE_APPEND);
    if (file) {
      
      file.println(timeString); // Log the input from the serial monitor
      file.close();
      Serial.print("Logged in /los.csv: ");

    } else {
      Serial.println("Error opening /los.csv for writing.");
    }
  }
}



void CREAT() {
  // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");

  // Check if the /log.csv file exists, if not create it
  if (!SD.exists("/log.csv")) {
    File file = SD.open("/log.csv", FILE_WRITE);
    if (file) {
      //file.println("Press Count, Status"); // Add header to CSV
      file.close();
      Serial.println("/log.csv file created.");
    } else {
      Serial.println("Failed to create /log.csv file!");
    }
  }

  // Check if the /los.csv file exists, if not create it
  if (!SD.exists("/los.csv")) {
    File file = SD.open("/los.csv", FILE_WRITE);
    if (file) {
      //file.println("Press Count, Status"); // Add header to CSV
      file.close();
      Serial.println("/los.csv file created.");
    } else {
      Serial.println("Failed to create /los.csv file!");
    }
  }
}







void sendftp(){

String timeString = 
                    (currentTime.tm_hour < 10 ? "0" : "") + String(currentTime.tm_hour) + "-" +
                    (currentTime.tm_min < 10 ? "0" : "") + String(currentTime.tm_min) + "_" + 
                    (currentTime.tm_mday < 10 ? "0" : "") + String(currentTime.tm_mday) + "-" + 
                    (currentTime.tm_mon + 1 < 10 ? "0" : "") + String(currentTime.tm_mon + 1) + "-" + 
                    String(currentTime.tm_year + 1900);

// Append ".csv" to the time string
timeString += "_";
timeString += id;
timeString += ".csv";

// Convert String to const char*
const char* timeChar = timeString.c_str();

  // Connect to FTP server
  ftp.OpenConnection();
  Serial.println("Connected to FTP server");

  // Change working directory on the FTP server
  ftp.ChangeWorkDir("/home/dmacam/intrusion/LOS_FILE_TEST");
  Serial.println("Directory changed");

  // Open the file from SD card
  File file = SD.open("/los.csv");
  if (!file) {
    Serial.println("Failed to open los.csv on SD card");
    ftp.CloseConnection();
    return;
  }

  ftp.InitFile("Type A");
  ftp.NewFile(timeChar);
  byte buffer[128];  //in this part it will read the file bit my bit then upload it
  while (file.available()) {
    int bytesRead = file.read(buffer, sizeof(buffer));
    ftp.WriteData(buffer, bytesRead);  // Write data to FTP server
  }
  Serial.println("File uploaded successfully");
  ftp.CloseFile();  // Close the file transfer
  file.close();
  ftp.CloseConnection();

}
