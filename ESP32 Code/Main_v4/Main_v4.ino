#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RTClib.h>

// Pins for the 2 sensors using internal ESP32 ADCs
const int SENSOR_PIN_1 = 34; // GPIO 34
const int SENSOR_PIN_2 = 35; // GPIO 35
const int REF_OUT_PIN = 16;
const int INDICATOR_PIN = 17;

// HW-125 SD Card Chip Select Pin
const int SD_CS_PIN = 5; 
char filename[32]; // Expanded buffer to hold longer timestamp filenames

// Lock-in Parameters (Handled completely on Core 0)
const unsigned long sampleInterval = 1000;     // 1 ms sampling (1000 Hz sampling rate)
const unsigned long modulationInterval = 10000; // 10 ms half-cycle (50 Hz modulation)
unsigned long lastSampleTime = 0;
unsigned long lastToggleTime = 0;
bool refState = false;

// Rolling Average Parameters (Handled completely on Core 0)
const int WINDOW_SIZE = 60;
float buffer1[WINDOW_SIZE];
float buffer2[WINDOW_SIZE];
int bufferIndex = 0;
float runningSum1 = 0.0;
float runningSum2 = 0.0;

// Shared Core-to-Core Variables (Protected by Spinlock)
volatile float secondSum1 = 0.0;
volatile float secondSum2 = 0.0;
volatile unsigned long secondSampleCount = 0;

// FreeRTOS Spinlock for safe core-to-core data sharing
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

// 1-Second Logging Timer (Handled on Core 1)
unsigned long lastSecTime = 0;
const unsigned long oneSecondInterval = 1000000; // 1 second in microseconds

// Hardware Instances
Adafruit_BME280 bme1; // Address 0x76
Adafruit_BME280 bme2; // Address 0x77
RTC_DS3231 rtc;       // Address 0x68
File dataFile;

// Forward declaration of Core 0 High-Speed Task
void Core0HighSpeedTask(void * pvParameters);

void setup() {
  Serial.begin(921600);
  
  pinMode(REF_OUT_PIN, OUTPUT);
  pinMode(INDICATOR_PIN, OUTPUT);
  pinMode(SENSOR_PIN_1, INPUT);
  pinMode(SENSOR_PIN_2, INPUT);

  // Initialize rolling filters
  for (int i = 0; i < WINDOW_SIZE; i++) {
    buffer1[i] = 0.0;
    buffer2[i] = 0.0;
  }

  // Start I2C bus (Default ESP32 pins: SDA=21, SCL=22)
  Wire.begin();

  // 1. Initialize DS3231 RTC FIRST (Needed to name the log file)
  Serial.println("Initializing RTC...");
  if (!rtc.begin()) {
    Serial.println("Could not find RTC DS3231! Check wiring.");
    while (1);
  }
  
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time to compile time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // 2. Initialize BME280 Sensors
  Serial.println("Initializing BME280 Sensors...");
  if (!bme1.begin(0x76, &Wire)) {
    Serial.println("Could not find BME280 Sensor 1 (0x76)!");
    while (1);
  }
  if (!bme2.begin(0x77, &Wire)) {
    Serial.println("Could not find BME280 Sensor 2 (0x77)!");
    while (1);
  }

  // 3. Initialize SD Card
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card initialization failed!");
    while (1);
  }

  // 4. Generate dynamic file name using current RTC date and time
  DateTime now = rtc.now();
  // Format: /log_YYYYMMDD_HHMMSS.txt (e.g., /log_20260707_122752.txt)
  sprintf(filename, "/log_%04d%02d%02d_%02d%02d.txt", 
          now.year(), now.month(), now.day(), 
          now.hour(), now.minute());

  // Safe check: If file somehow exists, append an incrementor
  int fileCount = 0;
  while (SD.exists(filename) && fileCount < 100) {
    sprintf(filename, "/log_%04d%02d%02d_%02d%02d_%d.txt", 
            now.year(), now.month(), now.day(), 
            now.hour(), now.minute(), fileCount);
    fileCount++;
  }

  Serial.print("Creating fresh timestamped log file: ");
  Serial.println(filename);

  // Open the unique file
  dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error creating unique datalog file!");
    while (1);
  }
  
  // Write CSV Header
  dataFile.println("DateTime,Avg_LockIn1,Avg_LockIn2,BME1_Temp_C,BME1_Hum_Pct,BME2_Temp_C,BME2_Hum_Pct");
  dataFile.flush();

  // 5. Spawn the High-Speed Task on Core 0
  xTaskCreatePinnedToCore(
    Core0HighSpeedTask,   /* Function to implement the task */
    "HighSpeedTask",      /* Name of the task */
    4096,                 /* Stack size in words */
    NULL,                 /* Task input parameter */
    3,                    /* Priority of the task (High) */
    NULL,                 /* Task handle */
    0                     /* Core ID (0) */
  );
}

// Core 1 handles this loop (Slow tasks: I2C Reads, SD Writes, Serial output)
void loop() {
  unsigned long currentMicros = micros();

  // Low-Speed Task: Process environmental sensors and Save Data (Every 1 Second)
  if (currentMicros - lastSecTime >= oneSecondInterval) {
    lastSecTime = currentMicros;
    digitalWrite(INDICATOR_PIN, HIGH);
    
    // --- CRITICAL SECTION: Snapshot and reset shared accumulators safely ---
    portENTER_CRITICAL(&dataMux);
    float localSum1 = secondSum1;
    float localSum2 = secondSum2;
    unsigned long localCount = secondSampleCount;

    secondSum1 = 0;
    secondSum2 = 0;
    secondSampleCount = 0;
    portEXIT_CRITICAL(&dataMux);
    // --- END CRITICAL SECTION ---

    // Calculate final averaged Lock-In magnitudes for the past second
    float finalAvg1 = (localCount > 0) ? (localSum1 / localCount) : 0;
    float finalAvg2 = (localCount > 0) ? (localSum2 / localCount) : 0;

    // Read BME280 Data (Takes ~4-6ms, isolated on Core 1)
    float t1 = bme1.readTemperature();
    float h1 = bme1.readHumidity();
    float t2 = bme2.readTemperature();
    float h2 = bme2.readHumidity();

    // Read Real-Time Clock
    DateTime now = rtc.now();
    
    // Format the timestamp for the CSV row
    char timeBuffer[22];
    sprintf(timeBuffer, "%04d-%02d-%02d %02d:%02d:%02d", 
            now.year(), now.month(), now.day(), 
            now.hour(), now.minute(), now.second());

    // Log compiled row to SD Card (Can block up to 100ms, completely safe out here!)
    if (dataFile) {
      dataFile.print(timeBuffer);       dataFile.print(",");
      dataFile.print(finalAvg1, 2);     dataFile.print(",");
      dataFile.print(finalAvg2, 2);     dataFile.print(",");
      dataFile.print(t1, 1);            dataFile.print(",");
      dataFile.print(h1, 1);            dataFile.print(",");
      dataFile.print(t2, 1);            dataFile.print(",");
      dataFile.print(h2, 1);
      dataFile.println();
      
      dataFile.flush(); 
    }

    // Diagnostics print to Serial Monitor
    Serial.print("["); Serial.print(timeBuffer); Serial.print("] ");
    Serial.print("LI1: ");              Serial.print(finalAvg1, 1);
    Serial.print(" | LI2: ");           Serial.print(finalAvg2, 1);
    Serial.print(" | BME1: ");          Serial.print(t1, 1); Serial.print("C ");
    Serial.print(" | BME2: ");          Serial.print(t2, 1); Serial.println("C");
    
    digitalWrite(INDICATOR_PIN, LOW);
  }
}

// Dedicated Core 0 Task: High-Speed Lock-In & Modulation
void Core0HighSpeedTask(void * pvParameters) {
  while (1) {
    unsigned long currentMicros = micros();

    // 1. Generate Modulation Reference Signal (~50 Hz Square Wave)
    if (currentMicros - lastToggleTime >= modulationInterval) {
      refState = !refState;
      digitalWrite(REF_OUT_PIN, refState);
      lastToggleTime = currentMicros;
    }

    // 2. High-speed Asynchronous Sampling (Every 1 ms)
    if (currentMicros - lastSampleTime >= sampleInterval) {
      lastSampleTime = currentMicros;

      int adc1 = analogRead(SENSOR_PIN_1);
      int adc2 = analogRead(SENSOR_PIN_2);
      
      float ac_signal1 = (float)adc1 - 2048.0;
      float ac_signal2 = (float)adc2 - 2048.0;

      float psdValue1 = refState ? ac_signal1 : -ac_signal1;
      float psdValue2 = refState ? ac_signal2 : -ac_signal2;

      // 3. Rolling Average Filter
      runningSum1 -= buffer1[bufferIndex];
      runningSum2 -= buffer2[bufferIndex];
      
      buffer1[bufferIndex] = psdValue1;
      buffer2[bufferIndex] = psdValue2;
      
      runningSum1 += psdValue1;
      runningSum2 += psdValue2;
      
      bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;

      float lockInOutput1 = runningSum1 / WINDOW_SIZE;
      float lockInOutput2 = runningSum2 / WINDOW_SIZE;

      // --- CRITICAL SECTION: Safe update to shared data ---
      portENTER_CRITICAL(&dataMux);
      secondSum1 += lockInOutput1;
      secondSum2 += lockInOutput2;
      secondSampleCount++;
      portEXIT_CRITICAL(&dataMux);
      // --- END CRITICAL SECTION ---
    }

    // Feed FreeRTOS scheduler watchdog by yielding for 1 millisecond
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}