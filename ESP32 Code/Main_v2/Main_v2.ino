#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Pins for the 2 sensors using internal ESP32 ADCs
const int SENSOR_PIN_1 = 34; // GPIO 34
const int SENSOR_PIN_2 = 35; // GPIO 35
const int REF_OUT_PIN = 16;
const int INDICATOR_PIN = 17;

// HW-125 SD Card Chip Select Pin
const int SD_CS_PIN = 5; 
char filename[20]; // Will hold names like "/log_000.txt"

// Lock-in Parameters
const unsigned long sampleInterval = 1000;     // 1 ms sampling (1000 Hz sampling rate)
const unsigned long modulationInterval = 10000; // 10 ms half-cycle (50 Hz modulation)
unsigned long lastSampleTime = 0;
unsigned long lastToggleTime = 0;
bool refState = false;

// 1-Second Logging Timer
unsigned long lastSecTime = 0;
const unsigned long oneSecondInterval = 1000000; // 1 second in microseconds

// Rolling Average Parameters
const int WINDOW_SIZE = 60;
float buffer1[WINDOW_SIZE];
float buffer2[WINDOW_SIZE];
int bufferIndex = 0;
float runningSum1 = 0.0;
float runningSum2 = 0.0;

// Variables to hold the running totals for 1-second averaging
float secondSum1 = 0.0;
float secondSum2 = 0.0;
unsigned long secondSampleCount = 0;

// BME280 Instances
Adafruit_BME280 bme1; // Will use address 0x76 (SDO to GND)
Adafruit_BME280 bme2; // Will use address 0x77 (SDO to 3.3V)
File dataFile;

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

  // Initialize BME280 Sensors
  Serial.println("Initializing BME280 Sensors...");
  if (!bme1.begin(0x76, &Wire)) {
    Serial.println("Could not find BME280 Sensor 1 (0x76)! Check SDO wire.");
    while (1);
  }
  if (!bme2.begin(0x77, &Wire)) {
    Serial.println("Could not find BME280 Sensor 2 (0x77)! Check SDO wire.");
    while (1);
  }

  // Initialize SD Card
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card initialization failed!");
    while (1);
  }

  // Find a unique filename
  int fileCount = 0;
  while (fileCount < 1000) {
    // Create names like "/log_000.txt", "/log_001.txt", etc.
    sprintf(filename, "/log_%03d.txt", fileCount);
    
    // If the file does NOT exist, we can safely use this name!
    if (!SD.exists(filename)) {
      break; 
    }
    fileCount++;
  }

  Serial.print("Creating fresh log file: ");
  Serial.println(filename);

  // Open the new unique file
  dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error creating unique datalog file!");
    while (1);
  }
  
  // Write CSV Header to the brand-new file
  dataFile.println("Timestamp_ms,Avg_LockIn1,Avg_LockIn2,BME1_Temp_C,BME1_Hum_Pct,BME2_Temp_C,BME2_Hum_Pct");
  dataFile.flush();
}

void loop() {
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

    // 3. Rolling Average Filter (Cleans phase ripple)
    runningSum1 -= buffer1[bufferIndex];
    runningSum2 -= buffer2[bufferIndex];
    
    buffer1[bufferIndex] = psdValue1;
    buffer2[bufferIndex] = psdValue2;
    
    runningSum1 += psdValue1;
    runningSum2 += psdValue2;
    
    bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;

    float lockInOutput1 = runningSum1 / WINDOW_SIZE;
    float lockInOutput2 = runningSum2 / WINDOW_SIZE;

    // Accumulate current lock-in outputs for the 1-second true math average
    secondSum1 += lockInOutput1;
    secondSum2 += lockInOutput2;
    secondSampleCount++;
  }

  // 4. Low-Speed Task: Process environmental sensors and Save Data (Every 1 Second)
  if (currentMicros - lastSecTime >= oneSecondInterval) {
    lastSecTime = currentMicros;
    digitalWrite(INDICATOR_PIN, HIGH);
    // Calculate final averaged Lock-In magnitudes for the past second
    float finalAvg1 = (secondSampleCount > 0) ? (secondSum1 / secondSampleCount) : 0;
    float finalAvg2 = (secondSampleCount > 0) ? (secondSum2 / secondSampleCount) : 0;

    // Reset 1-second accumulators immediately
    secondSum1 = 0;
    secondSum2 = 0;
    secondSampleCount = 0;

    // Read BME280 Data (Takes ~4-6ms, safe to run out here once a second)
    float t1 = bme1.readTemperature();
    float h1 = bme1.readHumidity();
    float t2 = bme2.readTemperature();
    float h2 = bme2.readHumidity();

    // Log compiled row to SD Card
    if (dataFile) {
      dataFile.print(millis());         dataFile.print(",");
      dataFile.print(finalAvg1, 2);     dataFile.print(",");
      dataFile.print(finalAvg2, 2);     dataFile.print(",");
      dataFile.print(t1, 1);            dataFile.print(",");
      dataFile.print(h1, 1);            dataFile.print(",");
      dataFile.print(t2, 1);            dataFile.print(",");
      dataFile.print(h2, 1);
      dataFile.println();
      
      // Force write to physical storage immediately since it happens only once a second
      dataFile.flush(); 
    }

    // Diagnostics print to Serial Monitor
    Serial.print("Saved Row -> LI1: "); Serial.print(finalAvg1,1);
    Serial.print(" | LI2: ");           Serial.print(finalAvg2,1);
    Serial.print(" | BME1: ");          Serial.print(t1,1); Serial.print("C ");
    Serial.print(" | BME2: ");          Serial.print(t2,1); Serial.println("C");
    digitalWrite(INDICATOR_PIN, LOW);
  }
}