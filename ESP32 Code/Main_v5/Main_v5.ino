#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RTClib.h>

// --- Hardware Pin Allocations ---
const int SENSOR_PIN_1 = 34;    // Analog Input 1
const int SENSOR_PIN_2 = 35;    // Analog Input 2
const int REF_OUT_PIN = 16;     // Reference Signal Output (Demodulation square wave)
const int INDICATOR_PIN = 17;   // Status Indicator LED
const int SD_CS_PIN = 5;        // Chip Select for SD Card Module

// --- Lock-in & Modulation Constants ---
const int WINDOW_SIZE = 800;
const int MODULATION_HALF_PERIOD_MS = 8; // 8 ms half-cycle = 62.5 Hz square wave modulation

// --- Global State Variables ---
char filename[32];
volatile bool refState = false;

// Rolling Average Filter Buffers (Processed safely inside the worker task)
float buffer1[WINDOW_SIZE];
float buffer2[WINDOW_SIZE];
int bufferIndex = 0;
float runningSum1 = 0.0;
float runningSum2 = 0.0;

// Shared variables to bridge worker task to low speed loop()
volatile float secondSum1 = 0.0;
volatile float secondSum2 = 0.0;
volatile unsigned long secondDcSum1 = 0;       
volatile unsigned long secondDcSum2 = 0;       
volatile unsigned long secondSampleCount = 0;   
volatile unsigned long secondDcSampleCount = 0; 

// Spinlock for thread-safe shared variables
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

// FreeRTOS Task Handle for Deferred Processing
TaskHandle_t lockInTaskHandle = NULL;

// 1-Second Logging Timer
unsigned long lastSecTime = 0;
const unsigned long oneSecondInterval = 1000000; 

// Hardware Peripheral Instances
Adafruit_BME280 bme1; // I2C Address 0x76
Adafruit_BME280 bme2; // I2C Address 0x77
RTC_DS3231 rtc;       // I2C Address 0x68
File dataFile;

// Hardware Timer Pointer
hw_timer_t *timer = NULL;

// --- Ultra-Lightweight 1 ms Hardware Timer ISR ---
void IRAM_ATTR onTimer() {
  // 1. Handle Reference Square Wave Modulation (50 Hz)
  static int msCounter = 0;
  msCounter++;
  if (msCounter >= MODULATION_HALF_PERIOD_MS) {
    refState = !refState;
    digitalWrite(REF_OUT_PIN, refState);
    msCounter = 0;
  }

  // 2. Unblock the high-priority worker task via direct notification
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(lockInTaskHandle, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// --- High-Priority Worker Task (Safe context for floats & analog reads) ---
void lockInProcessingTask(void *pvParameters) {
  while (1) {
    // Wait until the 1 ms hardware timer triggers a notification
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // 1. High-Speed Synchronous Sampling (Safe to access core API functions here)
    int adc1 = analogReadMilliVolts(SENSOR_PIN_1);
    int adc2 = analogReadMilliVolts(SENSOR_PIN_2);
    
    // 2. Floating-point calculations (Safe to use the FPU within a task)
    float ac_signal1 = (float)adc1 - 2048.0;
    float ac_signal2 = (float)adc2 - 2048.0;

    float psdValue1 = refState ? ac_signal1 : -ac_signal1;
    float psdValue2 = refState ? ac_signal2 : -ac_signal2;

    runningSum1 -= buffer1[bufferIndex];
    runningSum2 -= buffer2[bufferIndex];
    
    buffer1[bufferIndex] = psdValue1;
    buffer2[bufferIndex] = psdValue2;
    
    runningSum1 += psdValue1;
    runningSum2 += psdValue2;
    
    bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;

    float lockInOutput1 = runningSum1 / WINDOW_SIZE;
    float lockInOutput2 = runningSum2 / WINDOW_SIZE;

    // 3. Export data safely to shared memory variables under a Spinlock
    portENTER_CRITICAL(&dataMux);
    secondSum1 += lockInOutput1;
    secondSum2 += lockInOutput2;
    secondSampleCount++;

    if (!refState) { // DC Baseline: average light when the LED is off
      secondDcSum1 += adc1; 
      secondDcSum2 += adc2;
      secondDcSampleCount++;
    }
    portEXIT_CRITICAL(&dataMux);
  }
}

void setup() {
  Serial.begin(921600);
  
  pinMode(REF_OUT_PIN, OUTPUT);
  pinMode(INDICATOR_PIN, OUTPUT);
  pinMode(SENSOR_PIN_1, INPUT);
  pinMode(SENSOR_PIN_2, INPUT);

  for (int i = 0; i < WINDOW_SIZE; i++) {
    buffer1[i] = 0.0;
    buffer2[i] = 0.0;
  }

  Wire.begin();

  Serial.println("Initializing RTC...");
  if (!rtc.begin()) {
    Serial.println("Could not find RTC DS3231!");
    while (1);
  }
  
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.println("Initializing BME280 Sensors...");
  if (!bme1.begin(0x76, &Wire) || !bme2.begin(0x77, &Wire)) {
    Serial.println("One or more BME280 sensors missing!");
    while (1);
  }

  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card initialization failed!");
    while (1);
  }

  DateTime now = rtc.now();
  sprintf(filename, "/log_%04d%02d%02d_%02d%02d.txt", 
          now.year(), now.month(), now.day(), 
          now.hour(), now.minute());

  int fileCount = 0;
  while (SD.exists(filename) && fileCount < 100) {
    sprintf(filename, "/log_%04d%02d%02d_%02d%02d_%d.txt", 
            now.year(), now.month(), now.day(), 
            now.hour(), now.minute(), fileCount);
    fileCount++;
  }

  dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error creating datalog file!");
    while (1);
  }
  
  dataFile.println("DateTime,Avg_LockIn1,Avg_LockIn2,BME1_Temp_C,BME1_Hum_Pct,BME2_Temp_C,BME2_Hum_Pct,Ambient_DC1_mV,Ambient_DC2_mV");
  dataFile.flush();

  // --- Create High-Priority Worker Task on Core 0 ---
  xTaskCreatePinnedToCore(
    lockInProcessingTask,   /* Task function */
    "LockInTask",           /* Name of task */
    4096,                   /* Stack size in words */
    NULL,                   /* Task input parameter */
    3,                      /* Priority (high priority) */
    &lockInTaskHandle,      /* Task handle */
    0                       /* Pin to Core 0 */
  );

  // --- ESP32 CORE v3.x TIMER INITIALIZATION ---
  timer = timerBegin(1000000);            // 1 MHz base clock (1 tick = 1 microsecond)
  timerAttachInterrupt(timer, &onTimer);  // Attach ISR function
  timerAlarm(timer, 1000, true, 0);       // Trigger alarm every 1000 ticks (1 ms) continuous

  Serial.println("System fully initialized. Lock-in processing running safely via FreeRTOS Task.");
}

// Core 1 executes this lower-priority background loop
void loop() {
  unsigned long currentMicros = micros();

  if (currentMicros - lastSecTime >= oneSecondInterval) {
    lastSecTime = currentMicros;
    digitalWrite(INDICATOR_PIN, HIGH);
    
    // --- CRITICAL SECTION: Snapshot and clear shared variables ---
    portENTER_CRITICAL(&dataMux);
    float localSum1 = secondSum1;
    float localSum2 = secondSum2;
    unsigned long localDcSum1 = secondDcSum1;
    unsigned long localDcSum2 = secondDcSum2;
    unsigned long localCount = secondSampleCount;
    unsigned long localDcCount = secondDcSampleCount;

    secondSum1 = 0;
    secondSum2 = 0;
    secondDcSum1 = 0;
    secondDcSum2 = 0;
    secondSampleCount = 0;
    secondDcSampleCount = 0;
    portEXIT_CRITICAL(&dataMux);
    // --- END CRITICAL SECTION ---

    float finalAvg1 = (localCount > 0) ? (localSum1 / localCount) : 0;
    float finalAvg2 = (localCount > 0) ? (localSum2 / localCount) : 0;
    float finalDc1  = (localDcCount > 0) ? ((float)localDcSum1 / localDcCount) : 0;
    float finalDc2  = (localDcCount > 0) ? ((float)localDcSum2 / localDcCount) : 0;

    float t1 = bme1.readTemperature();
    float h1 = bme1.readHumidity();
    float t2 = bme2.readTemperature();
    float h2 = bme2.readHumidity();

    DateTime now = rtc.now();
    char timeBuffer[22];
    sprintf(timeBuffer, "%04d-%02d-%02d %02d:%02d:%02d", 
            now.year(), now.month(), now.day(), 
            now.hour(), now.minute(), now.second());

    if (dataFile) {
      dataFile.print(timeBuffer);       dataFile.print(",");
      dataFile.print(finalAvg1, 2);     dataFile.print(",");
      dataFile.print(finalAvg2, 2);     dataFile.print(",");
      dataFile.print(t1, 1);            dataFile.print(",");
      dataFile.print(h1, 1);            dataFile.print(",");
      dataFile.print(t2, 1);            dataFile.print(",");
      dataFile.print(h2, 1);            dataFile.print(","); 
      dataFile.print(finalDc1, 1);      dataFile.print(","); 
      dataFile.print(finalDc2, 1);                           
      dataFile.println();
      dataFile.flush(); 
    }

    Serial.print("["); Serial.print(timeBuffer); Serial.print("] ");
    Serial.print("CH1 [AC: "); Serial.print(finalAvg1, 1);
    Serial.print(" | Amb_DC: "); Serial.print(finalDc1, 0);
    Serial.print("mV] || CH2 [AC: "); Serial.print(finalAvg2, 1);
    Serial.print(" | Amb_DC: "); Serial.print(finalDc2, 0);
    Serial.println("mV]");
    
    digitalWrite(INDICATOR_PIN, LOW);
  }
}