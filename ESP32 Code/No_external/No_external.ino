#include <Arduino.h>

// Pins for the 2 sensors using internal ESP32 ADCs
const int SENSOR_PIN_1 = 34; // GPIO 34 (ADC1_CH6)
const int SENSOR_PIN_2 = 35; // GPIO 35 (ADC1_CH7)
const int REF_OUT_PIN = 16;

// Lock-in Parameters
const unsigned long sampleInterval = 2500;     // 1 ms sampling (1000 Hz sampling rate)
const unsigned long modulationInterval = 25000; // 10 ms half-cycle (50 Hz modulation)

unsigned long lastSampleTime = 0;
unsigned long lastToggleTime = 0;
bool refState = false;

// Rolling Average Parameters
const int WINDOW_SIZE = 60;
float buffer1[WINDOW_SIZE];
float buffer2[WINDOW_SIZE];
int bufferIndex = 0;
float runningSum1 = 0.0;
float runningSum2 = 0.0;

void setup() {
  Serial.begin(921600);
  
  pinMode(REF_OUT_PIN, OUTPUT);
  pinMode(SENSOR_PIN_1, INPUT);
  pinMode(SENSOR_PIN_2, INPUT);

  // Initialize buffers
  for (int i = 0; i < WINDOW_SIZE; i++) {
    buffer1[i] = 0.0;
    buffer2[i] = 0.0;
  }
}

void loop() {
  unsigned long currentMicros = micros();

  // 1. Generate Modulation Reference Signal (~100 Hz Square Wave)
  if (currentMicros - lastToggleTime >= modulationInterval) {
    refState = !refState;
    digitalWrite(REF_OUT_PIN, refState);
    lastToggleTime = currentMicros;
  }

  // 2. High-speed Asynchronous Sampling
  if (currentMicros - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentMicros;

    // Internal analogRead takes ~10 microseconds (completely non-blocking)
    int adc1 = analogRead(SENSOR_PIN_1);
    int adc2 = analogRead(SENSOR_PIN_2);
    
    // Remove mid-point DC offset (12-bit ESP32 ADC goes 0 - 4095, midpoint ~2048)
    float ac_signal1 = (float)adc1 - 2048.0;
    float ac_signal2 = (float)adc2 - 2048.0;

    // Phase-Sensitive Detection (+1 when HIGH, -1 when LOW)
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

    // Calculate final lock-in output (DC magnitudes)
    float lockInOutput1 = runningSum1 / WINDOW_SIZE;
    float lockInOutput2 = runningSum2 / WINDOW_SIZE;

    // Print results to Serial Plotter (Only sensor 1 shown to keep plotter neat)
    Serial.print("Raw_ADC1:");     Serial.print(adc1);
    Serial.print(",");
    Serial.print("Raw_ADC2:");     Serial.print(adc2);
    Serial.print(",");
    Serial.print("LockIn_Out1:");  Serial.print(lockInOutput1);
    Serial.print(",");
    Serial.print("LockIn_Out2:");  Serial.println(lockInOutput2);
  }
  
  // NOTE: Removed the delay(10) from the bottom of your loop. 
  // Any delay() here will break your microsecond timing logic!
}