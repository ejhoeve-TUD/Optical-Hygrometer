#include <Wire.h>
#include <Adafruit_ADS1X15.h>

//pins
const int REF_OUT_PIN = 14;

// Lock-in Parameters
const unsigned long sampleInterval = 1250;
const unsigned long modulationInterval = 5000; // 1 ms per half-cycle -> ~500 Hz modulation

unsigned long lastSampleTime = 0;
unsigned long lastToggleTime = 0;
bool refState = false;

// Rolling Average Parameters
const int WINDOW_SIZE = 128;  // Must be a power of 2 for optimal speed if optimizing, but 128 is fine
float buffer3[WINDOW_SIZE];
float buffer2[WINDOW_SIZE];
int bufferIndex = 0;
float runningSum3 = 0.0;
float runningSum2 = 0.0;

// Create an instance of the ADS1115
Adafruit_ADS1115 ads;
// SCL in 22, sda in 21
void setup() {
  // Initialize Serial Monitor at 115200 baud (to match standard ESP32 speeds)
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial port to open

  // Set pins
  pinMode(REF_OUT_PIN, OUTPUT);

  for (int i = 0; i < WINDOW_SIZE; i++) {
        buffer3[i] = 0.0;
        buffer2[i] = 0.0;
  }

  Serial.println("Initializing ADS1115...");

  // Initialize I2C on standard ESP32 pins (SDA: 21, SCL: 22)
  // If your board uses different pins, use Wire.begin(SDA_PIN, SCL_PIN);
  Wire.begin();

  // Start the ADS1115
  // The default I2C address is 0x48 (when ADDR pin is connected to GND)
  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS1115! Check your wiring.");
    while (1); // Halt program if sensor is not found
  }

  // Set the Gain (Voltage Range)
  // adsGAIN_TWOTHIRDS: +/- 6.144V (1 bit = 0.1875mV) - Default
  // adsGAIN_ONE:       +/- 4.096V (1 bit = 0.125mV)
  // adsGAIN_TWO:       +/- 2.048V (1 bit = 0.0625mV) <-- Best for 3.3V ESP32 signals
  ads.setGain(GAIN_ONE); 

  Serial.println("ADS1115 Initialized Successfully!");
}

void loop() {
  int16_t adc3;
  int16_t adc2;
  //Get time
  unsigned long currentMicros = micros();
  //float voltage;

  // Read raw 16-bit value from Channel 0 (A0)
  

  // 1. Generate Modulation Reference Signal (~500 Hz Square Wave)
    if (currentMicros - lastToggleTime >= modulationInterval) {
        refState = !refState;
        digitalWrite(REF_OUT_PIN, refState);
        lastToggleTime = currentMicros;
    }

    if (currentMicros - lastSampleTime >= sampleInterval) {

        lastSampleTime = currentMicros;
        // 2. Phase-Sensitive Detection (Sample synchronously right after toggling)
        adc3 = ads.readADC_SingleEnded(3);
        adc2 = ads.readADC_SingleEnded(2);
        
        // Multiply by Reference: +1 when HIGH, -1 when LOW
        float psdValue3 = refState ? adc3 : -adc3;
        float psdValue2 = refState ? adc2 : -adc2;

        // 3. Rolling Average Filter (Low-Pass Filter)
        // Subtract the oldest value from the sum
        runningSum3 -= buffer3[bufferIndex];
        runningSum2 -= buffer2[bufferIndex];
        
        // Put the new value into the buffer
        buffer3[bufferIndex] = psdValue3;
        buffer2[bufferIndex] = psdValue2;
        
        // Add the new value to the sum
        runningSum3 += psdValue3;
        runningSum2 += psdValue2;
        
        // Advance buffer pointer
        bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;

        // Calculate final lock-in output (DC magnitude)
        float lockInOutput3 = runningSum3 / WINDOW_SIZE;
        float lockInOutput2 = runningSum2 / WINDOW_SIZE;

        // Print results to Serial Plotter
        Serial.print("Raw_ADC:");     Serial.print(adc3);
        Serial.print(",");
        Serial.print("LockIn_Out:");  Serial.println(lockInOutput3);
    }

  // Small delay to prevent flooding the serial stream (adjust as needed)
  delay(10); 
}